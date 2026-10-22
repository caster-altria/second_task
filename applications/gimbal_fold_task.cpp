#include <cmath>
#include <cstdint>

#include "auto_straight.hpp"
#include "cmsis_os.h"
#include "gimbal_controller.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"

// distance 由 applications/laser_task.cpp 发布，单位为 cm，无效值为 0xFFFF。
// motor_fold 由 applications/gimbal_task.cpp 创建，这里只读取它的角度和在线状态。
// remote 由 applications/uart_task.cpp 创建，这里只读取遥控器在线状态和右拨杆。
// 这些对象不能在本文件中再次定义，否则会与原有任务产生重复对象和控制冲突。
extern uint16_t distance;
extern float chassis_travelled_m;
extern sp::RM_Motor motor_fold;
extern sp::DBus remote;
extern chassis::AutoStraight auto_straight;

namespace
{
enum class FoldState : uint8_t
{
  NORMAL,
  FOLDING,
  FOLDED,
  RESTORING,
  FAULT
};

constexpr uint32_t FOLD_TASK_PERIOD_MS = 10;

// 使用 0xFFFF 表示无效距离。
constexpr uint16_t INVALID_DISTANCE_CM = 0xFFFF;

// GY-56 测到这个距离以内时，认为云台前端到达隧道入口，单位为 cm。
constexpr uint16_t FOLD_DISTANCE_CM = 100;

// 折叠到位后，读取自动模式已经行驶的距离；超过该值后开始复位，单位为 m。
// 该值要根据云台折叠到位点到安全复位点的实际距离调整。
constexpr float RESTORE_TRAVEL_DISTANCE_M = 0.20f;

// 目标角度与电机反馈角度的误差小于该值时，认为动作到位，单位为 rad。
constexpr float FOLD_ANGLE_TOLERANCE_RAD = 0.05f;

// 折叠或复位超过该时间仍未到位，进入 FAULT，不再追踪原目标，单位为 ms。
constexpr uint32_t FOLD_ACTION_TIMEOUT_MS = 3000;

// 正常角度加上这个偏移量得到折叠目标角度，单位为 rad。
// 当前尚未完成实机标定，暂时为 0；实测后再填写实际折叠方向和角度。
constexpr float FOLD_ANGLE_OFFSET_RAD = 0.0f;

bool is_fold_angle_reached(float target_angle_rad)
{
  return std::fabs(target_angle_rad - motor_fold.angle) <= FOLD_ANGLE_TOLERANCE_RAD;
}

float get_fold_angle(float normal_angle_rad)
{
  // 先按正常角度加偏移量计算，再套用 gimbal_controller.hpp 中的折叠角度限幅。
  const float requested_angle_rad = normal_angle_rad + FOLD_ANGLE_OFFSET_RAD;
  return std::fmax(
    gimbal::FOLD_ANGLE_MIN_RAD, std::fmin(requested_angle_rad, gimbal::FOLD_ANGLE_MAX_RAD));
}

}  // namespace

extern "C" void gimbal_fold_task(void const * argument)
{
  (void)argument;

  FoldState fold_state = FoldState::NORMAL;
  uint32_t action_start_time_ms = 0;
  float normal_angle_rad = 0.0f;
  float fold_angle_rad = 0.0f;
  bool entry_armed = true;
  bool auto_restore_enabled = false;
  float auto_distance_at_folded_m = 0.0f;

  for (;;) {
    const uint32_t now_ms = osKernelSysTick();

    // gimbal_task.cpp 负责失能、清 PID、计算位置环并发送 CAN。
    // 本任务只更新 gimbal::target_angle.fold，不能调用 motor_fold.cmd() 或 write()。
    const bool control_available = remote.is_alive(now_ms) &&
                                   remote.sw_r != sp::DBusSwitchMode::DOWN &&
                                   motor_fold.is_alive(now_ms);

    if (!control_available) {
      // 遥控器失联、右拨杆 DOWN 或电机反馈超时时，不推进折叠流程。
      // 具体的零力矩处理仍由 gimbal_task.cpp 执行。
      osDelay(FOLD_TASK_PERIOD_MS);
      continue;
    }

    const uint16_t measured_distance_cm = distance;

    // 红外安装在云台前端，折叠后其方向会变化，因此只用它判断“进入隧道”。
    // 复位不依赖红外；复位完成后，必须先重新测到阈值外距离，才允许下一次折叠。
    if (
      fold_state == FoldState::NORMAL && measured_distance_cm != INVALID_DISTANCE_CM &&
      measured_distance_cm > FOLD_DISTANCE_CM) {
      entry_armed = true;
    }

    switch (fold_state) {
      case FoldState::NORMAL:
        // GY-56 检测到入口且已经解除上一次触发锁定时，记录正常角度并开始折叠。
        if (
          entry_armed && measured_distance_cm != INVALID_DISTANCE_CM &&
          measured_distance_cm <= FOLD_DISTANCE_CM) {
          normal_angle_rad = motor_fold.angle;
          fold_angle_rad = get_fold_angle(normal_angle_rad);
          gimbal::target_angle.fold = fold_angle_rad;
          entry_armed = false;
          action_start_time_ms = now_ms;
          fold_state = FoldState::FOLDING;
        }
        break;

      case FoldState::FOLDING:
        // 目标角交给 gimbal_task.cpp 的折叠位置 PID 执行。
        gimbal::target_angle.fold = fold_angle_rad;

        if (is_fold_angle_reached(fold_angle_rad)) {
          // 必须从真正折叠到位的时刻开始计算行程，不能把折叠动作时间算进去。
          auto_restore_enabled = auto_straight.active();
          auto_distance_at_folded_m = chassis_travelled_m;
          fold_state = FoldState::FOLDED;
        }
        else if (now_ms - action_start_time_ms >= FOLD_ACTION_TIMEOUT_MS) {
          fold_state = FoldState::FAULT;
        }
        break;

      case FoldState::FOLDED: {
        // 保持折叠目标。自动模式本身由 chassis_task.cpp 驱动，这里只读取它的状态和行程。
        gimbal::target_angle.fold = fold_angle_rad;

        // 不能调用 auto_straight.update()，否则会与 chassis_task.cpp 重复累加行程。
        const bool auto_distance_reached =
          auto_restore_enabled && auto_straight.active() &&
          auto_straight.travelled_m() - auto_distance_at_folded_m >= RESTORE_TRAVEL_DISTANCE_M;

        if (auto_distance_reached) {
          auto_restore_enabled = false;
          gimbal::target_angle.fold = normal_angle_rad;
          action_start_time_ms = now_ms;
          fold_state = FoldState::RESTORING;
        }
        break;
      }

      case FoldState::RESTORING:
        // 自动行程达到阈值后，发布正常角度，由现有位置 PID 执行复位。
        gimbal::target_angle.fold = normal_angle_rad;

        if (is_fold_angle_reached(normal_angle_rad)) {
          fold_state = FoldState::NORMAL;
        }
        else if (now_ms - action_start_time_ms >= FOLD_ACTION_TIMEOUT_MS) {
          fold_state = FoldState::FAULT;
        }
        break;

      case FoldState::FAULT:
        // 超时异常时把目标锁在当前反馈角，避免 PID 继续追逐不可达位置。
        gimbal::target_angle.fold = motor_fold.angle;
        break;
    }
    osDelay(FOLD_TASK_PERIOD_MS);
  }
}
