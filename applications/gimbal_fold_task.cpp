#include <cmath>
#include <cstdint>

#include "auto_straight.hpp"
#include "cmsis_os.h"
#include "gimbal_controller.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/math_tools/math_tools.hpp"

// distance 由 applications/laser_task.cpp 发布，单位为 cm，无效值为 0xFFFF。
// motor_fold 由 applications/gimbal_task.cpp 创建，这里只读取它的角度和在线状态。
// remote 由 applications/uart_task.cpp 创建，这里只读取遥控器在线状态和右拨杆。
// 这些对象不能在本文件中再次定义，否则会与原有任务产生重复对象和控制冲突。
extern uint16_t distance;
extern float chassis_travelled_m;
extern sp::RM_Motor motor_fold;
extern sp::DBus remote;
extern chassis::AutoStraight auto_straight;

// 折叠后复位到零点完成时置 true，下一次折叠开始或失能时清零，供 gimbal_task.cpp 判断是否允许追踪。
volatile bool gimbal_fold_restored = false;

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
constexpr uint16_t FOLD_DISTANCE_CM = 75;

// 折叠到位后延时该时间再复位到零点，单位为 ms。
constexpr uint32_t RESTORE_DELAY_MS = 6000;

// 目标角度与电机反馈角度的误差小于该值时，认为动作到位，单位为 rad。
constexpr float FOLD_ANGLE_TOLERANCE_RAD = 0.05f;

// 折叠或复位超过该时间仍未到位，进入 FAULT，不再追踪原目标，单位为 ms。
constexpr uint32_t FOLD_ACTION_TIMEOUT_MS = 3000;

// 正常角度加上这个偏移量得到折叠目标角度，单位为 rad。
// 0.79 为占位值，尚未完成实机标定；需按实测折叠方向和角度修改。
constexpr float FOLD_ANGLE_OFFSET_RAD = 1.50f;

bool is_fold_angle_reached(float target_angle_rad)
{
  return std::fabs(sp::limit_angle(target_angle_rad - gimbal::fold_relative_angle(motor_fold.angle))) <=
         FOLD_ANGLE_TOLERANCE_RAD;
}

float get_fold_angle(float normal_angle_rad)
{
  // 先按零位（正常角度）加偏移量计算，再套用 gimbal_controller.hpp 中的折叠角度限幅。
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
  const float normal_angle_rad = 0.0f;  // 相对上电零位
  float fold_angle_rad = 0.0f;
  bool entry_armed = true;
  uint32_t folded_time_ms = 0;

  for (;;) {
    const uint32_t now_ms = osKernelSysTick();

    // gimbal_task.cpp 负责失能、清 PID、计算位置环并发送 CAN。
    // 本任务只更新 gimbal::target_angle.fold，不能调用 motor_fold.cmd() 或 write()。
    const bool control_available = remote.is_alive(now_ms) &&
                                   remote.sw_r != sp::DBusSwitchMode::DOWN &&
                                   motor_fold.is_alive(now_ms) && gimbal::fold_zero_valid;

    if (!control_available) {
      // 遥控器失联、右拨杆 DOWN 或电机反馈超时时，不推进折叠流程。
      // 具体的零力矩处理仍由 gimbal_task.cpp 执行。
      // 失能后 gimbal_task 会重新锁存当前角度，状态机同步回到 NORMAL，避免重新使能时目标突变。
      fold_state = FoldState::NORMAL;
      gimbal_fold_restored = false;
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
        // 未触发折叠时持续保持零位，上电使能后由位置环输出力矩锁住该角度。
        gimbal::target_angle.fold = normal_angle_rad;

        // GY-56 检测到入口且已经解除上一次触发锁定时，按零位计算折叠角并开始折叠。
        if (
          entry_armed && measured_distance_cm != INVALID_DISTANCE_CM &&
          measured_distance_cm <= FOLD_DISTANCE_CM) {
          fold_angle_rad = get_fold_angle(normal_angle_rad);
          gimbal::target_angle.fold = fold_angle_rad;
          entry_armed = false;
          gimbal_fold_restored = false;
          action_start_time_ms = now_ms;
          fold_state = FoldState::FOLDING;
        }
        break;

      case FoldState::FOLDING:
        // 目标角交给 gimbal_task.cpp 的折叠位置 PID 执行。
        gimbal::target_angle.fold = fold_angle_rad;

        if (is_fold_angle_reached(fold_angle_rad)) {
          // 从真正折叠到位的时刻开始计时，不能把折叠动作时间算进去。
          folded_time_ms = now_ms;
          fold_state = FoldState::FOLDED;
        }
        else if (now_ms - action_start_time_ms >= FOLD_ACTION_TIMEOUT_MS) {
          fold_state = FoldState::FAULT;
        }
        break;

      case FoldState::FOLDED: {
        // 保持折叠目标，折叠到位后延时 RESTORE_DELAY_MS 再复位。
        gimbal::target_angle.fold = fold_angle_rad;

        if (now_ms - folded_time_ms >= RESTORE_DELAY_MS) {
          gimbal::target_angle.fold = normal_angle_rad;
          action_start_time_ms = now_ms;
          fold_state = FoldState::RESTORING;
        }
        break;
      }

      case FoldState::RESTORING:
        // 延时到达后，发布正常角度，由现有位置 PID 执行复位。
        gimbal::target_angle.fold = normal_angle_rad;

        if (is_fold_angle_reached(normal_angle_rad)) {
          fold_state = FoldState::NORMAL;
          gimbal_fold_restored = true;
        }
        else if (now_ms - action_start_time_ms >= FOLD_ACTION_TIMEOUT_MS) {
          fold_state = FoldState::FAULT;
        }
        break;

      case FoldState::FAULT:
        // 超时异常时把目标锁在当前反馈角，避免 PID 继续追逐不可达位置。
        gimbal::target_angle.fold = gimbal::fold_relative_angle(motor_fold.angle);
        break;
    }
    osDelay(FOLD_TASK_PERIOD_MS);
  }
}
