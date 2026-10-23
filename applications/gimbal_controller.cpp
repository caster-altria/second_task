#include "gimbal_controller.hpp"

#include <cmath>

#include "cmsis_os.h"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

namespace
{
// 当前任务周期暂定为 1 ms，单位为 s。
constexpr float PID_DT = 0.001f;

// ===== 云台yaw位置环：set 目标角度 rad，fdb 实测角度 rad，out 力矩 N·m =====
constexpr float YAW_KP = 1.0f;
constexpr float YAW_KI = 0.0f;
constexpr float YAW_KD = 0.0f;

// PID 输出单位为 N·m，先设置保守上限。
constexpr float YAW_MAX_OUTPUT = sp::GM6020_MAX_TORQUE;
constexpr float YAW_MAX_INTEGRAL_OUTPUT = 0.0f;

constexpr float YAW_D_FILTER_ALPHA = 1.0f;

// ===== 云台三轴位置环：set 目标角度 rad，fdb 实测角度 rad，out 力矩 N·m =====
constexpr float PITCH_KP = 1.0f;
constexpr float PITCH_KI = 0.0f;
constexpr float PITCH_KD = 0.0f;

// PID 输出单位为 N·m，先设置保守上限。
constexpr float PITCH_MAX_OUTPUT = sp::M2006_P36_MAX_TORQUE;
constexpr float PITCH_MAX_INTEGRAL_OUTPUT = 0.0f;

constexpr float PITCH_D_FILTER_ALPHA = 1.0f;

// ===== 折叠位置环：set 目标角度 rad，fdb 实测角度 rad，out 力矩 N·m =====
constexpr float FOLD_KP = 1.0f;
constexpr float FOLD_KI = 0.5f;
constexpr float FOLD_KD = 0.05f;  // 作用于电机角速度 rad/s，起阻尼作用

constexpr float FOLD_MAX_OUTPUT = sp::GM6020_MAX_TORQUE;
constexpr float FOLD_MAX_INTEGRAL_OUTPUT = 0.5f;

constexpr float FOLD_D_FILTER_ALPHA = 1.0f;

}  // namespace

namespace gimbal
{
/*cpp中定义*/
//yaw定义:
void yaw_detect(float gimbal_angle, float plan_angle, float* fire_ratio)
{
  if (fire_ratio == nullptr) {
    return;
  }
  constexpr float alpha = 0.001f;                  // 1ms的freertos对应的滤波系数
  constexpr float threshold = 0.5f / 57.3f;    // 角度转换成弧度

  static uint8_t fire = 0;  

  fire = (fabs(gimbal_angle - plan_angle) < threshold) ? 1u : 0u;

  *fire_ratio = alpha * static_cast<float>(fire) + (1.0f - alpha) * (*fire_ratio);
}

//pitch定义:
void pitch_detect(float gimbal_angle, float plan_angle, float* fire_ratio)
{
  if (fire_ratio == nullptr) {
    return;
  }
  constexpr float alpha = 0.001f;                  // 1ms的freertos对应的滤波系数
  constexpr float threshold = 1.5f / 57.3f;    // 角度转换成弧度

  static uint8_t fire = 0;  

  fire = (fabs(gimbal_angle - plan_angle) < threshold) ? 1u : 0u;

  *fire_ratio = alpha * static_cast<float>(fire) + (1.0f - alpha) * (*fire_ratio);
}
}  // namespace gimbal

namespace gimbal
{
void GimbalController::enable() { this->state_ = State::ENABLED; }

void GimbalController::disable() { this->state_ = State::DISABLED; }

State GimbalController::state() const { return this->state_; }

// 三轴速度环 PID
sp::PID pid_yaw(
  PID_DT, YAW_KP, YAW_KI, YAW_KD, YAW_MAX_OUTPUT, YAW_MAX_INTEGRAL_OUTPUT, YAW_D_FILTER_ALPHA);

sp::PID pid_pitch(
  PID_DT, PITCH_KP, PITCH_KI, PITCH_KD, PITCH_MAX_OUTPUT, PITCH_MAX_INTEGRAL_OUTPUT,
  PITCH_D_FILTER_ALPHA);

// 折叠位置环 PID
sp::PID pid_fold(
  PID_DT, FOLD_KP, FOLD_KI, FOLD_KD, FOLD_MAX_OUTPUT, FOLD_MAX_INTEGRAL_OUTPUT,
  FOLD_D_FILTER_ALPHA, true);

gimbal::GimbalCommand target_angle;

float fold_zero_rad = 0.0f;
bool fold_zero_valid = false;

float fold_relative_angle(float absolute_angle_rad)
{
  return sp::limit_angle(absolute_angle_rad - fold_zero_rad);
}
}  // namespace gimbal
