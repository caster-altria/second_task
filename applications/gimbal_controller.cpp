#include "gimbal_controller.hpp"

#include <cmath>

#include "cmsis_os.h"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

namespace
{
// 当前任务周期暂定为 1 ms，单位为 s。
constexpr float PID_DT = 0.001f;

// ===== 云台三轴位置环：set 目标角度 rad，fdb 实测角度 rad，out 力矩 N·m =====
constexpr float GIMBAL_KP = 0.0f;
constexpr float GIMBAL_KI = 0.0f;
constexpr float GIMBAL_KD = 0.0f;

// PID 输出单位为 N·m，先设置保守上限。
constexpr float GIMBAL_MAX_OUTPUT = sp::GM6020_MAX_TORQUE;
constexpr float GIMBAL_MAX_INTEGRAL_OUTPUT = 0.0f;

constexpr float GIMBAL_D_FILTER_ALPHA = 1.0f;

// ===== 折叠位置环：set 目标角度 rad，fdb 实测角度 rad，out 力矩 N·m =====
constexpr float FOLD_KP = 0.0f;
constexpr float FOLD_KI = 0.0f;
constexpr float FOLD_KD = 0.0f;

constexpr float FOLD_MAX_OUTPUT = sp::M2006_P36_MAX_TORQUE;
constexpr float FOLD_MAX_INTEGRAL_OUTPUT = 0.0f;

constexpr float FOLD_D_FILTER_ALPHA = 1.0f;

}  // namespace

namespace gimbal
{
void GimbalController::enable() { this->state_ = State::ENABLED; }

void GimbalController::disable() { this->state_ = State::DISABLED; }

State GimbalController::state() const { return this->state_; }

// 三轴速度环 PID
sp::PID pid_yaw(
  PID_DT, GIMBAL_KP, GIMBAL_KI, GIMBAL_KD, GIMBAL_MAX_OUTPUT, GIMBAL_MAX_INTEGRAL_OUTPUT,
  GIMBAL_D_FILTER_ALPHA);

sp::PID pid_pitch(
  PID_DT, GIMBAL_KP, GIMBAL_KI, GIMBAL_KD, GIMBAL_MAX_OUTPUT, GIMBAL_MAX_INTEGRAL_OUTPUT,
  GIMBAL_D_FILTER_ALPHA);

// 折叠位置环 PID
sp::PID pid_fold(
  PID_DT, FOLD_KP, FOLD_KI, FOLD_KD, FOLD_MAX_OUTPUT, FOLD_MAX_INTEGRAL_OUTPUT,
  FOLD_D_FILTER_ALPHA);

gimbal::GimbalCommand target_angle;
}  // namespace gimbal