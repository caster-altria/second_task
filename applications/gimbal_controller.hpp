#ifndef GIMBAL_CONTROLLER_HPP
#define GIMBAL_CONTROLLER_HPP

#include <cstdint>

#include "tools/pid/pid.hpp"

namespace gimbal
{
enum class State
{
  DISABLED,
  ENABLED
};

struct FoldLimit
{
  float min_rad;
  float max_rad;  // 折叠轴角度限制，单位: rad
};

struct GimbalCommand
{
  float yaw = 0.0f;
  float pitch = 0.0f;
  float roll = 0.0f;  // 三轴目标角度，单位: rad
  float fold = 0.0f;  // 折叠轴目标角度，单位: rad
};

struct GimbalSpeedCommand
{
  float v_yaw = 0.0f;    // 偏航轴目标角速度，单位: rad/s
  float v_pitch = 0.0f;  // 垂直轴目标角速度，单位: rad/s
  float v_roll = 0.0f;   // 横滚轴目标角速度，单位: rad/s
  float v_fold = 0.0f;   // 折叠轴目标角速度，单位: rad/s
};

struct GimbalTorqueCommand
{
  float t_yaw = 0.0f;    // 偏航轴目标力矩，单位: N·m
  float t_pitch = 0.0f;  // 垂直轴目标力矩，单位: N·m
  float t_roll = 0.0f;   // 横滚轴目标力矩，单位: N·m
  float t_fold = 0.0f;   // 折叠轴目标力矩，单位: N·m
};

// yaw 角度限幅，单位 rad；接机构前按实际行程改
constexpr float YAW_ANGLE_MIN_RAD = -1.5f;
constexpr float YAW_ANGLE_MAX_RAD = 1.5f;

// pitch 角度限幅，单位 rad；接机构前按实际行程改
constexpr float PITCH_ANGLE_MIN_RAD = -0.6f;
constexpr float PITCH_ANGLE_MAX_RAD = 0.9f;

// 折叠角度限幅，单位 rad；接机构前按实际行程改
constexpr float FOLD_ANGLE_MIN_RAD = 0.0f;
constexpr float FOLD_ANGLE_MAX_RAD = 1.57f;

// 折叠轴以上电后反馈稳定时的角度为零位；target_angle.fold 及上面的折叠限幅均为相对该零位的角度
extern float fold_zero_rad;
extern bool fold_zero_valid;

// 电机绝对角度 -> 相对零位的角度，已归一化到 (-PI, PI]
float fold_relative_angle(float absolute_angle_rad);


void yaw_detect(float gimbal_angle, float plan_angle, float* fire_ratio);
void pitch_detect(float gimbal_angle, float plan_angle, float* fire_ratio);


// 位置环 PID 与目标角，定义在 gimbal_controller.cpp
extern sp::PID pid_yaw;
extern sp::PID pid_pitch;
extern sp::PID pid_fold;
extern GimbalCommand target_angle;

class GimbalController
{
public:
  // 使能：允许输出力矩
  void enable();

  // 失能：禁止输出力矩
  void disable();

  // [只读] 当前状态
  State state() const;

private:
  State state_ = State::DISABLED;
};

}  // namespace gimbal

#endif  // GIMBAL_CONTROLLER_HPP