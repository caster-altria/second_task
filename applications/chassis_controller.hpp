#pragma once

#include <cstdint>

#include "io/dbus/dbus.hpp"

namespace chassis
{
enum class State
{
  DISABLED,
  ARMING,
  ENABLED
};

struct RemoteInput
{
  bool online = false;
  sp::DBusSwitchMode sw_r = sp::DBusSwitchMode::DOWN;
  float ch_lv = 0.0f;
  float ch_lh = 0.0f;
  float ch_lu = 0.0f;
};

struct ChassisCommand
{
  float vx = 0.0f;  // 前后速度，单位: m/s
  float vy = 0.0f;  // 左右速度，单位: m/s
  float wz = 0.0f;  // 旋转角速度，单位: rad/s
};

struct WheelSpeedCommand
{
  float lf = 0.0f;  // 左前轮目标角速度，单位: rad/s
  float lr = 0.0f;  // 左后轮目标角速度，单位: rad/s
  float rf = 0.0f;  // 右前轮目标角速度，单位: rad/s
  float rr = 0.0f;  // 右后轮目标角速度，单位: rad/s
};

class Controller
{
public:
  void update(const RemoteInput & input, uint32_t now_ms);

  const WheelSpeedCommand & wheel_speeds() const;
  const ChassisCommand & command() const;
  State state() const;
  // 更新某个电机的 CAN 反馈数据。
  void update_motor_feedback(uint32_t rx_id, uint8_t * data, uint32_t stamp_ms);

  // 根据目标轮速和实际轮速计算四个电机的控制输出。
  void update_motor_control(uint32_t now_ms);

  // 将四个电机的控制量写入 CAN 数据缓冲区。
  void write_motor_commands(uint8_t * data) const;

  // 获取四个 M3508 共用的控制帧 ID。
  uint16_t motor_tx_id() const;

  // 自动模式：直接下发前进速度，跳过摇杆映射与解锁流程
  void set_auto_speed(float vx);

  // [只读] 实测前进速度，单位 m/s，供里程计积分
  float measured_vx();

private:
  bool sticks_centered(const RemoteInput & input) const;
  // 将[-1, 1]的摇杆输入去除死区并重新缩放到[-1, 1]
  static float apply_deadband(float value);
  // 失能或等待解锁时，底盘速度必须为零
  void clear_command();
  // 将遥控器摇杆转换为底盘速度指令
  void update_command(const RemoteInput & input);
  // 将底盘速度转换为四轮目标角速度
  void update_wheel_speeds();
  void clear_wheel_speeds();

  // 判断四个电机反馈是否都在线。
  bool motors_alive(uint32_t now_ms) const;

  // 清除 PID 状态，并让四个电机输出零力矩。
  void clear_motor_control();

  State state_ = State::DISABLED;
  uint32_t arming_start_ms_ = 0;
  ChassisCommand command_{};
  // 轮速指令
  WheelSpeedCommand wheel_speeds_{};
};

}  // namespace chassis
