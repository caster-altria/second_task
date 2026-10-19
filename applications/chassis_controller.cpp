#include "chassis_controller.hpp"

#include <cmath>
#include <cstdint>

#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"

namespace
{
// 底盘几何参数，单位分别为 m、m、m 和无量纲减速比
constexpr float kWheelRadius = 0.077f;
constexpr float kHalfLength = 0.165f;
constexpr float kHalfWidth = 0.185f;
constexpr float kGearRatio = 13.7f;

// 暂定轮序和 C620 ID，确认实际接线后只需修改这里
constexpr uint8_t kMotorIdLf = 1U;
constexpr uint8_t kMotorIdLr = 2U;
constexpr uint8_t kMotorIdRf = 3U;
constexpr uint8_t kMotorIdRr = 4U;

// 遥控器安全参数：摇杆死区和进入 ENABLED 前的回中保持时间
constexpr float kStickDeadband = 0.12f;
constexpr uint32_t kArmingHoldMs = 200U;

// 保守的底盘速度上限，后续实车调试时再调整
constexpr float kMaxVx = 0.20f;
constexpr float kMaxVy = 0.15f;
constexpr float kMaxWz = 0.25f;

// DBUS 摇杆方向与麦轮坐标系方向的暂定修正符号
constexpr float kVyInputSign = -1.0f;
constexpr float kWzInputSign = -1.0f;

// 四个轮子的统一目标角速度上限，单位: rad/s。
constexpr float kMaxWheelSpeed = 3.0f;

// 功率预测模型参数，公式来自《底盘功率控制》附件：
// Pin = Σ(τ * ω) + K1 * Σ(τ²) + K2 * Σ(ω²) + K3
// K1、K2 使用附件给出的初始经验值；K3 和功率上限需要接入实测数据后再替换。
constexpr float kPowerK1 = 2.0f;
constexpr float kPowerK2 = 0.01f;
constexpr float kPowerK3 = 5.0f;      // 暂定静态功率，单位: W
constexpr float kPowerLimit = 60.0f;  // 暂定输入功率上限，单位: W
constexpr float kPowerQuadraticEpsilon = 1.0e-6f;

// 功率模型的参考系必须和拟合 K1、K2 时使用的参考系一致。
// 当前默认使用输出轴量：RM_Motor::speed 是减速后转速，RM_Motor::cmd() 接收减速后力矩。
// 如果后续确认 K1、K2 是按转子侧数据拟合，将这里改成 ROTOR；计算函数会自动换算。
enum class PowerReferenceFrame
{
  OUTPUT_SHAFT,
  ROTOR
};

constexpr PowerReferenceFrame kPowerReferenceFrame = PowerReferenceFrame::OUTPUT_SHAFT;

// 按照当前参考系计算功率模型中的转矩和角速度。
void convert_to_power_reference(
  float output_torque, float output_speed, float & torque, float & speed)
{
  if (kPowerReferenceFrame == PowerReferenceFrame::ROTOR) {
    torque = output_torque / kGearRatio;
    speed = output_speed * kGearRatio;
  }
  else {
    torque = output_torque;
    speed = output_speed;
  }
}

// 根据附件中的二次方程求解统一力矩缩放系数 K。
// 所有轮子的力矩，包括反向制动力，都使用同一个 K，保证预测值和实际发送值一致。
// 返回值限制在 [0, 1]：1 表示不需要限幅，0 表示本周期不发送主动力矩。
float calculate_power_scale(const float torques[4], const float speeds[4])
{
  float torque_speed_sum = 0.0f;
  float torque_square_sum = 0.0f;
  float speed_square_sum = 0.0f;

  // 先把所有输入转换到与 K1、K2 拟合时相同的参考系，再计算三项系数。
  for (int index = 0; index < 4; ++index) {
    float model_torque = 0.0f;
    float model_speed = 0.0f;
    convert_to_power_reference(torques[index], speeds[index], model_torque, model_speed);
    torque_speed_sum += model_torque * model_speed;
    torque_square_sum += model_torque * model_torque;
    speed_square_sum += model_speed * model_speed;
  }

  // τ实际 = K * τPID，代入功率模型后得到：
  // P(K) = a*K² + b*K + c，其中 P(K) <= 0 表示没有超过功率上限。
  const float quadratic = kPowerK1 * torque_square_sum;
  const float linear = torque_speed_sum;
  const float constant = kPowerK2 * speed_square_sum + kPowerK3 - kPowerLimit;

  const auto power_error = [quadratic, linear, constant](float scale) {
    return quadratic * scale * scale + linear * scale + constant;
  };

  // K=1 对应完整发送 PID 力矩；如果没有超限，不进行任何缩放。
  if (power_error(1.0f) <= 0.0f) {
    return 1.0f;
  }

  // 二次项接近零时退化为一次方程，避免数值上除以接近零的数。
  if (quadratic <= kPowerQuadraticEpsilon) {
    if (std::fabs(linear) <= kPowerQuadraticEpsilon) {
      return constant <= 0.0f ? 1.0f : 0.0f;
    }

    const float scale = -constant / linear;
    if (scale >= 0.0f && scale <= 1.0f && power_error(scale) <= 0.0f) {
      return scale;
    }

    return 0.0f;
  }

  const float discriminant = linear * linear - 4.0f * quadratic * constant;
  if (discriminant < 0.0f) {
    return 0.0f;
  }

  // 取较大的根，因为它代表满足功率上限时仍可保留的最大力矩比例。
  const float scale = (-linear + std::sqrt(discriminant)) / (2.0f * quadratic);
  if (!std::isfinite(scale)) {
    return 0.0f;
  }

  const float clamped_scale = std::fmax(0.0f, std::fmin(1.0f, scale));
  return power_error(clamped_scale) <= 0.0f ? clamped_scale : 0.0f;
}

// 根据功率缩放结果处理单个轮子的输出。
// 所有力矩统一使用 K 缩放，包括反向制动力。
float apply_power_scale(float torque, float power_scale) { return torque * power_scale; }

sp::Mecanum mecanum(kWheelRadius, kHalfLength, kHalfWidth, false, false, true, true);

// 四个轮子对应的电机对象
sp::RM_Motor motorLf(kMotorIdLf, sp::RM_Motors::M3508, kGearRatio);
sp::RM_Motor motorLr(kMotorIdLr, sp::RM_Motors::M3508, kGearRatio);
sp::RM_Motor motorRf(kMotorIdRf, sp::RM_Motors::M3508, kGearRatio);
sp::RM_Motor motorRr(kMotorIdRr, sp::RM_Motors::M3508, kGearRatio);

// 当前任务周期暂定为 1 ms，单位为 s。
constexpr float kPidDt = 0.001f;

// 第一次测试只使用 P，积分和微分暂时关闭。
constexpr float kWheelKp = 0.20f;
constexpr float kWheelKi = 0.0f;
constexpr float kWheelKd = 0.0f;

// PID输出单位为 N·m，先设置保守上限。
constexpr float kWheelMaxOut = 0.30f;
constexpr float kWheelMaxIOut = 0.0f;

// kd为0时该参数暂时没有作用。
constexpr float kWheelDAlpha = 1.0f;

sp::PID pidLf(kPidDt, kWheelKp, kWheelKi, kWheelKd, kWheelMaxOut, kWheelMaxIOut, kWheelDAlpha);

sp::PID pidLr(kPidDt, kWheelKp, kWheelKi, kWheelKd, kWheelMaxOut, kWheelMaxIOut, kWheelDAlpha);

sp::PID pidRf(kPidDt, kWheelKp, kWheelKi, kWheelKd, kWheelMaxOut, kWheelMaxIOut, kWheelDAlpha);

sp::PID pidRr(kPidDt, kWheelKp, kWheelKi, kWheelKd, kWheelMaxOut, kWheelMaxIOut, kWheelDAlpha);

}  // namespace

namespace chassis
{
bool Controller::sticks_centered(const RemoteInput & input) const
{
  // 摇杆回中判定
  return std::fabs(input.ch_lv) <= kStickDeadband && std::fabs(input.ch_lh) <= kStickDeadband &&
         std::fabs(input.ch_lu) <= kStickDeadband;
}

float Controller::apply_deadband(float value)
{
  const float magnitude = std::fabs(value);

  // 死区内认为摇杆回中。
  if (magnitude <= kStickDeadband) {
    return 0.0f;
  }

  // 将死区以外的输入重新映射，使刚越过死区时输出仍从零开始
  const float normalized = (magnitude - kStickDeadband) / (1.0f - kStickDeadband);
  return std::copysign(normalized, value);
}

void Controller::clear_command()
{
  command_.vx = 0.0f;
  command_.vy = 0.0f;
  command_.wz = 0.0f;
}

void Controller::update_command(const RemoteInput & input)
{
  // ch_lv控制前后，ch_lh控制横移，ch_lu（左上方拨轮）控制旋转
  const float vx_input = apply_deadband(input.ch_lv);
  const float vy_input = apply_deadband(input.ch_lh);
  const float wz_input = apply_deadband(input.ch_lu);
  // 将遥控器输入映射到底盘速度
  command_.vx = vx_input * kMaxVx;
  command_.vy = kVyInputSign * vy_input * kMaxVy;
  command_.wz = kWzInputSign * wz_input * kMaxWz;
}

// 将四个轮子的目标角速度清零
void Controller::clear_wheel_speeds()
{
  wheel_speeds_.lf = 0.0f;
  wheel_speeds_.lr = 0.0f;
  wheel_speeds_.rf = 0.0f;
  wheel_speeds_.rr = 0.0f;
}

void Controller::update_wheel_speeds()
{
  // 使用麦轮中间件，将底盘速度转换为四轮目标角速度。
  mecanum.calc(command_.vx, command_.vy, command_.wz);

  float max_abs_speed = std::fabs(mecanum.speed_lf);
  max_abs_speed = std::fmax(max_abs_speed, std::fabs(mecanum.speed_lr));
  max_abs_speed = std::fmax(max_abs_speed, std::fabs(mecanum.speed_rf));
  max_abs_speed = std::fmax(max_abs_speed, std::fabs(mecanum.speed_rr));

  // 如果某个轮子超过角速度上线，则四个轮子按同一比例缩小，以免行进方向发生变化
  const float scale = max_abs_speed > kMaxWheelSpeed ? kMaxWheelSpeed / max_abs_speed : 1.0f;

  wheel_speeds_.lf = mecanum.speed_lf * scale;
  wheel_speeds_.lr = mecanum.speed_lr * scale;
  wheel_speeds_.rf = mecanum.speed_rf * scale;
  wheel_speeds_.rr = mecanum.speed_rr * scale;
}

void Controller::update(const RemoteInput & input, uint32_t now_ms)
{
  // 遥控器掉线或右拨杆DOWN时，立即进入失能状态。
  if (!input.online || input.sw_r == sp::DBusSwitchMode::DOWN) {
    state_ = State::DISABLED;
    arming_start_ms_ = 0;
    clear_command();
    clear_wheel_speeds();
    return;
  }

  switch (state_) {
    case State::DISABLED:
      // 摇杆回中后，进入等待解锁状态。
      if (sticks_centered(input)) {
        state_ = State::ARMING;
        arming_start_ms_ = now_ms;
      }
      break;

    case State::ARMING:
      // 等待解锁期间摇杆偏转，取消本次解锁。
      if (!sticks_centered(input)) {
        state_ = State::DISABLED;
        arming_start_ms_ = 0;
      }
      else if (now_ms - arming_start_ms_ >= kArmingHoldMs) {
        state_ = State::ENABLED;
      }
      break;

    case State::ENABLED:
      break;
  }

  // 只有 ENABLED 状态才根据摇杆生成底盘速度。
  if (state_ == State::ENABLED) {
    update_command(input);
  }
  else {
    clear_command();
  }

  // 将 vx、vy、wz 转换为四个轮子的目标角速度。
  update_wheel_speeds();
}

void Controller::update_motor_feedback(uint32_t rx_id, uint8_t * data, uint32_t stamp_ms)
{
  // RM_Motor::read() 负责解析 C620 返回的 8 字节反馈帧，并将速度转换为 rad/s。这里先根据反馈帧 ID 找到对应电机，再把原始数据交给电机中间件处理。
  if (rx_id == motorLf.rx_id) {
    motorLf.read(data, stamp_ms);
  }
  else if (rx_id == motorLr.rx_id) {
    motorLr.read(data, stamp_ms);
  }
  else if (rx_id == motorRf.rx_id) {
    motorRf.read(data, stamp_ms);
  }
  else if (rx_id == motorRr.rx_id) {
    motorRr.read(data, stamp_ms);
  }
  // 其他 CAN 帧不是本底盘的四个 M3508 反馈帧，直接忽略。
}

bool Controller::motors_alive(uint32_t now_ms) const
{
  // 四个轮子必须全部在 RM_Motor 中间件规定的超时时间内收到反馈，才允许继续输出力矩。只要有一个电机掉线，就进入安全停机逻辑。
  return motorLf.is_alive(now_ms) && motorLr.is_alive(now_ms) && motorRf.is_alive(now_ms) &&
         motorRr.is_alive(now_ms);
}

void Controller::clear_motor_control()
{
  // 清除 PID 的历史误差和积分，避免失能后重新启动时继承旧的控制量。
  pidLf.clear();
  pidLr.clear();
  pidRf.clear();
  pidRr.clear();

  // RM_Motor::cmd() 的输入单位是 N·m。这里给每个电机写入零力矩，后续 write() 会把零力矩转换为 C620 需要的原始电流命令。
  motorLf.cmd(0.0f);
  motorLr.cmd(0.0f);
  motorRf.cmd(0.0f);
  motorRr.cmd(0.0f);
}

void Controller::update_motor_control(uint32_t now_ms)
{
  // 只有遥控器状态机已经进入 ENABLED，且四个电机都在线时，才允许执行 PID。其余情况统一清 PID 并输出零力矩。
  if (state_ != State::ENABLED || !motors_alive(now_ms)) {
    clear_motor_control();
    return;
  }

  // 目标轮速和 RM_Motor::speed 的单位都是 rad/s，因此可以直接作为sp::PID 的 set 和 feedback 输入，不需要再手动换算 RPM 或编码器数值。
  pidLf.calc(wheel_speeds_.lf, motorLf.speed);
  pidLr.calc(wheel_speeds_.lr, motorLr.speed);
  pidRf.calc(wheel_speeds_.rf, motorRf.speed);
  pidRr.calc(wheel_speeds_.rr, motorRr.speed);

  // PID 输出是期望转矩，RM_Motor::speed 是电机输出轴角速度。
  // 两者都来自 RM_Motor 中间件约定的单位，可以直接代入附件中的功率预测公式。
  const float requested_torques[4] = {pidLf.out, pidLr.out, pidRf.out, pidRr.out};
  const float measured_speeds[4] = {
    motorLf.speed,
    motorLr.speed,
    motorRf.speed,
    motorRr.speed,
  };

  // 功率超限时，四个轮子的全部力矩统一乘同一个 K，包括反向制动力。
  const float power_scale = calculate_power_scale(requested_torques, measured_speeds);

  // 先经过功率限制，再交给 RM_Motor::cmd() 转换成 C620 原始电流命令。
  motorLf.cmd(apply_power_scale(requested_torques[0], power_scale));
  motorLr.cmd(apply_power_scale(requested_torques[1], power_scale));
  motorRf.cmd(apply_power_scale(requested_torques[2], power_scale));
  motorRr.cmd(apply_power_scale(requested_torques[3], power_scale));
}

void Controller::write_motor_commands(uint8_t * data) const
{
  // RM_Motor::write() 只负责把已经设置好的力矩命令写入 CAN 8 字节数据区，不在底盘代码中手动拼接 C620 的高低字节。
  motorLf.write(data);
  motorLr.write(data);
  motorRf.write(data);
  motorRr.write(data);
}

uint16_t Controller::motor_tx_id() const
{
  // 四个 ID 为 1~4 的 M3508 都使用同一个标准控制帧 ID 0x200；具体 ID 计算由 RM_Motor 中间件完成，这里直接读取电机对象的 tx_id。
  return motorLf.tx_id;
}

const ChassisCommand & Controller::command() const { return command_; }

const WheelSpeedCommand & Controller::wheel_speeds() const { return wheel_speeds_; }

State Controller::state() const { return state_; }

void Controller::set_auto_speed(float vx)
{
  this->state_ = State::ENABLED;                                  // 自动模式直接使能，不等回中
  this->command_.vx = std::fmax(-kMaxVx, std::fmin(kMaxVx, vx));  // 与手动共用上限
  this->command_.vy = 0.0f;
  this->command_.wz = 0.0f;
  this->update_wheel_speeds();
}

float Controller::measured_vx()
{
  // Mecanum::update() 只写 vx/vy/wz，不覆盖 speed_*，可与本周期 calc() 共用同一实例
  mecanum.update(motorLf.speed, motorLr.speed, motorRf.speed, motorRr.speed);
  return mecanum.vx;
}

}  // namespace chassis
