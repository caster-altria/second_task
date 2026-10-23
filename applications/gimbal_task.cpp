#include <cmath>

#include "cmsis_os.h"
#include "gimbal_controller.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"
#include "trajectory_task.hpp"
#include "io/bmi088/bmi088.hpp"
#include "tools/gimbal/gimbal.hpp"
#include "tools/mahony/mahony.hpp"

sp::RM_Motor motor_fold(2, sp::RM_Motors::GM6020);
sp::RM_Motor motor_yaw(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_pitch(3, sp::RM_Motors::M2006, 36.0f);

gimbal::GimbalController gimbal_controller;
sp::Gimbal gim(0.0f, 0.0f, false, false, 0.001f, 0.0f, 0.0f, sp::GimbalFilterConfig());

extern sp::DBus remote;
extern sp::CAN can2;
extern volatile bool gimbal_fold_restored;
extern sp::BMI088 bmi088;
extern sp::Mahony imu;

float yaw_ratio = 0.0f;
float pitch_ratio = 0.0f;

extern "C" void gimbal_task(void const * argument)
{
  (void)argument;

  // 折叠轴零位锁存：连续 100 个周期（约 100 ms）变化小于 0.01 rad
  constexpr float FOLD_ZERO_STABLE_RAD = 0.01f;
  constexpr uint16_t FOLD_ZERO_STABLE_COUNT = 100;
  float fold_last_angle = 0.0f;
  uint16_t fold_stable_count = 0;

  // IMU 静止检测：去重力后的加速度做带泄漏积分得到速度估计，速度与角速度都小于阈值并持续一段时间视为静止
  constexpr float STILL_DT_S = 0.001f;
  constexpr float STILL_GRAVITY_ALPHA = 0.001f;  // 重力估计的低通系数
  constexpr float STILL_SPEED_LEAK = 0.995f;     // 速度泄漏系数，抑制积分漂移
  constexpr float STILL_SPEED_MPS = 0.05f;       // 速度阈值，单位 m/s
  constexpr float STILL_GYRO_RADPS = 0.05f;      // 角速度阈值，单位 rad/s
  constexpr uint16_t STILL_HOLD_COUNT = 100;     // 连续满足的周期数（约 100 ms）
  float gravity_est[3] = {0.0f, 0.0f, 0.0f};
  float speed_est[3] = {0.0f, 0.0f, 0.0f};
  bool gravity_inited = false;
  uint16_t still_count = 0;
  bool still = false;

  while (true) {
    const uint32_t now_ms = osKernelSysTick();

    gim.update_all_single(imu, motor_yaw.angle, motor_pitch.angle);

    if (!gravity_inited) {
      for (int i = 0; i < 3; i++) gravity_est[i] = bmi088.acc[i];
      gravity_inited = true;
    }
    float speed_sq = 0.0f;
    for (int i = 0; i < 3; i++) {
      gravity_est[i] += STILL_GRAVITY_ALPHA * (bmi088.acc[i] - gravity_est[i]);
      speed_est[i] = STILL_SPEED_LEAK * speed_est[i] +
                     (bmi088.acc[i] - gravity_est[i]) * STILL_DT_S;
      speed_sq += speed_est[i] * speed_est[i];
    }
    const float gyro_norm = std::sqrt(
      bmi088.gyro[0] * bmi088.gyro[0] + bmi088.gyro[1] * bmi088.gyro[1] +
      bmi088.gyro[2] * bmi088.gyro[2]);
    if (speed_sq < STILL_SPEED_MPS * STILL_SPEED_MPS && gyro_norm < STILL_GYRO_RADPS) {
      if (still_count < STILL_HOLD_COUNT) still_count++;
    }
    else {
      still_count = 0;
    }
    still = still_count >= STILL_HOLD_COUNT;

    // 上电后等折叠轴反馈连续稳定，再把该角度定为零位，避免首帧跳变污染零位
    if (!gimbal::fold_zero_valid && motor_fold.is_alive(now_ms)) {
      if (std::fabs(sp::limit_angle(motor_fold.angle - fold_last_angle)) < FOLD_ZERO_STABLE_RAD) {
        fold_stable_count++;
      }
      else {
        fold_stable_count = 0;
      }
      fold_last_angle = motor_fold.angle;

      if (fold_stable_count >= FOLD_ZERO_STABLE_COUNT) {
        gimbal::fold_zero_rad = motor_fold.angle;
        gimbal::fold_zero_valid = true;
      }
    }

    const bool remote_online = remote.is_alive(now_ms);
    const bool switch_down = remote.sw_r == sp::DBusSwitchMode::DOWN;
    const bool motors_ok = motor_yaw.is_alive(now_ms) && motor_pitch.is_alive(now_ms) &&
                           motor_fold.is_alive(now_ms) && gimbal::fold_zero_valid;
    const bool was_disabled = gimbal_controller.state() == gimbal::State::DISABLED;

    //调试用，左拨杆上档启用追踪
    const bool track_requested = remote_online && remote.sw_l == sp::DBusSwitchMode::UP;

    if (!remote_online || switch_down || !motors_ok) {
      // 失能：清 PID，写零力矩
      gimbal::pid_yaw.clear();
      gimbal::pid_pitch.clear();
      gimbal::pid_fold.clear();

      motor_yaw.cmd(0.0f);
      motor_pitch.cmd(0.0f);
      motor_fold.cmd(0.0f);

      gimbal_controller.disable();
    }
    else {
      // 上一周期是否失能，用于使能瞬间锁存零点
      if (was_disabled) {
        gimbal::target_angle.yaw = motor_yaw.angle;
        gimbal::target_angle.pitch = motor_pitch.angle;
        gimbal::target_angle.fold = 0.0f;
      }

      if (track_requested || (gimbal_fold_restored && still)) {
        gimbal::target_angle.yaw = yaw;
        gimbal::target_angle.pitch = pitch;

        /*调用*/
gimbal::yaw_detect(imu.yaw, gimbal::target_angle.yaw, &yaw_ratio);    //imu.yaw是云台实际的yaw,plan_yaw是对应的期望值,以弧度为单位,yaw_ratio为自己定义的变量名
gimbal::pitch_detect(imu.pitch, gimbal::target_angle.pitch, &pitch_ratio);    //imu.pitch是云台实际的pitch,plan_pitch是对应的期望值,以弧度为单位,pitch_ratio为自己定义的变量名
//调用时需要放在一个 以1ms为周期freertos任务里
      }
      // 中档 / 上档：限幅后pid输出
      gimbal::target_angle.yaw = std::fmin(
        std::fmax(gimbal::target_angle.yaw, gimbal::YAW_ANGLE_MIN_RAD), gimbal::YAW_ANGLE_MAX_RAD);
      gimbal::target_angle.pitch = std::fmin(
        std::fmax(gimbal::target_angle.pitch, gimbal::PITCH_ANGLE_MIN_RAD),
        gimbal::PITCH_ANGLE_MAX_RAD);
      gimbal::target_angle.fold = std::fmin(
        std::fmax(gimbal::target_angle.fold, gimbal::FOLD_ANGLE_MIN_RAD),
        gimbal::FOLD_ANGLE_MAX_RAD);

      // 位置环：目标角与实测角之差 → 力矩
      gimbal::pid_yaw.calc(gimbal::target_angle.yaw, motor_yaw.angle, yaw_vel, motor_yaw.speed);
      gimbal::pid_pitch.calc(
        gimbal::target_angle.pitch, motor_pitch.angle, pitch_vel, motor_pitch.speed);
      gimbal::pid_fold.calc(
        gimbal::fold_zero_rad + gimbal::target_angle.fold, motor_fold.angle, 0.0f,
        motor_fold.speed);

      motor_yaw.cmd(gimbal::pid_yaw.out);
      motor_pitch.cmd(gimbal::pid_pitch.out);
      motor_fold.cmd(gimbal::pid_fold.out);

      gimbal_controller.enable();
    }

    // 0x1FE：GM6020 的 yaw 与 fold 共用一帧；0x200：M2006 pitch
    motor_yaw.write(can2.tx_data);
    motor_fold.write(can2.tx_data);
    motor_pitch.write(can2.tx_data);

    osDelay(1);
  }
}
