#include <cmath>

#include "cmsis_os.h"
#include "gimbal_controller.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

sp::RM_Motor motor_fold(3, sp::RM_Motors::M2006, 36.0f);
sp::RM_Motor motor_yaw(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_pitch(2, sp::RM_Motors::GM6020);

gimbal::GimbalController gimbal_controller;

extern sp::DBus remote;
extern sp::CAN can2;

extern "C" void gimbal_task(void const * argument)
{
  (void)argument;

  while (true) {
    const uint32_t now_ms = osKernelSysTick();

    const bool remote_online = remote.is_alive(now_ms);
    const bool switch_down = remote.sw_r == sp::DBusSwitchMode::DOWN;
    const bool motors_ok =
      motor_yaw.is_alive(now_ms) && motor_pitch.is_alive(now_ms) && motor_fold.is_alive(now_ms);
    const bool was_disabled = gimbal_controller.state() == gimbal::State::DISABLED;

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
        gimbal::target_angle.fold = motor_fold.angle;
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
      gimbal::pid_yaw.calc(gimbal::target_angle.yaw, motor_yaw.angle);
      gimbal::pid_pitch.calc(gimbal::target_angle.pitch, motor_pitch.angle);
      gimbal::pid_fold.calc(gimbal::target_angle.fold, motor_fold.angle);

      motor_yaw.cmd(gimbal::pid_yaw.out);
      motor_pitch.cmd(gimbal::pid_pitch.out);
      motor_fold.cmd(gimbal::pid_fold.out);

      gimbal_controller.enable();
    }

    // 0x1FF：yaw 与 pitch 共用一帧
    motor_yaw.write(can2.tx_data);
    motor_pitch.write(can2.tx_data);

    // 0x200：M2006 折叠电机
    motor_fold.write(can2.tx_data);

    osDelay(1);
  }
}