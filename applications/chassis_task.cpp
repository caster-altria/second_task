#include "auto_straight.hpp"
#include "chassis_controller.hpp"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"

extern sp::DBus remote;
extern sp::CAN can1;
chassis::Controller chassis_controller;  // 全局，供 can_task.cpp 的中断调用
chassis::AutoStraight auto_straight;

extern "C" void chassis_task(void const * argument)
{
  (void)argument;

  for (;;) {
    const uint32_t now_ms = osKernelSysTick();
    const bool remote_online = remote.is_alive(now_ms);

    chassis::RemoteInput input;
    input.online = remote_online;
    if (remote_online) {
      input.sw_r = remote.sw_r;
      input.ch_lv = remote.ch_lv;
      input.ch_lh = remote.ch_lh;
      input.ch_lu = remote.ch_lu;
    }

    if (!remote_online || remote.sw_r == sp::DBusSwitchMode::DOWN) {
      auto_straight.stop();
    }
    else if (remote.sw_r == sp::DBusSwitchMode::UP) {
      // 自动：右拨杆上档，直线行驶至设定里程
      if (!auto_straight.active()) {
        auto_straight.start();
      }
      const float measured_vx = chassis_controller.measured_vx();
      const float vx_cmd = auto_straight.update(measured_vx);
      chassis_controller.set_auto_speed(vx_cmd);
    }
    else {
      // 中档：手动。退出自动时回到失能，要求摇杆回中解锁
      if (auto_straight.active()) {
        auto_straight.stop();
      }
      chassis_controller.update(input, now_ms);
    }

    chassis_controller.update_motor_control(now_ms);
    chassis_controller.write_motor_commands(can1.tx_data);

    osDelay(1);
  }
}