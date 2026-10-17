#include "chassis_controller.hpp"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"

extern sp::DBus remote;
extern sp::CAN can1;
chassis::Controller chassis_controller;  // 全局，供 can_task.cpp 的中断调用

extern "C" void chassis_task(void const * argument)
{
  (void)argument;

  for (;;) {
    const uint32_t now_ms = osKernelSysTick();

    // 先判断DBUS是否在线，再读取遥控器的公开数据。
    chassis::RemoteInput input;
    input.online = remote.is_alive(now_ms);

    if (input.online) {
      input.sw_r = remote.sw_r;
      input.ch_lv = remote.ch_lv;
      input.ch_lh = remote.ch_lh;
      input.ch_lu = remote.ch_lu;
    }

    // 控制器根据输入更新状态机和vx、vy、wz指令。
    chassis_controller.update(input, now_ms);

    // 根据目标轮速和反馈轮速执行四个 sp::PID，并在失能或掉线时输出零力矩。
    chassis_controller.update_motor_control(now_ms);

    // 将四个 RM_Motor 的命令填入同一个 C620 控制帧，再通过 CAN1 发送。
    chassis_controller.write_motor_commands(can1.tx_data);

    // 当前控制周期约为 1 ms；PID 构造时的 dt 也按 0.001 s 配置。
    osDelay(1);
  }
}
