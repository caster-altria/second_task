#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "chassis_controller.hpp"

namespace
{
// DBUS 中间件负责解析遥控器的18字节数据帧。
sp::DBus remote(&huart3);

// CAN 中间件对象绑定 CubeMX 生成的 CAN1 外设句柄。
// 当前四个 M3508 暂定全部接在 CAN1 上。
sp::CAN can1(&hcan1);

// 底盘控制器负责遥控器状态机和速度指令生成。
chassis::Controller chassis_controller;
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == &huart3) {
    // UART收到一帧数据后，交给DBUS中间件解析，并立即准备接收下一帧。
    auto stamp_ms = osKernelSysTick();
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    // 接收出错时重新启动DBUS接收，等待下一帧有效数据。
    remote.request();
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  // 当前底盘只使用 CAN1；其他 CAN 外设的接收帧不交给底盘控制器。
  if (hcan != &hcan1) {
    return;
  }

  const uint32_t stamp_ms = osKernelSysTick();

  // 一个中断可能已经积累多帧 CAN 数据，因此要一直读到 FIFO 为空。
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
    // CAN 中间件负责从 HAL 读取帧头和 8 字节数据，并更新 rx_id/rx_data。
    can1.recv(CAN_RX_FIFO0);

    // 控制器再根据反馈 ID 分发给对应的 RM_Motor 中间件对象。
    chassis_controller.update_motor_feedback(can1.rx_id, can1.rx_data, stamp_ms);
  }
}

extern "C" void chassis_task(void const * argument)
{
  (void)argument;

  // 启动DBUS的DMA接收。
  remote.request();

  // 按照 CAN 中间件说明，先配置过滤器，再启动 CAN 和接收中断。
  can1.config();
  can1.start();

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
    can1.send(chassis_controller.motor_tx_id());

    // 当前控制周期约为 1 ms；PID 构造时的 dt 也按 0.001 s 配置。
    osDelay(1);
  }
}
