#include "chassis_controller.hpp"
#include "cmsis_os.h"
#include "gimbal_controller.hpp"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "power_meter.hpp"

extern chassis::Controller chassis_controller;

sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);
//底盘和云台各占用一条can线

sp::PowerMeter power_meter;

extern gimbal::GimbalController gimbal_controller;
extern sp::RM_Motor motor_yaw;
extern sp::RM_Motor motor_pitch;
extern sp::RM_Motor motor_fold;

extern "C" void can_task()
{
  can1.config();
  can1.start();
  can2.config();
  can2.start();

  while (true) {
    can2.send(motor_yaw.tx_id);

    can2.send(motor_pitch.tx_id);

    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      chassis_controller.update_motor_feedback(can1.rx_id, can1.rx_data, stamp_ms);
      if (can1.rx_id == power_meter.power_meter_rx_id) power_meter.read(can1.rx_data, stamp_ms);
    }
    else if (hcan == &hcan2) {
      can2.recv();

      if (can2.rx_id == motor_yaw.rx_id) motor_yaw.read(can2.rx_data, stamp_ms);
      if (can2.rx_id == motor_pitch.rx_id) motor_pitch.read(can2.rx_data, stamp_ms);
      if (can2.rx_id == motor_fold.rx_id) motor_fold.read(can2.rx_data, stamp_ms);
    }
  }
}