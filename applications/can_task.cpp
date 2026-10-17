#include "chassis_controller.hpp"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "power_meter.hpp"

extern chassis::Controller chassis_controller;

sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);
sp::RM_Motor gimbal_fold(1, sp::RM_Motors::M2006, 36.0f);
sp::RM_Motor gimbal_yaw(1, sp::RM_Motors::GM6020);
sp::RM_Motor gimbal_pitch(2, sp::RM_Motors::GM6020);
//底盘和云台各占用一条can线

sp::PowerMeter power_meter;

extern "C" void can_task()
{
  can1.config();
  can1.start();
  can2.config();
  can2.start();

  while (true) {
    can1.send(chassis_controller.motor_tx_id());

    gimbal_yaw.write(can2.tx_data);
    gimbal_pitch.write(can2.tx_data);
    can2.send(gimbal_yaw.tx_id);

    gimbal_fold.write(can2.tx_data);
    can2.send(gimbal_fold.tx_id);

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

      if (can2.rx_id == gimbal_yaw.rx_id) gimbal_yaw.read(can2.rx_data, stamp_ms);
      if (can2.rx_id == gimbal_pitch.rx_id) gimbal_pitch.read(can2.rx_data, stamp_ms);
      if (can2.rx_id == gimbal_fold.rx_id) gimbal_fold.read(can2.rx_data, stamp_ms);
    }
  }
}