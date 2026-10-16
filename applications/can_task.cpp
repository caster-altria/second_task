#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "power_meter.hpp"

sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);
sp::RM_Motor wheel_fl(1, sp::RM_Motors::M3508, 14.9f);
sp::RM_Motor wheel_fr(2, sp::RM_Motors::M3508, 14.9f);
sp::RM_Motor wheel_rl(3, sp::RM_Motors::M3508, 14.9f);
sp::RM_Motor wheel_rr(4, sp::RM_Motors::M3508, 14.9f);
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
    const uint32_t now_ms = osKernelSysTick();
    if (
      !wheel_fl.is_alive(now_ms) || !wheel_fr.is_alive(now_ms) || !wheel_rl.is_alive(now_ms) ||
      !wheel_rr.is_alive(now_ms) || !gimbal_yaw.is_alive(now_ms) ||
      !gimbal_pitch.is_alive(now_ms) || !gimbal_fold.is_alive(now_ms)) {
      wheel_fl.cmd(0.0f);
      wheel_fr.cmd(0.0f);
      wheel_rl.cmd(0.0f);
      wheel_rr.cmd(0.0f);
      gimbal_yaw.cmd(0.0f);
      gimbal_pitch.cmd(0.0f);
      gimbal_fold.cmd(0.0f);
    }

    wheel_fl.write(can1.tx_data);
    wheel_fr.write(can1.tx_data);
    wheel_rl.write(can1.tx_data);
    wheel_rr.write(can1.tx_data);
    can1.send(wheel_fr.tx_id);

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

      if (can1.rx_id == wheel_fl.rx_id) wheel_fl.read(can1.rx_data, stamp_ms);
      if (can1.rx_id == wheel_fr.rx_id) wheel_fr.read(can1.rx_data, stamp_ms);
      if (can1.rx_id == wheel_rl.rx_id) wheel_rl.read(can1.rx_data, stamp_ms);
      if (can1.rx_id == wheel_rr.rx_id) wheel_rr.read(can1.rx_data, stamp_ms);
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