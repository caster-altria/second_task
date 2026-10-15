#include "cmsis_os.h"
#include "io/laser/laser.hpp"

sp::LaserGY56 laser(&huart1);
uint16_t distance = 0xFFFF;

extern "C" void laser_task()
{
  laser.start_receive();
  laser.set_baud_rate_115200();
  laser.save_config();

  while (true) {
    // 对外只发布未超时的有效距离, 否则发布无效值
    distance = laser.is_valid() ? laser.get_distance() : 0xFFFF;
    osDelay(1);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart1) {
    laser.receive_complete(Size);
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart1) {
    laser.start_receive();
  }
}