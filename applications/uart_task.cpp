#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "io/laser/laser.hpp"

// C板
sp::DBus remote(&huart3);

// 达妙
// sp::DBus remote(&huart5, false);
// sp::PM02 pm02(&huart1, false);

extern sp::LaserGY56 laser;

extern "C" void uart_task()
{
  remote.request();

  while (true) osDelay(10);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }

  if (huart == &huart6) {
    laser.receive_complete(Size);
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }

  if (huart == &huart6) {
    laser.start_receive();
  }
}