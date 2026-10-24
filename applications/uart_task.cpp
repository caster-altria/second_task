#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "io/laser/laser.hpp"
#include "referee/pm02/pm02.hpp"

// C板
sp::DBus remote(&huart3);
sp::PM02 pm02(&huart6);

// 达妙
// sp::DBus remote(&huart5, false);
// sp::PM02 pm02(&huart1, false);

extern sp::LaserGY56 laser;

extern "C" void uart_task()
{
  remote.request();

  while (true) {
    // 使用调试(f5)查看remote内部变量的变化
    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }

  if (huart == pm02.huart) {
    pm02.update(Size);
    pm02.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }

  if (huart == pm02.huart) {
    pm02.request();
  }
}