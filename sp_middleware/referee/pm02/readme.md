# PM02 Demo

新建applications/uart_task.cpp
```cpp
#include "cmsis_os.h"
#include "referee/pm02/pm02.hpp"

// C板
sp::PM02 pm02(&huart6);

// 达妙
// sp::PM02 pm02(&huart1, false);

extern "C" void uart_task()
{

  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == pm02.huart) {
    pm02.update(Size);
    pm02.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == pm02.huart) {
    pm02.request();
  }
}
```

雷达自定义数据按副 ID 分别计时。收到字段所有位均为 1 时，PM02 只把该字段替换为默认值；对应副 ID 超过 1.5 s 未更新时，则把该类数据整体恢复为默认值：

- `0x0210`：飞镖预警为 0。
- `0x0213`：对方空中机器人反制状态为 0。
- `0x0211`（A01）：来源和全部坐标为 0。
- `0x0212`（A02）：全部机器人血量为 999。
- `0x0212`（A03）：全部允许发弹量为 999。
- `0x0212`（A04）：剩余/总金币数为 9999，占领状态为 0。
- `0x0212`（A05）：全部增益为 0，哨兵姿态为 3，机器人主要状态为 0。

编辑CMakeLists.txt
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/uart_task.cpp # <- 添加这一行
    sp_middleware/referee/pm02/pm02.cpp # <- 添加这一行
    sp_middleware/tools/crc/crc.cpp # <- 添加这一行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/ # <- 添加这一行
)
```
