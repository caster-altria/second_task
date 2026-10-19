#include "cmsis_os.h"
#include "chassis_controller.hpp"
#include "io/plotter/plotter.hpp"
#include "power_meter.hpp"

// 当前 Plotter 使用 USART1。使用时必须暂时停用激光任务，避免和 LaserGY56 共用 huart1。
sp::Plotter plotter(&huart1);

extern chassis::Controller chassis_controller;
extern sp::PowerMeter power_meter;

namespace
{
// 每次只选择一个模式，SerialPlot 的通道数量就等于对应 plotter.plot() 的参数数量。
enum class PlotMode
{
  LINK_TEST,       // 1 个固定值，先验证串口链路
  COMMAND,         // vx、vy、wz
  TARGET_WHEELS,   // 四个目标轮速
  LF_PID,          // 左前轮目标速度、实际速度、PID 力矩
  POWER,           // K、预测功率、实际功率
  ALL,             // 完整 18 通道
};

// 调试时只修改这一行，不要同时打开多个 plotter.plot()。
constexpr PlotMode kPlotMode = PlotMode::LF_PID;
constexpr uint32_t kPlotPeriodMs = 10U;

void plot_selected_data()
{
  const auto & data = chassis_controller.plot_data();
  const uint32_t now_ms = HAL_GetTick();
  const float actual_power = power_meter.is_alive(now_ms) ? power_meter.power : 0.0f;

  switch (kPlotMode) {
    case PlotMode::LINK_TEST:
      plotter.plot(1.0f);
      break;

    case PlotMode::COMMAND:
      plotter.plot(data.command_vx, data.command_vy, data.command_wz);
      break;

    case PlotMode::TARGET_WHEELS:
      plotter.plot(data.target_lf, data.target_lr, data.target_rf, data.target_rr);
      break;

    case PlotMode::LF_PID:
      plotter.plot(data.target_lf, data.speed_lf, data.torque_lf);
      break;

    case PlotMode::POWER:
      plotter.plot(data.power_scale, data.predicted_power, actual_power);
      break;

    case PlotMode::ALL:
      plotter.plot(
        data.command_vx, data.command_vy, data.command_wz,
        data.target_lf, data.target_lr, data.target_rf, data.target_rr,
        data.speed_lf, data.speed_lr, data.speed_rf, data.speed_rr,
        data.torque_lf, data.torque_lr, data.torque_rf, data.torque_rr,
        data.power_scale, data.predicted_power, actual_power);
      break;
  }
}
}  // namespace

extern "C" void plotter_task(void const * argument)
{
  (void)argument;

  while (true) {
    // Plotter 使用 DMA；上一次发送未完成时跳过本次，避免 HAL_BUSY。
    if (huart1.gState == HAL_UART_STATE_READY) {
      plot_selected_data();
    }
    osDelay(kPlotPeriodMs);
  }
}
