#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (1) {
    //    plotter.plot( 1.0f);
    osDelay(1);
  }
}