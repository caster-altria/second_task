#include "power_meter.hpp"

namespace sp
{
bool PowerMeter::is_alive(uint32_t now_ms) const
{
  return has_read_ && (now_ms - last_read_ms_ < 50);  // 50ms内有数据更新则认为是存活的
}

void PowerMeter::read(uint8_t * data, uint32_t stamp_ms)
{
  // 数据解析
  this->current = ((int16_t)(((data)[0] << 8) | (data)[1])) / 100.0f;
  this->voltage = ((int16_t)(((data)[2] << 8) | (data)[3])) / 100.0f;
  this->power = ((int16_t)(((data)[4] << 8) | (data)[5])) / 100.0f;

  // 更新时间戳
  last_read_ms_ = stamp_ms;
  has_read_ = true;
}

}  // namespace sp
