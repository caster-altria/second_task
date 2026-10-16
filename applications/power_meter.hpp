#ifndef SP__POWER_METER_HPP
#define SP__POWER_METER_HPP

#include <cstdint>

namespace sp
{

class PowerMeter
{
public:
  const uint16_t power_meter_rx_id = 0x301;  // 功率计反馈帧ID

  float current;  // 只读，单位:A
  float voltage;  // 只读，单位:V
  float power;    // 只读，单位:W

  bool is_alive(uint32_t now_ms) const;

  void read(uint8_t * data, uint32_t stamp_ms);

private:
  uint32_t last_read_ms_ = 0;  // 上一次读取数据的时间戳
  bool has_read_ = false;
};

}  // namespace sp

#endif  // IO_POWER_METER_HPP