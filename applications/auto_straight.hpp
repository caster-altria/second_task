#ifndef AUTO_STRAIGHT_HPP
#define AUTO_STRAIGHT_HPP

#include <cstdint>

namespace chassis
{
//自动直行：右拨杆上档后按固定速度前进，跑满设定里程后停车。
class AutoStraight
{
public:
  // 开始一次直行，复位里程
  void start();

  // 每 1 ms 调用一次，返回本周期需要下发的前进速度，单位 m/s
  float update(float measured_vx);

  // 中止并复位
  void stop();

  // [只读] 已行驶距离，单位 m
  float travelled_m() const;

  // [只读] 是否处于直行过程中
  bool active() const;

private:
  bool active_ = false;
  bool finished_ = false;
  float travelled_m_ = 0.0f;
};
}  // namespace chassis

#endif  // AUTO_STRAIGHT_HPP