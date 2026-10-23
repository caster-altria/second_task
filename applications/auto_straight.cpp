#include "auto_straight.hpp"

namespace
{
constexpr float AUTO_SPEED = 0.30f;      // 自动直行速度，单位 m/s
constexpr float AUTO_DISTANCE_M = 0.3f;  // 启动区到结束区行程，单位 m
constexpr float PERIOD_S = 0.001f;       // 任务周期，单位 s
}  // namespace

namespace chassis
{
void AutoStraight::start()
{
  this->active_ = true;
  this->finished_ = false;
  this->travelled_m_ = 0.0f;
}

float AutoStraight::update(float measured_vx)
{
  if (!this->active_ || this->finished_) return 0.0f;

  this->travelled_m_ += measured_vx * PERIOD_S;

  // 到点停车，直到下一次 start()
  if (this->travelled_m_ >= AUTO_DISTANCE_M) {
    this->finished_ = true;
    return 0.0f;
  }

  return AUTO_SPEED;
}

void AutoStraight::stop()
{
  this->active_ = false;
  this->finished_ = false;
  this->travelled_m_ = 0.0f;
}

float AutoStraight::travelled_m() const { return this->travelled_m_; }

bool AutoStraight::active() const { return this->active_; }

bool AutoStraight::finished() const { return this->finished_; }
}  // namespace chassis