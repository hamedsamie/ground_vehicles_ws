#pragma once
/**
 * @brief Simple integrator controller with gain and saturation.
 *
 * Maintains an internal state y. Each step applies y ← clamp(y + gain * u),
 * only when in RUNNING mode. Provides deterministic, testable behavior and
 * supports dynamic parameter updates.
 */

#include <sstream>
#include <stdexcept>
#include <string>

namespace ground_architect
{
namespace algo
{

enum class Mode
{
  IDLE,
  READY,
  RUNNING
};

inline const char *to_string(Mode m)
{
  switch (m)
  {
  case Mode::IDLE:
    return "IDLE";
  case Mode::READY:
    return "READY";
  case Mode::RUNNING:
    return "RUNNING";
  }
  return "UNKNOWN";
}

struct Config
{
  double gain{1.0};        // multiplier for input u
  double saturation{10.0}; // clamp: |y| <= saturation
};

struct Input
{
  double u{0.0}; // input command
};

struct Output
{
  double y{0.0};    // current output
  std::string note; // "ok" / "inactive" or error context
};

class Controller final
{
public:
  Controller() = default;

  void init(const Config &cfg)
  {
    cfg_ = cfg;
    y_ = 0.0;
    initialized_ = true;
    last_error_.clear();
  }

  void reset()
  {
    y_ = 0.0;
    last_error_.clear();
  }

  void set_mode(Mode m)
  {
    mode_ = m;
  }
  Mode mode() const
  {
    return mode_;
  }

  const Config &config() const
  {
    return cfg_;
  }
  void update_config(const Config &cfg)
  {
    cfg_ = cfg;
  }

  /// Single control step.
  Output step(const Input &in)
  {
    if (!initialized_)
      throw std::runtime_error("Controller not initialized");
    if (mode_ != Mode::RUNNING)
    {
      return Output{y_, "inactive"};
    }

    // Update
    y_ += cfg_.gain * in.u;

    // Clamp
    if (y_ > cfg_.saturation)
      y_ = cfg_.saturation;
    if (y_ < -cfg_.saturation)
      y_ = -cfg_.saturation;

    return Output{y_, "ok"};
  }

  std::string status() const
  {
    std::ostringstream oss;
    oss << "mode=" << to_string(mode_) << " y=" << y_ << " gain=" << cfg_.gain
        << " sat=" << cfg_.saturation;
    return oss.str();
  }

  double y() const
  {
    return y_;
  }

private:
  Config cfg_{};
  Mode mode_{Mode::IDLE};
  double y_{0.0};
  bool initialized_{false};
  std::string last_error_;
};

} // namespace algo
} // namespace ground_architect
