#pragma once

#include <chrono>
#include <cstdint>
#include <ratio>

namespace controller {

class Duration
{
 public:
  Duration() noexcept : duration_{0u} {}

  explicit Duration(uint64_t milliseconds) noexcept : duration_{milliseconds} {}

  Duration(std::chrono::duration<uint64_t, std::milli> duration) noexcept : duration_{duration} {}

  Duration(const Duration&) = default;

  Duration& operator=(const Duration&) = default;

  operator std::chrono::duration<uint64_t, std::milli>() const noexcept { return duration_; }

  double toSec() const noexcept
  {
    return std::chrono::duration_cast<std::chrono::duration<double>>(duration_).count();
  }

  uint64_t toMSec() const noexcept { return duration_.count(); }

  Duration operator+(const Duration& rhs) const noexcept { return duration_ + rhs.duration_; }

  Duration operator-(const Duration& rhs) const noexcept { return duration_ - rhs.duration_; }

 private:
  std::chrono::duration<uint64_t, std::milli> duration_;
};


}  // namespace controller
