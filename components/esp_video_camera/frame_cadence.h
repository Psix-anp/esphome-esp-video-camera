#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace esphome::esp_video_camera {

// Capture-task-owned cadence. Keep fractional phase without accumulating a
// burst of expensive work after a pause. This object owns no frame storage.
class FrameCadence {
 public:
  void reset() {
    this->seen_ = false;
    this->credit_ = 0;
    this->fps_ = 0.0f;
  }

  bool accept(uint32_t timestamp_us, float max_fps) {
    if (!(max_fps > 0.0f) || !std::isfinite(max_fps)) {
      this->reset();
      return true;
    }
    if (max_fps != this->fps_) {
      // A configuration change starts a fresh cadence epoch.
      this->reset();
      this->fps_ = max_fps;
      constexpr uint64_t maximum = std::numeric_limits<uint64_t>::max() / 2;
      const double interval = (1000000.0 * SCALE) / max_fps;
      this->interval_ = interval >= static_cast<double>(maximum)
                            ? maximum
                            : std::max<uint64_t>(1, static_cast<uint64_t>(interval + 0.5));
    }
    if (!this->seen_) {
      this->seen_ = true;
      this->last_us_ = timestamp_us;
      return true;
    }
    const uint64_t elapsed = static_cast<uint32_t>(timestamp_us - this->last_us_) * SCALE;
    this->last_us_ = timestamp_us;
    uint64_t credit = elapsed > std::numeric_limits<uint64_t>::max() - this->credit_
                          ? std::numeric_limits<uint64_t>::max()
                          : this->credit_ + elapsed;
    if (credit >= this->interval_ && credit - this->interval_ >= this->interval_)
      credit = this->interval_;
    if (credit < this->interval_) {
      this->credit_ = credit;
      return false;
    }
    this->credit_ = credit - this->interval_;
    return true;
  }

 protected:
  static constexpr uint64_t SCALE = 65536;
  uint64_t interval_{0};
  uint64_t credit_{0};
  uint32_t last_us_{0};
  float fps_{0.0f};
  bool seen_{false};
};

}  // namespace esphome::esp_video_camera
