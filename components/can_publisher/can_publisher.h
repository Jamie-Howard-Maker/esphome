#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/canbus/canbus.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <vector>

namespace esphome {
namespace can_publisher {

struct ValueSource {
  sensor::Sensor *sensor{nullptr};
  binary_sensor::BinarySensor *binary_sensor{nullptr};
  float scale{1.0f};
  float offset{0.0f};
};

struct FrameConfig {
  uint32_t can_id;
  std::vector<ValueSource> values;  // max 4
};

class CANPublisher : public PollingComponent {
 public:
  void set_canbus(canbus::Canbus *canbus) { this->canbus_ = canbus; }
  void set_log_frames(bool log) { this->log_frames_ = log; }

  void add_frame(uint32_t can_id, const std::vector<std::tuple<sensor::Sensor*, binary_sensor::BinarySensor*, float, float>> &vals) {
    FrameConfig frame;
    frame.can_id = can_id;
    for (auto &v : vals) {
      ValueSource src;
      src.sensor = std::get<0>(v);
      src.binary_sensor = std::get<1>(v);
      src.scale = std::get<2>(v);
      src.offset = std::get<3>(v);
      frame.values.push_back(src);
    }
    this->frames_.push_back(frame);
  }

  void setup() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  canbus::Canbus *canbus_{nullptr};
  bool log_frames_{false};
  std::vector<FrameConfig> frames_;

  void send_frame_(const FrameConfig &frame);
  int16_t get_value_(const ValueSource &src);
};

}  // namespace can_publisher
}  // namespace esphome
