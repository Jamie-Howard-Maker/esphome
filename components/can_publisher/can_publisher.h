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
  uint32_t can_id{0};
  std::vector<ValueSource> values;
};

class CANPublisher : public PollingComponent {
 public:
  void set_canbus(canbus::Canbus *canbus) { this->canbus_ = canbus; }
  void set_log_frames(bool log) { this->log_frames_ = log; }

  // Called from Python
  void start_frame(uint32_t can_id);
  void add_sensor_value(sensor::Sensor *sens, float scale, float offset);
  void add_binary_sensor_value(binary_sensor::BinarySensor *bsens, float scale, float offset);
  void end_frame();

  void setup() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  canbus::Canbus *canbus_{nullptr};
  bool log_frames_{false};
  std::vector<FrameConfig> frames_;
  FrameConfig current_frame_;   // temporary while building

  void send_frame_(const FrameConfig &frame);
  int16_t get_value_(const ValueSource &src);
};

}  // namespace can_publisher
}  // namespace esphome
