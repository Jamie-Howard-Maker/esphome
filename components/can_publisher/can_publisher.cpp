#include "can_publisher.h"
#include "esphome/core/log.h"

namespace esphome {
namespace can_publisher {

static const char *const TAG = "can_publisher";

void CANPublisher::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CAN Publisher...");
  ESP_LOGCONFIG(TAG, "  Number of frames: %d", this->frames_.size());
  ESP_LOGCONFIG(TAG, "  Log frames: %s", YESNO(this->log_frames_));
}

void CANPublisher::update() {
  for (const auto &frame : this->frames_) {
    this->send_frame_(frame);
  }
}

int16_t CANPublisher::get_value_(const ValueSource &src) {
  float value = 0.0f;

  if (src.sensor != nullptr) {
    if (src.sensor->has_state()) {
      value = src.sensor->state;
    }
  } else if (src.binary_sensor != nullptr) {
    value = src.binary_sensor->state ? 1.0f : 0.0f;
  }

  value = (value + src.offset) * src.scale;
  return static_cast<int16_t>(value);
}

void CANPublisher::send_frame_(const FrameConfig &frame) {
  std::vector<uint8_t> data(8, 0);

  for (size_t i = 0; i < frame.values.size() && i < 4; i++) {
    int16_t val = this->get_value_(frame.values[i]);
    data[i * 2]     = (val >> 8) & 0xFF;  // high byte
    data[i * 2 + 1] = val & 0xFF;         // low byte
  }

  auto err = this->canbus_->send_data(frame.can_id, false, data);  // standard 11-bit ID

  if (this->log_frames_) {
    if (err == canbus::ERROR_OK) {
      ESP_LOGI(TAG, "TX 0x%03X: %02X %02X %02X %02X %02X %02X %02X %02X",
               frame.can_id,
               data[0], data[1], data[2], data[3],
               data[4], data[5], data[6], data[7]);
    } else {
      ESP_LOGW(TAG, "TX 0x%03X failed (err=%d)", frame.can_id, err);
    }
  }
}

}  // namespace can_publisher
}  // namespace esphome
