#include "rd03d.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome::rd03d {

static const char *const TAG = "rd03d";

// Delay before sending configuration commands to allow radar to initialize
static constexpr uint32_t SETUP_TIMEOUT_MS = 100;

// Data frame format (radar -> host)
static constexpr uint8_t FRAME_HEADER[] = {0xAA, 0xFF, 0x03, 0x00};
static constexpr uint8_t FRAME_FOOTER[] = {0x55, 0xCC};

// Command frame format (host -> radar)
static constexpr uint8_t CMD_FRAME_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
static constexpr uint8_t CMD_FRAME_FOOTER[] = {0x04, 0x03, 0x02, 0x01};

// RD-03D tracking mode commands
static constexpr uint16_t CMD_SINGLE_TARGET = 0x0080;
static constexpr uint16_t CMD_MULTI_TARGET = 0x0090;

// Speed sentinel values (cm/s) - radar outputs these when no valid Doppler measurement
static constexpr int16_t SPEED_SENTINEL_248 = 248;
static constexpr int16_t SPEED_SENTINEL_256 = 256;

// Decode coordinate/speed value from RD-03D format
// Per datasheet: MSB=1 means positive, MSB=0 means negative
static constexpr int16_t decode_value(uint8_t low_byte, uint8_t high_byte) {
  int16_t value = ((high_byte & 0x7F) << 8) | low_byte;
  if ((high_byte & 0x80) == 0) {
    value = -value;
  }
  return value;
}

// Check if speed value indicates a valid Doppler measurement
static constexpr bool is_speed_valid(int16_t speed) {
  int16_t abs_speed = speed < 0 ? -speed : speed;
  return speed != 0 && abs_speed != SPEED_SENTINEL_248 && abs_speed != SPEED_SENTINEL_256;
}

void RD03DComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up RD-03D...");
  this->set_timeout(SETUP_TIMEOUT_MS, [this]() { this->apply_config_(); });
}

void RD03DComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "RD-03D:");
  if (this->tracking_mode_.has_value()) {
    ESP_LOGCONFIG(TAG, "  Tracking Mode: %s",
                  *this->tracking_mode_ == TrackingMode::SINGLE_TARGET ? "single" : "multi");
  }
  if (this->throttle_ > 0) {
    ESP_LOGCONFIG(TAG, "  Throttle: %ums", this->throttle_);
  }
#ifdef USE_SENSOR
  LOG_SENSOR("  ", "Target Count", this->target_count_sensor_);
#endif
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "Target", this->target_binary_sensor_);
#endif
  for (uint8_t i = 0; i < MAX_TARGETS; i++) {
    ESP_LOGCONFIG(TAG, "  Target %d:", i + 1);
#ifdef USE_SENSOR
    LOG_SENSOR("    ", "X", this->targets_[i].x);
    LOG_SENSOR("    ", "Y", this->targets_[i].y);
    LOG_SENSOR("    ", "Speed", this->targets_[i].speed);
    LOG_SENSOR("    ", "Distance", this->targets_[i].distance);
    LOG_SENSOR("    ", "Resolution", this->targets_[i].resolution);
    LOG_SENSOR("    ", "Angle", this->targets_[i].angle);
#endif
#ifdef USE_BINARY_SENSOR
    LOG_BINARY_SENSOR("    ", "Presence", this->target_presence_[i]);
#endif
  }
}

// =====================
// Robust frame parsing
// =====================
void RD03DComponent::loop() {
  while (this->available()) {
    uint8_t byte = this->read();

    // Shift buffer and append new byte
    for (uint8_t i = 0; i < FRAME_BUFFER_SIZE - 1; i++) {
      this->buffer_[i] = this->buffer_[i + 1];
    }
    this->buffer_[FRAME_BUFFER_SIZE - 1] = byte;

    // Try to find header
    for (uint8_t i = 0; i <= FRAME_BUFFER_SIZE - FRAME_SIZE; i++) {
      if (memcmp(&this->buffer_[i], FRAME_HEADER, sizeof(FRAME_HEADER)) == 0 &&
          this->buffer_[i + FRAME_SIZE - 2] == FRAME_FOOTER[0] &&
          this->buffer_[i + FRAME_SIZE - 1] == FRAME_FOOTER[1]) {
        // Found valid frame, process it
        memcpy(this->frame_tmp_, &this->buffer_[i], FRAME_SIZE);
        this->process_frame_();
        // Shift buffer after processed frame
        uint8_t shift = i + FRAME_SIZE;
        for (uint8_t j = 0; j < FRAME_BUFFER_SIZE - shift; j++) {
          this->buffer_[j] = this->buffer_[j + shift];
        }
        break;
      }
    }
  }
}

// =====================
// Process single frame
// =====================
void RD03DComponent::process_frame_() {
  if (this->throttle_ > 0) {
    uint32_t now = millis();
    if (now - this->last_publish_time_ < this->throttle_) {
      return;
    }
    this->last_publish_time_ = now;
  }

  uint8_t target_count = 0;

  for (uint8_t i = 0; i < MAX_TARGETS; i++) {
    uint8_t offset = FRAME_HEADER_SIZE + (i * TARGET_DATA_SIZE);

    uint8_t x_low = this->frame_tmp_[offset + 0];
    uint8_t x_high = this->frame_tmp_[offset + 1];
    uint8_t y_low = this->frame_tmp_[offset + 2];
    uint8_t y_high = this->frame_tmp_[offset + 3];
    uint8_t speed_low = this->frame_tmp_[offset + 4];
    uint8_t speed_high = this->frame_tmp_[offset + 5];
    uint8_t res_low = this->frame_tmp_[offset + 6];
    uint8_t res_high = this->frame_tmp_[offset + 7];

    int16_t x = decode_value(x_low, x_high);
    int16_t y = decode_value(y_low, y_high);
    int16_t speed = decode_value(speed_low, speed_high);
    uint16_t resolution = (res_high << 8) | res_low;

    bool target_present = (x != 0 || y != 0);
    if (target_present) {
      target_count++;
    }

#ifdef USE_SENSOR
    this->publish_target_(i, x, y, speed, resolution);
#endif

#ifdef USE_BINARY_SENSOR
    if (this->target_presence_[i] != nullptr) {
      this->target_presence_[i]->publish_state(target_present);
    }
#endif
  }

#ifdef USE_SENSOR
  if (this->target_count_sensor_ != nullptr) {
    this->target_count_sensor_->publish_state(target_count);
  }
#endif

#ifdef USE_BINARY_SENSOR
  if (this->target_binary_sensor_ != nullptr) {
    this->target_binary_sensor_->publish_state(target_count > 0);
  }
#endif
}

// =====================
// Target publishing
// =====================
#ifdef USE_SENSOR
void RD03DComponent::publish_target_(uint8_t target_num, int16_t x, int16_t y, int16_t speed, uint16_t resolution) {
  TargetSensor &target = this->targets_[target_num];

  bool valid = (x != 0 || y != 0); // Any non-zero coordinates count as valid

  if (target.x != nullptr) {
    target.x->publish_state(valid ? static_cast<float>(x) : NAN);
  }
  if (target.y != nullptr) {
    target.y->publish_state(valid ? static_cast<float>(y) : NAN);
  }
  if (target.speed != nullptr) {
    target.speed->publish_state(valid ? static_cast<float>(speed) * 10.0f : NAN);
  }
  if (target.resolution != nullptr) {
    target.resolution->publish_state(resolution);
  }
  if (target.distance != nullptr) {
    if (valid) {
      target.distance->publish_state(std::hypot(static_cast<float>(x), static_cast<float>(y)));
    } else {
      target.distance->publish_state(NAN);
    }
  }
  if (target.angle != nullptr) {
    if (valid) {
      float angle = std::atan2(static_cast<float>(x), static_cast<float>(y)) * 180.0f / M_PI;
      target.angle->publish_state(angle);
    } else {
      target.angle->publish_state(NAN);
    }
  }
}
#endif

// =====================
// Commands
// =====================
void RD03DComponent::send_command_(uint16_t command, const uint8_t *data, uint8_t data_len) {
  this->write_array(CMD_FRAME_HEADER, sizeof(CMD_FRAME_HEADER));

  uint16_t len = 2 + data_len;
  this->write_byte(len & 0xFF);
  this->write_byte((len >> 8) & 0xFF);

  this->write_byte(command & 0xFF);
  this->write_byte((command >> 8) & 0xFF);

  if (data != nullptr && data_len > 0) {
    this->write_array(data, data_len);
  }

  this->write_array(CMD_FRAME_FOOTER, sizeof(CMD_FRAME_FOOTER));
  ESP_LOGD(TAG, "Sent command 0x%04X with %d bytes of data", command, data_len);
}

void RD03DComponent::apply_config_() {
  if (this->tracking_mode_.has_value()) {
    uint16_t mode_cmd = (*this->tracking_mode_ == TrackingMode::SINGLE_TARGET) ? CMD_SINGLE_TARGET : CMD_MULTI_TARGET;
    this->send_command_(mode_cmd);
  }
}

}  // namespace esphome::rd03d
