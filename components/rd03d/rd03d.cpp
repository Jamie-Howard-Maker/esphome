#include "rd03d.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome::rd03d {

static const char *const TAG = "rd03d";

// Radar frame format
// AA FF 03 LL [payload LL bytes] 55 CC
static constexpr uint8_t FRAME_SYNC[] = {0xAA, 0xFF, 0x03};
static constexpr size_t FRAME_SYNC_SIZE = 3;
static constexpr uint8_t FRAME_FOOTER[] = {0x55, 0xCC};

// Command frame format (host -> radar)
static constexpr uint8_t CMD_FRAME_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
static constexpr uint8_t CMD_FRAME_FOOTER[] = {0x04, 0x03, 0x02, 0x01};

// RD-03D tracking mode commands
static constexpr uint16_t CMD_SINGLE_TARGET = 0x0080;
static constexpr uint16_t CMD_MULTI_TARGET  = 0x0090;

// Decode coordinate/speed value from RD-03D format
static constexpr int16_t decode_value(uint8_t low, uint8_t high) {
  int16_t v = ((high & 0x7F) << 8) | low;
  if ((high & 0x80) == 0)
    v = -v;
  return v;
}

void RD03DComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up RD-03D...");
  this->set_timeout(100, [this]() { this->apply_config_(); });
}

void RD03DComponent::loop() {
  while (this->available()) {
    uint8_t byte = this->read();

    // --- Sync detection ---
    if (this->buffer_pos_ < FRAME_SYNC_SIZE) {
      if (byte == FRAME_SYNC[this->buffer_pos_]) {
        this->buffer_[this->buffer_pos_++] = byte;
      } else {
        this->buffer_pos_ = (byte == FRAME_SYNC[0]) ? 1 : 0;
        this->buffer_[0] = byte;
      }
      continue;
    }

    // Store byte
    this->buffer_[this->buffer_pos_++] = byte;

    // Length byte
    if (this->buffer_pos_ == FRAME_SYNC_SIZE + 1) {
      this->expected_length_ = byte;
      this->total_frame_length_ =
          FRAME_SYNC_SIZE + 1 + this->expected_length_ + 2;

      if (this->total_frame_length_ > sizeof(this->buffer_)) {
        ESP_LOGW(TAG, "Frame too large, dropping");
        this->buffer_pos_ = 0;
      }
      continue;
    }

    // Full frame received
    if (this->buffer_pos_ == this->total_frame_length_) {
      if (this->buffer_[this->buffer_pos_ - 2] == FRAME_FOOTER[0] &&
          this->buffer_[this->buffer_pos_ - 1] == FRAME_FOOTER[1]) {
        this->process_frame_();
      } else {
        ESP_LOGW(TAG, "Invalid frame footer");
      }
      this->buffer_pos_ = 0;
    }
  }
}

void RD03DComponent::process_frame_() {
  uint8_t target_count = 0;

  for (uint8_t i = 0; i < MAX_TARGETS; i++) {
    uint8_t offset = FRAME_SYNC_SIZE + 1 + (i * TARGET_DATA_SIZE);

    int16_t x = decode_value(this->buffer_[offset + 0],
                             this->buffer_[offset + 1]);
    int16_t y = decode_value(this->buffer_[offset + 2],
                             this->buffer_[offset + 3]);
    int16_t speed = decode_value(this->buffer_[offset + 4],
                                 this->buffer_[offset + 5]);
    uint16_t resolution =
        (this->buffer_[offset + 7] << 8) | this->buffer_[offset + 6];

    bool present = (x != 0 || y != 0);
    if (present)
      target_count++;

#ifdef USE_SENSOR
    this->publish_target_(i, present, x, y, speed, resolution);
#endif

#ifdef USE_BINARY_SENSOR
    if (this->target_presence_[i] != nullptr)
      this->target_presence_[i]->publish_state(present);
#endif
  }

#ifdef USE_SENSOR
  if (this->target_count_sensor_ != nullptr)
    this->target_count_sensor_->publish_state(target_count);
#endif

#ifdef USE_BINARY_SENSOR
  if (this->target_binary_sensor_ != nullptr)
    this->target_binary_sensor_->publish_state(target_count > 0);
#endif
}

#ifdef USE_SENSOR
void RD03DComponent::publish_target_(uint8_t idx, bool present,
                                     int16_t x, int16_t y,
                                     int16_t speed, uint16_t res) {
  auto &t = this->targets_[idx];

  if (t.x)
    t.x->publish_state(present ? x : NAN);
  if (t.y)
    t.y->publish_state(present ? y : NAN);
  if (t.speed)
    t.speed->publish_state(present ? speed * 10.0f : 0.0f);
  if (t.resolution)
    t.resolution->publish_state(res);

  if (t.distance)
    t.distance->publish_state(
        present ? std::hypot((float)x, (float)y) : NAN);

  if (t.angle)
    t.angle->publish_state(
        present ? atan2((float)x, (float)y) * 180.0f / M_PI : NAN);
}
#endif

void RD03DComponent::send_command_(uint16_t cmd,
                                  const uint8_t *data,
                                  uint8_t len) {
  this->write_array(CMD_FRAME_HEADER, sizeof(CMD_FRAME_HEADER));
  uint16_t l = 2 + len;
  this->write_byte(l & 0xFF);
  this->write_byte(l >> 8);
  this->write_byte(cmd & 0xFF);
  this->write_byte(cmd >> 8);
  if (data && len)
    this->write_array(data, len);
  this->write_array(CMD_FRAME_FOOTER, sizeof(CMD_FRAME_FOOTER));
}

void RD03DComponent::apply_config_() {
  if (!this->tracking_mode_.has_value())
    return;

  this->send_command_(
      *this->tracking_mode_ == TrackingMode::SINGLE_TARGET
          ? CMD_SINGLE_TARGET
          : CMD_MULTI_TARGET);
}

}  // namespace esphome::rd03d
