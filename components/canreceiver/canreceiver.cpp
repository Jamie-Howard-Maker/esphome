#include "canreceiver.h"

namespace esphome {
namespace canreceiver {

static const char *const TAG = "canreceiver";

void CanReceiver::setup() {
  if (bus_ == nullptr) {
    ESP_LOGW(TAG, "CAN receiver: no CAN bus configured");
    return;
  }

  // ESP32Can provides on-frame callback
  bus_->add_on_frame_callback([this](const esp32_can::CanFrame &frame) {
    this->process_frame(frame);
  });

  ESP_LOGI(TAG, "CAN receiver initialized");
}

void CanReceiver::process_frame(const esp32_can::CanFrame &frame) {
  // Expect at least 4 bytes: cmd, b0, b1, b2
  uint8_t cmd = frame.data[0];
  uint8_t b0  = frame.data[1];
  uint8_t b1  = frame.data[2];
  uint8_t b2  = frame.data[3];

  ESP_LOGD(TAG, "RX CAN ID=0x%03X cmd=%u b0=%u b1=%u b2=%u",
           frame.can_id, cmd, b0, b1, b2);

  for (auto &m : mappings_) {
    if (m.can_id == frame.can_id) {
      this->execute_mapping(m, cmd, b0, b1, b2);
    }
  }
}

void CanReceiver::execute_mapping(const CanMapping &m,
                                  uint8_t cmd,
                                  uint8_t b0,
                                  uint8_t b1,
                                  uint8_t b2) {
  uint16_t value = (uint16_t(b0) << 8) | b1;

  // Shelly channel brightness
  if (m.type == CAN_RECEIVER_TYPE_SHELLY_CHANNEL) {
    if (m.script != nullptr && cmd == 0x01) {
      m.script->execute();
    }
    return;
  }

  // Shelly all channels
  if (m.type == CAN_RECEIVER_TYPE_SHELLY_ALL) {
    if (m.script != nullptr && cmd == 0x01) {
      m.script->execute();
    }
    return;
  }

  // NeoPixel brightness
  if (m.type == CAN_RECEIVER_TYPE_NEOPIXEL_BRIGHTNESS) {
    if (m.light != nullptr && cmd == 0x01) {
      float br = value / 100.0f;
      auto call = m.light->make_call();
      call.set_brightness(br);
      call.perform();
    }
    return;
  }

  // Switch toggle
  if (m.type == CAN_RECEIVER_TYPE_SWITCH_TOGGLE) {
    if (m.sw != nullptr && cmd == 0x02) {
      m.sw->toggle();
    }
    return;
  }

  // Number set
  if (m.type == CAN_RECEIVER_TYPE_NUMBER_SET) {
    if (m.num != nullptr && cmd == 0x03) {
      m.num->publish_state(static_cast<float>(value));
    }
    return;
  }

  // Modbus write
  if (m.type == CAN_RECEIVER_TYPE_MODBUS_WRITE) {
    if (m.ctrl != nullptr && cmd == 0x04) {
      if (m.coil) {
        m.ctrl->write_coil(m.reg, value != 0);
      } else {
        m.ctrl->write_register(m.reg, value);
      }
    }
    return;
  }

  // Script trigger
  if (m.type == CAN_RECEIVER_TYPE_SCRIPT) {
    if (m.script != nullptr && cmd == 0x01) {
      m.script->execute();
    }
    return;
  }
}

}  // namespace canreceiver
}  // namespace esphome
