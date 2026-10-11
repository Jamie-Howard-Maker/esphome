#include "canreceiver.h"
#include "esphome/core/log.h"

namespace esphome {
namespace canreceiver {

static const char *const TAG = "canreceiver";

void CanReceiver::loop() {
  if (this->bus_ == nullptr)
    return;

  canbus::CanFrame frame;
  while (this->bus_->read_frame(&frame)) {
    this->process_frame(frame);
  }
}

void CanReceiver::process_frame(const canbus::CanFrame &frame) {
  if (frame.data_length < 4)
    return;

  uint8_t cmd    = frame.data[0];
  uint8_t target = frame.data[1];
  uint8_t value  = frame.data[2];
  uint8_t flags  = frame.data[3];

  for (auto &m : this->mappings_) {
    if (m.can_id == frame.can_id) {
      this->execute_mapping(m, cmd, target, value, flags);
    }
  }
}

void CanReceiver::execute_mapping(const CanMapping &m,
                                  uint8_t cmd, uint8_t target,
                                  uint8_t value, uint8_t flags) {
  ESP_LOGD(TAG, "Mapping type=%s cmd=%u target=%u value=%u flags=%u",
           m.type.c_str(), cmd, target, value, flags);

  // Shelly channel brightness
  if (m.type == "shelly_channel") {
    if (m.script != nullptr && cmd == 0x01) {
      m.script->execute();  // Script<> cannot take parameters in C++
    }
    return;
  }

  // Shelly all channels
  if (m.type == "shelly_all") {
    if (m.script != nullptr && cmd == 0x01) {
      m.script->execute();
    }
    return;
  }

  // NeoPixel brightness
  if (m.type == "neopixel_brightness") {
    if (m.light != nullptr && cmd == 0x01) {
      float br = value / 255.0f;
      auto call = m.light->make_call();
      call.set_brightness(br);
      call.perform();
    }
    return;
  }

  // Switch toggle
  if (m.type == "switch_toggle") {
    if (m.sw != nullptr && cmd == 0x02) {
      m.sw->toggle();
    }
    return;
  }

  // Number set
  if (m.type == "number_set") {
    if (m.num != nullptr && cmd == 0x04) {
      m.num->publish_state(value);
    }
    return;
  }

  // Modbus write (disabled until correct API known)
  if (m.type == "modbus_write") {
    ESP_LOGW(TAG, "modbus_write mapping present but no valid Modbus API available");
    return;
  }

  // Script trigger
  if (m.type == "script") {
    if (m.script != nullptr && cmd == 0x03) {
      m.script->execute();
    }
    return;
  }

  ESP_LOGW(TAG, "Unknown mapping type: %s", m.type.c_str());
}

}  // namespace canreceiver
}  // namespace esphome
