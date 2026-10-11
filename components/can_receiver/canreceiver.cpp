#include "canreceiver.h"
#include "esphome/core/log.h"

namespace esphome {
namespace canreceiver {

static const char *const TAG = "canreceiver";

void CanReceiver::loop() {
  if (this->bus_ == nullptr)
    return;

  canbus::CANFrame frame;
  while (this->bus_->receive(&frame)) {
    this->process_frame(frame);
  }
}

void CanReceiver::process_frame(const canbus::CANFrame &frame) {
  if (frame.data.size() < 4)
    return;

  uint8_t cmd = frame.data[0];
  uint8_t target = frame.data[1];
  uint8_t value = frame.data[2];
  uint8_t flags = frame.data[3];

  ESP_LOGV(TAG, "RX ID=0x%03X CMD=0x%02X TGT=%u VAL=%u FLG=0x%02X",
           frame.can_id, cmd, target, value, flags);

  for (auto &m : this->mappings_) {
    if (m.can_id == frame.can_id) {
      this->execute_mapping(m, cmd, target, value, flags);
      return;
    }
  }
}

void CanReceiver::execute_mapping(const CanMapping &m, uint8_t cmd,
                                  uint8_t target, uint8_t value,
                                  uint8_t flags) {
  // Uniform payload:
  // cmd   = Byte 0 (command type)
  // target= Byte 1 (channel/entity index)
  // value = Byte 2 (0–255)
  // flags = Byte 3 (bitmask)

  if (m.type == "shelly_channel") {
    if (m.script != nullptr && cmd == 0x01) {  // Set brightness
      uint8_t ch = target;
      if (value > 100)
        value = 100;
      ESP_LOGD(TAG, "Shelly channel: ch=%u val=%u", ch, value);
      m.script->execute({ch, value});
    }
    return;
  }

  if (m.type == "shelly_all") {
    if (m.script != nullptr && cmd == 0x01) {
      if (value > 100)
        value = 100;
      ESP_LOGD(TAG, "Shelly all: val=%u", value);
      m.script->execute({value});
    }
    return;
  }

  if (m.type == "neopixel_brightness") {
    if (m.light != nullptr && cmd == 0x01) {
      float br = value / 100.0f;
      ESP_LOGD(TAG, "NeoPixel brightness: val=%u → %f", value, br);
      auto call = m.light->make_call();
      call.set_brightness(br);
      call.perform();
    }
    return;
  }

  if (m.type == "switch_toggle") {
    if (m.sw != nullptr && cmd == 0x02) {
      ESP_LOGD(TAG, "Switch toggle: val=%u", value);
      if (value)
        m.sw->turn_on();
      else
        m.sw->turn_off();
    }
    return;
  }

  if (m.type == "number_set") {
    if (m.num != nullptr && cmd == 0x04) {
      float v = value;
      ESP_LOGD(TAG, "Number set: val=%u", value);
      m.num->publish_state(v);
    }
    return;
  }

  if (m.type == "modbus_write") {
    if (m.ctrl != nullptr && cmd == 0x07) {
      ESP_LOGD(TAG, "Modbus write: reg=%u val=%u", m.reg, value);
      m.ctrl->write_single_register(m.reg, value);
    }
    return;
  }

  if (m.type == "script") {
    if (m.script != nullptr && cmd == 0x03) {
      ESP_LOGD(TAG, "Script execute: target=%u val=%u", target, value);
      m.script->execute({});
    }
    return;
  }

  ESP_LOGW(TAG, "Unknown mapping type: %s", m.type.c_str());
}

}  // namespace can_receiver
}  // namespace esphome
