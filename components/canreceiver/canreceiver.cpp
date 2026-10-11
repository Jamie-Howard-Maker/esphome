#include "canreceiver.h"
#include "esphome/core/log.h"

namespace esphome {
namespace canreceiver {

static const char *const TAG = "canreceiver";

void CanReceiver::setup() {
  if (this->bus_ == nullptr) {
    ESP_LOGW(TAG, "No CAN bus configured for canreceiver");
    return;
  }

  // Use callback API from esp32_can / canbus
  this->bus_->add_callback(
      [this](uint32_t can_id, bool extended, bool rtr, const std::vector<uint8_t> &data) {
        (void) extended;
        (void) rtr;
        this->process_frame(can_id, data);
      });
}

void CanReceiver::add_mapping(uint32_t can_id, const std::string &type,
                              script::Script<> *script, light::LightState *light,
                              switch_::Switch *sw, number::Number *number,
                              modbus_controller::ModbusController *ctrl,
                              uint16_t reg) {
  CanMapping m;
  m.can_id = can_id;
  m.type = type;
  m.script = script;
  m.light = light;
  m.sw = sw;
  m.number = number;
  m.ctrl = ctrl;
  m.reg = reg;
  this->mappings_.push_back(m);
}

void CanReceiver::process_frame(uint32_t can_id,
                                const std::vector<uint8_t> &data) {
  if (data.size() < 4)
    return;

  uint8_t cmd = data[0];
  uint8_t target = data[1];
  uint8_t value = data[2];
  uint8_t flags = data[3];

  for (auto &m : this->mappings_) {
    if (m.can_id == can_id) {
      this->execute_mapping(m, cmd, target, value, flags);
    }
  }
}

void CanReceiver::execute_mapping(const CanMapping &m,
                                  uint8_t cmd, uint8_t target,
                                  uint8_t value, uint8_t flags) {
  // Shelly per‑channel brightness
  if (m.type == CAN_RECEIVER_TYPE_SHELLY_CHANNEL) {
    if (m.script != nullptr && cmd == 0x01) {  // Set brightness
      uint8_t ch = target;
      m.script->execute();  // Script has no parameters in ESPHome C++ API
    }
    return;
  }

  // Shelly all‑channels brightness
  if (m.type == CAN_RECEIVER_TYPE_SHELLY_ALL) {
    if (m.script != nullptr && cmd == 0x01) {
      m.script->execute();  // Again, no arguments
    }
    return;
  }

  // NeoPixel brightness (mapped to a light)
  if (m.type == CAN_RECEIVER_TYPE_NEOPIXEL_BRIGHTNESS) {
    if (m.light != nullptr && cmd == 0x01) {
      float brightness = value / 255.0f;
      auto call = m.light->make_call();
      call.set_brightness(brightness);
      call.perform();
    }
    return;
  }

  // Switch toggle
  if (m.type == CAN_RECEIVER_TYPE_SWITCH_TOGGLE) {
    if (m.switch_ != nullptr && cmd == 0x02) {  // Toggle
      m.switch_->toggle();
    }
    return;
  }

  // Number set
  if (m.type == CAN_RECEIVER_TYPE_NUMBER_SET) {
    if (m.number != nullptr && cmd == 0x01) {  // Set value
      m.number->publish_state(value);
    }
    return;
  }

  // Modbus write (coil/register) – **fixed calls here**
  if (m.type == CAN_RECEIVER_TYPE_MODBUS_WRITE) {
    if (m.ctrl != nullptr) {
      // flags bit 0: 0 = coil, 1 = register (example convention)
      bool is_register = (flags & 0x01) != 0;

      if (is_register) {
        // OLD: m.ctrl->write_single_register(m.reg, value);
        m.ctrl->write_register(m.reg, value);
      } else {
        // OLD: m.ctrl->write_single_coil(m.reg, value != 0);
        m.ctrl->write_coil(m.reg, value != 0);
      }
    }
    return;
  }

  // Plain script trigger
  if (m.type == CAN_RECEIVER_TYPE_SCRIPT) {
    if (m.script != nullptr && cmd == 0x03) {
      m.script->execute();  // no arguments
    }
    return;
  }
}


}  // namespace canreceiver
}  // namespace esphome
