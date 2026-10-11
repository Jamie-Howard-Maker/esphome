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

void CanReceiver::execute_mapping(const CanMapping &m, uint8_t cmd,
                                  uint8_t target, uint8_t value,
                                  uint8_t flags) {
  (void) flags;

  if (m.type == "shelly_channel") {
    if (m.light != nullptr && cmd == 0x01) {
      auto call = m.light->make_call();
      call.set_brightness(value / 255.0f);
      call.perform();
    }
    if (m.script != nullptr && cmd == 0x01) {
      m.script->execute();
    }

  } else if (m.type == "shelly_all") {
    if (m.light != nullptr && cmd == 0x01) {
      auto call = m.light->make_call();
      call.set_brightness(value / 255.0f);
      call.perform();
    }
    if (m.script != nullptr && cmd == 0x01) {
      m.script->execute();
    }

  } else if (m.type == "neopixel_brightness") {
    if (m.light != nullptr && cmd == 0x01) {
      auto call = m.light->make_call();
      call.set_brightness(value / 255.0f);
      call.perform();
    }

  } else if (m.type == "switch_toggle") {
    if (m.sw != nullptr && cmd == 0x02) {
      m.sw->toggle();
    }

  } else if (m.type == "number_set") {
    if (m.number != nullptr && cmd == 0x02) {
      m.number->publish_state(value);
    }

  } else if (m.type == "modbus_write") {
    if (m.ctrl != nullptr && cmd == 0x03) {
      // FIX: use existing ModbusController API
      m.ctrl->write_single_register(m.reg, value);
    }

  } else if (m.type == "script") {
    if (m.script != nullptr) {
      m.script->execute();
    }

  } else {
    ESP_LOGW(TAG, "Unknown mapping type '%s'", m.type.c_str());
  }
}

}  // namespace canreceiver
}  // namespace esphome
