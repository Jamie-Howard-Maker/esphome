#pragma once

#include "esphome/core/component.h"
#include "esphome/components/esp32_can/esp32_can.h"
#include "esphome/components/script/script.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"
#include "esphome/components/modbus_controller/modbus_controller.h"

namespace esphome {
namespace canreceiver {

struct CanMapping {
  uint32_t can_id;
  std::string type;
  script::Script<> *script;                 // Script with no parameters
  light::LightState *light;
  switch_::Switch *sw;
  number::Number *num;
  modbus_controller::ModbusController *ctrl;
  uint16_t reg;
};

class CanReceiver : public Component {
 public:
  // Use ESP32Can, not generic CANBus
  void set_canbus(esp32_can::ESP32Can *bus) { this->bus_ = bus; }

  void add_mapping(uint32_t can_id, const std::string &type,
                   script::Script<> *script, light::LightState *light,
                   switch_::Switch *sw, number::Number *num,
                   modbus_controller::ModbusController *ctrl, uint16_t reg) {
    CanMapping m;
    m.can_id = can_id;
    m.type = type;
    m.script = script;
    m.light = light;
    m.sw = sw;
    m.num = num;
    m.ctrl = ctrl;
    m.reg = reg;
    this->mappings_.push_back(m);
  }

  void loop() override;

 protected:
  esp32_can::ESP32Can *bus_{nullptr};
  std::vector<CanMapping> mappings_;

  void process_frame(const esp32_can::CanFrame &frame);
  void execute_mapping(const CanMapping &m, uint8_t cmd, uint8_t target,
                       uint8_t value, uint8_t flags);
};

}  // namespace canreceiver
}  // namespace esphome
