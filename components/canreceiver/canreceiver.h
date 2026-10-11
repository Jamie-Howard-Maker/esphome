#pragma once

#include "esphome/core/component.h"
#include "esphome/components/canbus/canbus.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"
#include "esphome/components/script/script.h"
#include "esphome/components/modbus_controller/modbus_controller.h"

namespace esphome {
namespace canreceiver {

struct CanMapping {
  uint32_t can_id;
  std::string type;
  script::Script<> *script{nullptr};
  light::LightState *light{nullptr};
  switch_::Switch *sw{nullptr};
  number::Number *number{nullptr};
  modbus_controller::ModbusController *ctrl{nullptr};
  uint16_t reg{0};
};

class CanReceiver : public Component {
 public:
  void set_canbus(canbus::Canbus *bus) { this->bus_ = bus; }

  void add_mapping(uint32_t can_id, const std::string &type,
                   script::Script<> *script, light::LightState *light,
                   switch_::Switch *sw, number::Number *number,
                   modbus_controller::ModbusController *ctrl, uint16_t reg);

  void setup() override;
  void loop() override {}

 protected:
  void process_frame(uint32_t can_id, const std::vector<uint8_t> &data);
  void execute_mapping(const CanMapping &m, uint8_t cmd, uint8_t target,
                       uint8_t value, uint8_t flags);

  canbus::Canbus *bus_{nullptr};
  std::vector<CanMapping> mappings_;
};

}  // namespace canreceiver
}  // namespace esphome
