#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include "esphome/components/esp32_can/esp32_can.h"
#include "esphome/components/script/script.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"
#include "esphome/components/modbus_controller/modbus_controller.h"

#include <vector>

namespace esphome {
namespace canreceiver {

enum CanReceiverType : uint8_t {
  CAN_RECEIVER_TYPE_NONE = 0,
  CAN_RECEIVER_TYPE_SHELLY_CHANNEL,
  CAN_RECEIVER_TYPE_SHELLY_ALL,
  CAN_RECEIVER_TYPE_NEOPIXEL_BRIGHTNESS,
  CAN_RECEIVER_TYPE_SWITCH_TOGGLE,
  CAN_RECEIVER_TYPE_NUMBER_SET,
  CAN_RECEIVER_TYPE_MODBUS_WRITE,
  CAN_RECEIVER_TYPE_SCRIPT,
};

struct CanMapping {
  uint32_t can_id{0};
  CanReceiverType type{CAN_RECEIVER_TYPE_NONE};

  // Shelly
  script::Script<> *script{nullptr};

  // NeoPixel
  light::LightState *light{nullptr};

  // Switch
  switch_::Switch *sw{nullptr};

  // Number
  number::Number *num{nullptr};

  // Modbus
  modbus_controller::ModbusController *ctrl{nullptr};
  uint16_t reg{0};
  bool coil{false};
};

class CanReceiver : public Component {
 public:
  void set_canbus(esp32_can::ESP32Can *bus) { bus_ = bus; }
  void add_mapping(const CanMapping &m) { mappings_.push_back(m); }

  void setup() override;
  void loop() override {}  // CAN handled via callback

 protected:
  void process_frame(const esp32_can::CanFrame &frame);
  void execute_mapping(const CanMapping &m,
                       uint8_t cmd, uint8_t b0, uint8_t b1, uint8_t b2);

  esp32_can::ESP32Can *bus_{nullptr};
  std::vector<CanMapping> mappings_;
};

}  // namespace canreceiver
}  // namespace esphome
