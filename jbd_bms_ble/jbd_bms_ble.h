#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace jbd_bms_ble {

class JbdBmsBle : public PollingComponent, public ble_client::BLEClientNode {
 public:
  // Сетери для ESPHome
  void set_voltage_sensor(sensor::Sensor *s) { voltage_sensor_ = s; }
  void set_current_sensor(sensor::Sensor *s) { current_sensor_ = s; }
  
  // ВИПРАВЛЕНО: назва має бути повною, як у sensor.py
  void set_cell_temperature_sensor(sensor::Sensor *s) { cell_temp_sensor_ = s; }
  void set_mosfet_temperature_sensor(sensor::Sensor *s) { mosfet_temp_sensor_ = s; }

  void set_soc_sensor(sensor::Sensor *s) { soc_sensor_ = s; }
  void set_soh_sensor(sensor::Sensor *s) { soh_sensor_ = s; }
  void set_cycles_sensor(sensor::Sensor *s) { cycles_sensor_ = s; }
  void set_equilibriumstate_sensor(sensor::Sensor *s) { equilibriumstate_sensor_ = s; }
  void set_batterystate_sensor(sensor::Sensor *s) { batterystate_sensor_ = s; }
  void set_charge_sensor(sensor::Sensor *s) { charge_sensor_ = s; }
  void set_capacity_sensor(sensor::Sensor *s) { capacity_sensor_ = s; }
  void set_cell_sensor(size_t index, sensor::Sensor *s) {
    if (index < 8) this->cells_[index] = s;
  }

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) override;
  void update() override;
  void setup() override;
  void dump_config() override;

 protected:
  void decode_data_(const std::vector<uint8_t> &data);
  int32_t get_int(const std::vector<uint8_t> &data, size_t offset, size_t size, bool is_signed);
  
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *cell_temp_sensor_{nullptr};
  sensor::Sensor *mosfet_temp_sensor_{nullptr};
  sensor::Sensor *soc_sensor_{nullptr};
  sensor::Sensor *soh_sensor_{nullptr};
  sensor::Sensor *cycles_sensor_{nullptr};
  sensor::Sensor *equilibriumstate_sensor_{nullptr};
  sensor::Sensor *batterystate_sensor_{nullptr};
  sensor::Sensor *charge_sensor_{nullptr};
  sensor::Sensor *capacity_sensor_{nullptr};
  sensor::Sensor *cells_[8]{nullptr};
  
  ble_client::BLECharacteristic *char_write_{nullptr};
  ble_client::BLECharacteristic *char_notify_{nullptr};
};

} // namespace jbd_bms_ble
} // namespace esphome