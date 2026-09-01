#include "jbd_bms_ble.h"
#include "esphome/core/log.h"

namespace esphome {
namespace jbd_bms_ble {

static const char *TAG = "jbd_bms_ble";

void JbdBmsBle::setup() {
    this->parent_->register_ble_node(this);
}

void JbdBmsBle::dump_config() {
    ESP_LOGCONFIG(TAG, "Redodo BMS BLE (E&J Protocol)");
}

// Допоміжна функція для збирання байтів
int32_t JbdBmsBle::get_int(const std::vector<uint8_t> &data, size_t offset, size_t size, bool is_signed) {
    uint32_t res = 0;
    for (size_t i = 0; i < size; i++) {
        res |= (uint32_t)data[offset + i] << (8 * i);
    }
    if (is_signed) {
        if (size == 4 && (res & 0x80000000)) return (int32_t)res;
        if (size == 2 && (res & 0x8000)) return (int16_t)res;
    }
    return res;
}

void JbdBmsBle::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    if (event == ESP_GATTC_SEARCH_CMPL_EVT) {
        auto *service = this->parent_->get_service(esp32_ble_tracker::ESPBTUUID::from_uint16(0xFFE0));
        if (service != nullptr) {
            this->char_notify_ = service->get_characteristic(esp32_ble_tracker::ESPBTUUID::from_uint16(0xFFE1));
            this->char_write_ = service->get_characteristic(esp32_ble_tracker::ESPBTUUID::from_uint16(0xFFE2));
            if (this->char_notify_ != nullptr) {
                esp_ble_gattc_register_for_notify(gattc_if, this->parent_->get_remote_bda(), this->char_notify_->handle);
            }
        }
    } else if (event == ESP_GATTC_NOTIFY_EVT && param->notify.handle == this->char_notify_->handle) {
        std::vector<uint8_t> data(param->notify.value, param->notify.value + param->notify.value_len);
        this->decode_data_(data);
    }
}

void JbdBmsBle::update() {
    if (this->parent_->connected() && this->char_write_ != nullptr) {
        // Команда запиту даних
        std::vector<uint8_t> request = {0x00, 0x00, 0x04, 0x01, 0x13, 0x55, 0xAA, 0x17};
        this->char_write_->write_value(request.data(), request.size(), ESP_GATT_WRITE_TYPE_NO_RSP);
    }
}

void JbdBmsBle::decode_data_(const std::vector<uint8_t> &data) {
    if (data.size() < 100) return;

    float voltage = this->get_int(data, 12, 2, false) / 1000.0f;
    float current = this->get_int(data, 48, 4, true) / 1000.0f;
    float cell_temp = (float)this->get_int(data, 52, 2, false);
    float mosfet_temp = (float)this->get_int(data, 54, 2, false);
    float soc = (float)this->get_int(data, 90, 2, false);
    float soh = (float)this->get_int(data, 92, 4, false);
    uint32_t cycles = this->get_int(data, 96, 4, false);
    uint32_t equilibriumState = this->get_int(data, 84, 4, false);
    uint32_t batteryState = this->get_int(data, 88, 2, false);
    float charge = this->get_int(data, 62, 2, false) / 100.0f;
    float capacity = this->get_int(data, 64, 2, false) / 100.0f;

    if (this->voltage_sensor_) this->voltage_sensor_->publish_state(voltage);
    if (this->current_sensor_) this->current_sensor_->publish_state(current);
    if (this->cell_temp_sensor_) this->cell_temp_sensor_->publish_state(cell_temp);
    if (this->mosfet_temp_sensor_) this->mosfet_temp_sensor_->publish_state(mosfet_temp);
    if (this->soc_sensor_) this->soc_sensor_->publish_state(soc);
    if (this->soh_sensor_) this->soh_sensor_->publish_state(soh);
    if (this->cycles_sensor_) this->cycles_sensor_->publish_state(cycles);
    if (this->equilibriumstate_sensor_) this->equilibriumstate_sensor_->publish_state(equilibriumState);
    if (this->batterystate_sensor_) this->batterystate_sensor_->publish_state(batteryState);
    if (this->charge_sensor_) this->charge_sensor_->publish_state(charge);
    if (this->capacity_sensor_) this->capacity_sensor_->publish_state(capacity);

    for (int i = 0; i < 8; i++) {
        if (this->cells_[i]) {
            float cell_v = this->get_int(data, 16 + (i * 2), 2, false) / 1000.0f;
            this->cells_[i]->publish_state(cell_v);
        }
    }
    ESP_LOGI(TAG, "Redodo decoded: %.2fV, SOC: %.0f%%", voltage, soc);
}

} // namespace jbd_bms_ble
} // namespace esphome