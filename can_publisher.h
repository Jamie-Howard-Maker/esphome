#include "esphome.h"
#include "driver/twai.h"

using namespace esphome;

static inline int16_t f_to_i100(float v) { return (int16_t)(v * 100.0f); }
static inline int16_t f_to_i10(float v)  { return (int16_t)(v * 10.0f);  }
static inline int16_t f_to_i1(float v)   { return (int16_t)(v);          }
static inline int16_t b_to_i(bool v)     { return v ? 1 : 0;             }

class CANPublisher : public Component {
 public:

void setup() override {
    // LilyGO T-CAN485 pinout
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        GPIO_NUM_5,   // CAN TX
        GPIO_NUM_4,   // CAN RX
        TWAI_MODE_NORMAL
    );

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();

    ESP_LOGI("CAN", "TWAI CAN bus started on T-CAN485 (TX=5, RX=4)");
}


  void log_frame(uint32_t id, int16_t a, int16_t b, int16_t c, int16_t d, const uint8_t *raw) {
    ESP_LOGI("CAN",
             "TX ID 0x%03X | RAW: %02X %02X %02X %02X %02X %02X %02X %02X | A=%d B=%d C=%d D=%d",
             id,
             raw[0], raw[1], raw[2], raw[3],
             raw[4], raw[5], raw[6], raw[7],
             a, b, c, d);
  }

  void send_frame(uint32_t id, int16_t a, int16_t b, int16_t c, int16_t d) {
    twai_message_t msg;
    msg.identifier = id;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;

    msg.data[0] = (uint8_t)(a >> 8);
    msg.data[1] = (uint8_t)(a & 0xFF);
    msg.data[2] = (uint8_t)(b >> 8);
    msg.data[3] = (uint8_t)(b & 0xFF);
    msg.data[4] = (uint8_t)(c >> 8);
    msg.data[5] = (uint8_t)(c & 0xFF);
    msg.data[6] = (uint8_t)(d >> 8);
    msg.data[7] = (uint8_t)(d & 0xFF);

    // Log BEFORE sending
    log_frame(id, a, b, c, d, msg.data);

    esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(10));

    if (result == ESP_OK) {
      ESP_LOGI("CAN", "TX OK ID 0x%03X", id);
    } else {
      ESP_LOGE("CAN", "TX FAIL ID 0x%03X (err=%d)", id, result);
    }
  }

  void loop() override {
   // ─────────────────────────────────────────────
    // JBD BMS 0  (0x200–0x20F)
    // ─────────────────────────────────────────────
    send_frame(0x200,
               f_to_i100(id(bms0_total_voltage).state),
               f_to_i10(id(bms0_current).state),
               f_to_i1(id(bms0_state_of_charge).state),
               f_to_i1(id(bms0_power).state));

    send_frame(0x201,
               f_to_i100(id(bms0_average_cell_voltage).state),
               f_to_i100(id(bms0_delta_cell_voltage).state),
               f_to_i1(id(bms0_nominal_capacity).state),
               f_to_i1(id(bms0_capacity_remaining).state));

    send_frame(0x202,
               f_to_i100(id(bms0_cell_voltage_1).state),
               f_to_i100(id(bms0_cell_voltage_2).state),
               f_to_i100(id(bms0_cell_voltage_3).state),
               f_to_i100(id(bms0_cell_voltage_4).state));

    send_frame(0x203,
               f_to_i1(id(bms0_charging_power).state),
               f_to_i1(id(bms0_discharging_power).state),
               f_to_i1(id(bms0_charging_cycles).state),
               f_to_i1(id(bms0_battery_cycle_capacity).state));

    send_frame(0x204,
               b_to_i(id(bms0_balancing).state),
               b_to_i(id(bms0_charging).state),
               b_to_i(id(bms0_discharging).state),
               b_to_i(id(bms0_online_status).state));

    send_frame(0x205,
               b_to_i(id(bms0_cell_overvoltage_protection).state),
               b_to_i(id(bms0_cell_undervoltage_protection).state),
               b_to_i(id(bms0_pack_overvoltage_protection).state),
               b_to_i(id(bms0_pack_undervoltage_protection).state));

    send_frame(0x206,
               b_to_i(id(bms0_charge_overtemperature_protection).state),
               b_to_i(id(bms0_charge_undertemperature_protection).state),
               b_to_i(id(bms0_discharge_overtemperature_protection).state),
               b_to_i(id(bms0_discharge_undertemperature_protection).state));

    send_frame(0x207,
               b_to_i(id(bms0_charge_overcurrent_protection).state),
               b_to_i(id(bms0_discharge_overcurrent_protection).state),
               b_to_i(id(bms0_short_circuit_protection).state),
               b_to_i(id(bms0_ic_frontend_error).state));

    send_frame(0x208,
               b_to_i(id(bms0_mosfet_software_lock).state),
               b_to_i(id(bms0_auto_balance).state),
               b_to_i(id(bms0_clear_alarm).state),   // or alarm flag if you expose one
               f_to_i1(id(bms0_charging_cycles).state)); // reuse as "error count" if needed

    // text_sensor enums/strings: you can map to small ints on sender or receiver
    send_frame(0x209,
               f_to_i1(0), // operation_status enum index (map on receiver)
               f_to_i1(0), // device_model enum index
               f_to_i1(0), // errors bitmask
               f_to_i1(0)); // balancing_cells bitmask

    // ─────────────────────────────────────────────
    // JBD BMS 1  (0x210–0x21F)
    // ─────────────────────────────────────────────
    send_frame(0x210,
               f_to_i100(id(bms1_total_voltage).state),
               f_to_i10(id(bms1_current).state),
               f_to_i1(id(bms1_state_of_charge).state),
               f_to_i1(id(bms1_power).state));

    send_frame(0x211,
               f_to_i100(id(bms1_average_cell_voltage).state),
               f_to_i100(id(bms1_delta_cell_voltage).state),
               f_to_i1(id(bms1_nominal_capacity).state),
               f_to_i1(id(bms1_capacity_remaining).state));

    send_frame(0x212,
               f_to_i100(id(bms1_cell_voltage_1).state),
               f_to_i100(id(bms1_cell_voltage_2).state),
               f_to_i100(id(bms1_cell_voltage_3).state),
               f_to_i100(id(bms1_cell_voltage_4).state));

    send_frame(0x213,
               f_to_i1(id(bms1_charging_power).state),
               f_to_i1(id(bms1_discharging_power).state),
               f_to_i1(id(bms1_charging_cycles).state),
               f_to_i1(id(bms1_battery_cycle_capacity).state));

    send_frame(0x214,
               b_to_i(id(bms1_balancing).state),
               b_to_i(id(bms1_charging).state),
               b_to_i(id(bms1_discharging).state),
               b_to_i(id(bms1_online_status).state));

    send_frame(0x215,
               b_to_i(id(bms1_cell_overvoltage_protection).state),
               b_to_i(id(bms1_cell_undervoltage_protection).state),
               b_to_i(id(bms1_pack_overvoltage_protection).state),
               b_to_i(id(bms1_pack_undervoltage_protection).state));

    send_frame(0x216,
               b_to_i(id(bms1_charge_overtemperature_protection).state),
               b_to_i(id(bms1_charge_undertemperature_protection).state),
               b_to_i(id(bms1_discharge_overtemperature_protection).state),
               b_to_i(id(bms1_discharge_undertemperature_protection).state));

    send_frame(0x217,
               b_to_i(id(bms1_charge_overcurrent_protection).state),
               b_to_i(id(bms1_discharge_overcurrent_protection).state),
               b_to_i(id(bms1_short_circuit_protection).state),
               b_to_i(id(bms1_ic_frontend_error).state));

    send_frame(0x218,
               b_to_i(id(bms1_mosfet_software_lock).state),
               b_to_i(id(bms1_auto_balance).state),
               b_to_i(id(bms1_clear_alarm).state),
               f_to_i1(id(bms1_charging_cycles).state));

    send_frame(0x219,
               f_to_i1(0), // operation_status enum index
               f_to_i1(0), // device_model enum index
               f_to_i1(0), // errors bitmask
               f_to_i1(0)); // balancing_cells bitmask

    // ─────────────────────────────────────────────
    // Victron SmartSolar (0x220–0x22F)
    // ─────────────────────────────────────────────
    send_frame(0x220,
               f_to_i100(id(vic0_mppt_output_voltage).state),
               f_to_i10(id(vic0_mppt_output_current).state),
               f_to_i1(id(vic0_mppt_pv_power).state),
               f_to_i10(id(vic0_mppt_load_current).state));

    send_frame(0x221,
               f_to_i1(id(vic0_mppt_yield_today).state),
               f_to_i1(0), // MPPT state enum index (from MySmartSolar_MPPT_t)
               f_to_i1(0), // MPPT error enum index (from MySmartSolar_error_t)
               b_to_i(id(vic0_mppt_fault).state));

    // ─────────────────────────────────────────────
    // Victron SmartShunt (0x230–0x23F)
    // ─────────────────────────────────────────────
    send_frame(0x230,
               f_to_i100(id(vic1_system_voltage).state),
               f_to_i10(id(vic1_system_current).state),
               f_to_i1(id(vic1_system_power).state),
               f_to_i1(id(vic1_system_soc).state));

    send_frame(0x231,
               f_to_i1(id(vic1_consumed_ah).state),
               f_to_i1(id(vic1_ttg).state),
               f_to_i100(id(vic1_aux_voltage).state),
               b_to_i(id(vic1_alarm).state));

    send_frame(0x232,
               f_to_i1(0), // Alarm Reason enum index (MySmartShunt_alarm_reason)
               f_to_i1(0), // Warning Reason (if you add)
               f_to_i1(0), // Error Code (if any)
               f_to_i1(0));

    // ─────────────────────────────────────────────
    // Victron BatteryProtect (0x240–0x24F)
    // ─────────────────────────────────────────────
    send_frame(0x240,
               f_to_i100(id(vic2_input_voltage).state),
               f_to_i100(id(vic2_output_voltage).state),
               f_to_i1(0), // Output State enum index (MySmartBatteryProtect_output_state_t)
               f_to_i1(0)); // Device State enum index (MySmartBatteryProtect_state_t)

    send_frame(0x241,
               f_to_i1(0), // Alarm Reason enum index
               f_to_i1(0), // Warning Reason enum index
               f_to_i1(0), // Error Code enum index
               f_to_i1(0)); // Off Reason enum index

    // ─────────────────────────────────────────────
    // Victron Orion DC-DC (0x250–0x25F)
    // ─────────────────────────────────────────────
    send_frame(0x250,
               f_to_i100(id(vic3_input_voltage).state),
               f_to_i100(id(vic3_output_voltage).state),
               f_to_i1(0), // State enum index (MySmartOrionDCDC_state_t)
               f_to_i1(0)); // Error enum index (MySmartOrionDCDC_error_t)

    send_frame(0x251,
               f_to_i1(0), // Off Reason enum index (MySmartOrionDCDC_off_reason_t)
               f_to_i1(0), // Charger Error enum index
               b_to_i(id(vic3_fault).state),
               b_to_i(id(vic3_charger_error).state));

    // Adjust rate as needed; this is your CAN publis
    delay(500);
  }
};
