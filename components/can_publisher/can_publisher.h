#pragma once

#include "esphome.h"
#include "driver/twai.h"

using namespace esphome;

static inline int16_t f_to_i100(float v) { return (int16_t)(v * 100.0f); }
static inline int16_t f_to_i10(float v)  { return (int16_t)(v * 10.0f);  }
static inline int16_t f_to_i1(float v)   { return (int16_t)(v);          }
static inline int16_t b_to_i(bool v)     { return v ? 1 : 0;             }

class CANPublisher : public Component {
 public:
  void setup() override;
  void loop() override;

 private:
  void send_frame(uint32_t id, int16_t a, int16_t b, int16_t c, int16_t d);
  void log_frame(uint32_t id, int16_t a, int16_t b, int16_t c, int16_t d, const uint8_t *raw);
};
