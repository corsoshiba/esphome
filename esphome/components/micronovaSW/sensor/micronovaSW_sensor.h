#pragma once

#include "esphome/components/micronovaSW/micronovaSW.h"
#include "esphome/components/sensor/sensor.h"


namespace esphome {
namespace micronovaSW {

class MicroNovaSWSensor : public sensor::Sensor, public MicroNovaSWSensorListener {
 public:
  MicroNovaSWSensor(MicroNovaSW *m) : MicroNovaSWSensorListener(m) {}
  void dump_config() override { LOG_SENSOR("", "Micronova sensor", this); }

  void request_value_from_stove() override {
    this->micronovaSW_->request_address(this->memory_location_, this->memory_address_, this);
  }
  void process_value_from_stove(int value_from_stove) override;

  void set_fan_speed_offset(uint8_t f) { this->fan_speed_offset_ = f; }
  uint8_t get_set_fan_speed_offset() { return this->fan_speed_offset_; }

 protected:
  int fan_speed_offset_ = 0;
};

}  // namespace micronovaSW
}  // namespace esphome
