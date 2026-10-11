#pragma once
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "reg10_protocol.h"

namespace esphome { namespace reg10 {
class REG10 : public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  void on_shutdown() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void set_direction_pin(GPIOPin *pin) { direction_pin_ = pin; }
  void set_slave_address(uint8_t v) { slave_ = v; }
  void set_master_address(uint8_t v) { master_ = v; }
  void set_response_timeout(uint32_t v) { timeout_ms_ = v; }
  void set_request_gap(uint32_t v) { gap_ms_ = v; }
  void set_stale_after(uint32_t v) { stale_ms_ = v; }
  void set_retries(uint8_t v) { retries_ = v; }
  void set_tcp_port(uint16_t v) { tcp_port_ = v; }
  void set_tcp_unit_id(uint8_t v) { tcp_unit_ = v; }
  void set_temperature_sensor(size_t i, sensor::Sensor *s) { temperatures_[i] = s; }
  void set_flow_sensor(size_t i, sensor::Sensor *s) { flows_[i] = s; }
  void set_output_sensor(size_t i, binary_sensor::BinarySensor *s) { outputs_[i] = s; }
#define REG10_SENSOR_SETTER(name) void set_##name##_sensor(sensor::Sensor *s) { name##_ = s; }
  REG10_SENSOR_SETTER(outputs_raw)
  REG10_SENSOR_SETTER(valid_mask)
  REG10_SENSOR_SETTER(temperature_valid_mask)
  REG10_SENSOR_SETTER(good_frames)
  REG10_SENSOR_SETTER(crc_errors)
  REG10_SENSOR_SETTER(timeouts)
  REG10_SENSOR_SETTER(temperature_age)
  REG10_SENSOR_SETTER(output_age)
  REG10_SENSOR_SETTER(flow_age)
#undef REG10_SENSOR_SETTER
#define REG10_BINARY_SETTER(name) void set_##name##_sensor(binary_sensor::BinarySensor *s) { name##_ = s; }
  REG10_BINARY_SETTER(all_online)
  REG10_BINARY_SETTER(temperature_online)
  REG10_BINARY_SETTER(output_online)
  REG10_BINARY_SETTER(flow_online)
#undef REG10_BINARY_SETTER
 protected:
  void send_request_(uint32_t now);
  void handle_frame_(uint32_t now);
  void next_group_(uint32_t now);
  void publish_group_(uint8_t group);
  void publish_diagnostics_();
  void expire_states_();
  void tcp_loop_(uint32_t now);
  void start_server_();
  void close_server_();
  struct Client {
    int fd{-1};
    std::array<uint8_t, 520> rx{};
    std::array<uint8_t, 260> tx{};
    size_t rx_size{0}, tx_size{0}, tx_offset{0};
    uint32_t last_activity{0}, partial_started{0};
    void close();
  };
  void serve_client_(Client &c, uint32_t now);
  GPIOPin *direction_pin_{nullptr};
  uint8_t slave_{1}, master_{128}, retries_{2}, tcp_unit_{1};
  uint16_t tcp_port_{502};
  uint32_t timeout_ms_{1000}, gap_ms_{100}, stale_ms_{60000};
  bool polling_{false}, waiting_{false};
  uint8_t group_{0}, attempts_{0};
  uint32_t sent_at_{0}, next_request_at_{0}, last_rx_at_{0};
  uint32_t last_tick_{0}, last_diagnostics_{0}, last_listen_attempt_{0};
  bool tried_listen_{false};
  std::array<bool, 3> invalidated_{{false, false, false}};
  FrameDecoder decoder_;
  Snapshot snapshot_;
  int listener_{-1};
  std::array<Client, 2> clients_{};
  std::array<sensor::Sensor *, 4> temperatures_{};
  std::array<sensor::Sensor *, 2> flows_{};
  std::array<binary_sensor::BinarySensor *, 5> outputs_{};
  sensor::Sensor *outputs_raw_{nullptr}, *valid_mask_{nullptr}, *temperature_valid_mask_{nullptr};
  sensor::Sensor *good_frames_{nullptr}, *crc_errors_{nullptr}, *timeouts_{nullptr};
  sensor::Sensor *temperature_age_{nullptr}, *output_age_{nullptr}, *flow_age_{nullptr};
  binary_sensor::BinarySensor *all_online_{nullptr}, *temperature_online_{nullptr};
  binary_sensor::BinarySensor *output_online_{nullptr}, *flow_online_{nullptr};
};
}}  // namespace esphome::reg10
