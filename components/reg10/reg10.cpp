#include "reg10.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"
#include <cerrno>
#include <cmath>
#include <fcntl.h>
#include <lwip/sockets.h>
#include <unistd.h>

namespace esphome { namespace reg10 {
static const char *const TAG = "reg10";
static void publish(sensor::Sensor *s, float v) { if (s != nullptr) s->publish_state(v); }
static void publish(binary_sensor::BinarySensor *s, bool v) { if (s != nullptr) s->publish_state(v); }
static bool would_block() { return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR; }
static bool nonblocking(int fd) {
  const int flags = fcntl(fd, F_GETFL, 0);
  return flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}
void REG10::setup() {
  if (direction_pin_ == nullptr) {
    ESP_LOGE(TAG, "direction_pin is required for this RS485 interface");
    mark_failed();
    return;
  }
  direction_pin_->setup();
  direction_pin_->digital_write(false);  // Receive by default; never drive an idle bus.
  last_tick_ = millis(); last_rx_at_ = last_tick_;
  expire_states_(); publish_diagnostics_(); update();
}
void REG10::update() {
  if (polling_) return;  // Do not queue overlapping cycles when the bus is down.
  polling_ = true; waiting_ = false; group_ = 0; attempts_ = 0;
  next_request_at_ = millis() + gap_ms_;
}
void REG10::dump_config() {
  ESP_LOGCONFIG(TAG, "REG10 read-only; slave 0x%02X, master 0x%02X", slave_, master_);
  ESP_LOGCONFIG(TAG, "  Update %lu ms; timeout %lu ms; retries %u",
                (unsigned long) get_update_interval(), (unsigned long) timeout_ms_, retries_);
  ESP_LOGCONFIG(TAG, "  RS485 direction: component-controlled GPIO, TX flush before RX");
  ESP_LOGCONFIG(TAG, "  Stale after %lu ms; flows are RAW", (unsigned long) stale_ms_);
  ESP_LOGCONFIG(TAG, "  Modbus TCP port %u (0=disabled), unit %u; FC03/04 only", tcp_port_, tcp_unit_);
}
void REG10::loop() {
  const uint32_t now = millis();
  snapshot_.advance(now - last_tick_); last_tick_ = now;
  if (decoder_.active() && uint32_t(now - last_rx_at_) > 250) decoder_.reset();
  for (size_t budget = 0; budget < 512 && available() > 0; ++budget) {
    uint8_t b;
    if (!read_byte(&b)) break;
    last_rx_at_ = now;
    if (decoder_.feed(b)) handle_frame_(now);
  }
  if (waiting_ && uint32_t(now - sent_at_) >= timeout_ms_) {
    waiting_ = false; ++snapshot_.timeouts; decoder_.reset();
    const auto &r = READ_REQUESTS[group_];
    ESP_LOGW(TAG, "Timeout ctrl=0x%02X addr=0x%04X attempt=%u/%u",
             r.control, r.address, attempts_, uint8_t(retries_ + 1));
    if (attempts_ <= retries_) next_request_at_ = now + gap_ms_;
    else next_group_(now);
  }
  // This silence check is not multi-master arbitration.
  if (polling_ && !waiting_ && int32_t(now - next_request_at_) >= 0 &&
      uint32_t(now - last_rx_at_) >= gap_ms_) send_request_(now);
  expire_states_();
  if (uint32_t(now - last_diagnostics_) >= 5000) {
    last_diagnostics_ = now; publish_diagnostics_();
  }
  tcp_loop_(now);
}
void REG10::send_request_(uint32_t now) {
  const auto &r = READ_REQUESTS[group_];
  const auto wire = make_read_request(slave_, master_, r);
  decoder_.reset(); ++attempts_; waiting_ = true; sent_at_ = now;
  // ESPHome 2025.5.1 has no uart.flow_control_pin. REG10 owns DE/RE instead.
  // flush() waits for the UART transmitter, including its final stop bit.
  // Do not delay after flush: switch back to receive before the reply starts.
  direction_pin_->digital_write(true);
  delay_microseconds_safe(5);
  write_array(wire.data(), wire.size());
  flush();
  direction_pin_->digital_write(false);
  sent_at_ = millis();
  ESP_LOGD(TAG, "TX ctrl=0x%02X addr=0x%04X len=%u", r.control, r.address, r.count);
}
void REG10::next_group_(uint32_t now) {
  waiting_ = false; attempts_ = 0; ++group_;
  if (group_ >= READ_REQUESTS.size()) { polling_ = false; publish_diagnostics_(); }
  else next_request_at_ = now + gap_ms_;
}
void REG10::handle_frame_(uint32_t now) {
  if (!waiting_ || group_ >= READ_REQUESTS.size()) return;
  const uint8_t *b = decoder_.data();
  const auto result = validate_response(b, decoder_.size(), slave_, master_, READ_REQUESTS[group_]);
  if (result == ResponseResult::NOT_FOR_REQUEST) return;
  if (result == ResponseResult::BAD_CRC) {
    ++snapshot_.crc_errors; ESP_LOGW(TAG, "Response rejected: bad CRC"); return;
  }
  if (result != ResponseResult::OK) {
    ESP_LOGW(TAG, "Response rejected: length=%u reason=%u", unsigned(decoder_.size()), unsigned(result));
    return;
  }
  const uint8_t *data = b + 8;
  if (group_ == 0) {
    for (size_t i = 0; i < 4; ++i) snapshot_.temperatures[i] = read_be16_signed(data + 2*i);
  } else if (group_ == 1) snapshot_.outputs = data[0];
  else {
    snapshot_.flows[0] = read_be16(data); snapshot_.flows[1] = read_be16(data+2);
    ESP_LOGD(TAG, "Flow raw: 0x0048=%u 0x004A=%u", snapshot_.flows[0], snapshot_.flows[1]);
  }
  snapshot_.age_ms[group_] = 0; ++snapshot_.good_frames; invalidated_[group_] = false;
  publish_group_(group_); next_group_(now);
}
void REG10::publish_group_(uint8_t group) {
  if (group == 0) {
    for (size_t i = 0; i < 4; ++i) {
      const auto raw = snapshot_.temperatures[i];
      publish(temperatures_[i], temperature_valid(raw) ? raw / 10.0f : NAN);
      if (!temperature_valid(raw)) ESP_LOGW(TAG, "T%u invalid/Hi/Lo, raw=%d", unsigned(i+1), raw);
    }
  } else if (group == 1) {
    publish(outputs_raw_, snapshot_.outputs);
    for (size_t i = 0; i < 5; ++i) publish(outputs_[i], bool((snapshot_.outputs >> OUTPUT_BITS[i]) & 1));
  } else { publish(flows_[0], snapshot_.flows[0]); publish(flows_[1], snapshot_.flows[1]); }
}
void REG10::expire_states_() {
  const uint16_t mask = snapshot_.valid_mask(stale_ms_);
  for (uint8_t group = 0; group < 3; ++group) {
    if ((mask & (1U << group)) || invalidated_[group]) continue;
    invalidated_[group] = true;
    if (group == 0) for (auto *s : temperatures_) publish(s, NAN);
    else if (group == 1) {
      publish(outputs_raw_, NAN);
      // ESPHome 2025.5.1 BinarySensor has no invalidate_state() API.
      // Keep the last known booleans; NEVER invent OFF on communication loss.
      // Consumers must gate OUT1-OUT5 with the output_online connectivity sensor.
      // Numeric outputs_raw remains NaN while data are stale.
    } else for (auto *s : flows_) publish(s, NAN);
    publish_diagnostics_();
  }
}
void REG10::publish_diagnostics_() {
  const auto r = make_registers(snapshot_, stale_ms_);
  publish(valid_mask_, r[7]); publish(temperature_valid_mask_, r[8]);
  publish(good_frames_, snapshot_.good_frames); publish(crc_errors_, snapshot_.crc_errors);
  publish(timeouts_, snapshot_.timeouts);
  publish(temperature_age_, snapshot_.age_ms[0] == UINT32_MAX ? NAN : float(r[9]));
  publish(output_age_, snapshot_.age_ms[1] == UINT32_MAX ? NAN : float(r[10]));
  publish(flow_age_, snapshot_.age_ms[2] == UINT32_MAX ? NAN : float(r[11]));
  publish(all_online_, r[7] == 7); publish(temperature_online_, bool(r[7] & 1));
  publish(output_online_, bool(r[7] & 2)); publish(flow_online_, bool(r[7] & 4));
}
void REG10::Client::close() {
  if (fd >= 0) ::close(fd);
  fd = -1; rx_size = tx_size = tx_offset = 0;
}
void REG10::start_server_() {
  const int fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0) { ESP_LOGW(TAG, "TCP socket failed: %d", errno); return; }
  int yes = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  sockaddr_in address{};
  address.sin_family = AF_INET; address.sin_port = htons(tcp_port_);
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  if (!nonblocking(fd) || ::bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0 ||
      ::listen(fd, 2) < 0) {
    ESP_LOGW(TAG, "TCP listen failed: %d; UART polling continues", errno); ::close(fd); return;
  }
  listener_ = fd; ESP_LOGI(TAG, "Read-only Modbus TCP on port %u", tcp_port_);
}
void REG10::close_server_() {
  for (auto &c : clients_) c.close();
  if (listener_ >= 0) ::close(listener_);
  listener_ = -1;
}
void REG10::on_shutdown() {
  if (direction_pin_ != nullptr) direction_pin_->digital_write(false);
  close_server_();
}
void REG10::tcp_loop_(uint32_t now) {
  if (tcp_port_ == 0) return;
  if (!network::is_connected()) { close_server_(); tried_listen_ = false; return; }
  if (listener_ < 0) {
    if (!tried_listen_ || uint32_t(now - last_listen_attempt_) >= 5000) {
      tried_listen_ = true; last_listen_attempt_ = now; start_server_();
    }
    if (listener_ < 0) return;
  }
  sockaddr_in peer{};
  socklen_t size = sizeof(peer);
  int accepted = ::accept(listener_, reinterpret_cast<sockaddr *>(&peer), &size);
  if (accepted >= 0) {
    if (nonblocking(accepted)) for (auto &c : clients_) {
      if (c.fd >= 0) continue;
      c.fd = accepted; c.rx_size = c.tx_size = c.tx_offset = 0;
      c.last_activity = c.partial_started = now; accepted = -1; break;
    }
    if (accepted >= 0) ::close(accepted);
  } else if (!would_block()) {
    ESP_LOGW(TAG, "TCP accept failed: %d", errno); close_server_(); return;
  }
  for (auto &c : clients_) if (c.fd >= 0) serve_client_(c, now);
}
void REG10::serve_client_(Client &c, uint32_t now) {
  if (uint32_t(now-c.last_activity) > 120000 ||
      (c.rx_size && uint32_t(now-c.partial_started) > 2000)) { c.close(); return; }
  if (c.tx_offset < c.tx_size) {
    int flags = 0;
#ifdef MSG_NOSIGNAL
    flags = MSG_NOSIGNAL;
#endif
    const int sent = ::send(c.fd, c.tx.data()+c.tx_offset, c.tx_size-c.tx_offset, flags);
    if (sent > 0) { c.tx_offset += size_t(sent); c.last_activity = now; }
    else if (sent == 0 || !would_block()) { c.close(); return; }
    if (c.tx_offset < c.tx_size) return;
    c.tx_offset = c.tx_size = 0;
  }
  // Process an already buffered complete ADU before another recv(). This also
  // preserves queued requests when a client half-closes its sending side.
  bool complete = false;
  if (c.rx_size >= 6) {
    const uint16_t length = read_be16(c.rx.data()+4);
    if (read_be16(c.rx.data()+2) != 0 || length < 2 || length > 254) { c.close(); return; }
    complete = c.rx_size >= size_t(6 + length);
  }
  if (!complete) {
    if (c.rx_size == c.rx.size()) { c.close(); return; }
    const int received = ::recv(c.fd, c.rx.data()+c.rx_size, c.rx.size()-c.rx_size, 0);
    if (received > 0) {
      if (c.rx_size == 0) c.partial_started = now;
      c.rx_size += size_t(received); c.last_activity = now;
    } else if (received == 0 || !would_block()) { c.close(); return; }
  }
  if (c.rx_size < 6) return;
  const uint16_t length = read_be16(c.rx.data()+4);
  if (read_be16(c.rx.data()+2) != 0 || length < 2 || length > 254) { c.close(); return; }
  const size_t adu_size = size_t(6) + length;
  if (c.rx_size < adu_size) return;
  const auto r = make_registers(snapshot_, stale_ms_);
  c.tx_size = modbus_reply(c.rx.data(), adu_size, tcp_unit_, r, c.tx.data(), c.tx.size());
  if (!c.tx_size) { c.close(); return; }
  c.tx_offset = 0; c.rx_size -= adu_size;
  std::memmove(c.rx.data(), c.rx.data()+adu_size, c.rx_size);
  c.partial_started = now;  // Response is sent on the next loop, without waiting.
}
}}  // namespace esphome::reg10
