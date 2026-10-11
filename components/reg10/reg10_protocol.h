#pragma once
// Protocol helpers independent of ESPHome; tested separately on the host.
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace esphome { namespace reg10 {
inline uint16_t read_be16(const uint8_t *p) {
  return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}
inline int16_t read_be16_signed(const uint8_t *p) {
  const uint16_t n = read_be16(p);
  return static_cast<int16_t>(n < 0x8000 ? int32_t(n) : int32_t(n) - 65536);
}
inline void write_be16(uint8_t *p, uint16_t n) {
  p[0] = uint8_t(n >> 8); p[1] = uint8_t(n);
}
// CRC-16/IBM-SDLC (X-25), inferred from the supplied examples and checked
// against their CRC bytes: init FFFF, reflected polynomial 8408, xorout FFFF.
// REG10 transmits the result HIGH byte first, before byte stuffing.
inline uint16_t crc_reg10(const uint8_t *p, size_t size) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < size; ++i) {
    crc ^= p[i];
    for (uint8_t j = 0; j < 8; ++j)
      crc = (crc >> 1) ^ ((crc & 1) ? 0x8408 : 0);
  }
  return crc ^ 0xFFFF;
}
inline std::vector<uint8_t> encode_frame(const uint8_t *body, size_t size) {
  std::vector<uint8_t> out;
  out.reserve(2 * size + 2);
  out.push_back(0xC0);
  for (size_t i = 0; i < size; ++i) {
    const uint8_t b = body[i];
    if (b == 0xC0 || b == 0xC1 || b == 0x7D) {
      out.push_back(0x7D); out.push_back(b ^ 0x20);
    } else out.push_back(b);
  }
  out.push_back(0xC1);
  return out;
}
struct ReadRequest { uint8_t control; uint16_t address; uint8_t count; };
constexpr std::array<ReadRequest, 3> READ_REQUESTS{{
  {0x11, 0x00C8, 8},  // Internal RAM, four temperature words.
  {0x11, 0x002F, 1},  // Internal RAM, output byte.
  {0x31, 0x0048, 4},  // EXTERNAL RAM, corrected by the supplier.
}};
inline std::vector<uint8_t> make_read_request(uint8_t slave, uint8_t master,
                                            const ReadRequest &r) {
  uint8_t body[10] = {slave, master, r.control, 0, 0, uint8_t(r.address >> 8),
                     uint8_t(r.address), r.count, 0, 0};
  write_be16(body + 8, crc_reg10(body, 8));
  return encode_frame(body, sizeof(body));
}
class FrameDecoder {
 public:
  static constexpr size_t MAX_BODY = 138;  // 8-byte header + 128 data + 2 CRC.
  void reset() { active_ = false; escaped_ = false; size_ = 0; }
  bool active() const { return active_; }
  size_t size() const { return size_; }
  const uint8_t *data() const { return body_.data(); }
  bool feed(uint8_t b) {
    if (b == 0xC0) { active_ = true; escaped_ = false; size_ = 0; return false; }
    if (!active_) return false;
    if (b == 0xC1) {
      const bool complete = !escaped_ && size_ >= 10;
      active_ = false; escaped_ = false;
      return complete;
    }
    if (escaped_) {
      b ^= 0x20; escaped_ = false;
      if (b != 0xC0 && b != 0xC1 && b != 0x7D) { reset(); return false; }
    } else if (b == 0x7D) { escaped_ = true; return false; }
    if (size_ == MAX_BODY) { reset(); return false; }
    body_[size_++] = b;
    return false;
  }
 private:
  std::array<uint8_t, MAX_BODY> body_{};
  size_t size_{0};
  bool active_{false}, escaped_{false};
};
enum class ResponseResult { OK, NOT_FOR_REQUEST, BAD_LENGTH, BAD_CRC, JOB_ERROR };
inline ResponseResult validate_response(const uint8_t *b, size_t size,
                                       uint8_t slave, uint8_t master,
                                       const ReadRequest &r) {
  if (size < 10) return ResponseResult::BAD_LENGTH;
  // Also rejects local TX echo because its source/destination are swapped.
  if (b[0] != master || b[1] != slave || b[2] != r.control || b[4] != 0 ||
      read_be16(b + 5) != r.address) return ResponseResult::NOT_FOR_REQUEST;
  if (read_be16(b + size - 2) != crc_reg10(b, size - 2)) return ResponseResult::BAD_CRC;
  if (b[3] != 0) return ResponseResult::JOB_ERROR;
  if (b[7] != r.count || size != size_t(10 + r.count)) return ResponseResult::BAD_LENGTH;
  return ResponseResult::OK;
}
inline bool temperature_valid(int16_t raw) { return raw > -20000 && raw < 20000; }
struct Snapshot {
  std::array<int16_t, 4> temperatures{};
  std::array<uint16_t, 2> flows{};
  uint8_t outputs{0};
  std::array<uint32_t, 3> age_ms{{UINT32_MAX, UINT32_MAX, UINT32_MAX}};
  uint32_t good_frames{0}, crc_errors{0}, timeouts{0};
  void advance(uint32_t elapsed) {
    for (auto &age : age_ms) age = age > UINT32_MAX - elapsed ? UINT32_MAX : age + elapsed;
  }
  uint16_t valid_mask(uint32_t stale_ms) const {
    uint16_t mask = 0;
    for (size_t i = 0; i < 3; ++i) if (age_ms[i] <= stale_ms) mask |= 1U << i;
    return mask;
  }
};
constexpr size_t REGISTER_COUNT = 24;
constexpr std::array<uint8_t, 5> OUTPUT_BITS{{0, 1, 2, 3, 5}};
inline std::array<uint16_t, REGISTER_COUNT> make_registers(const Snapshot &s,
                                                         uint32_t stale_ms) {
  std::array<uint16_t, REGISTER_COUNT> r{};
  const uint16_t mask = s.valid_mask(stale_ms);
  for (size_t i = 0; i < 4; ++i) {
    r[i] = (mask & 1) ? static_cast<uint16_t>(s.temperatures[i]) : 0x8000;
    if ((mask & 1) && temperature_valid(s.temperatures[i])) r[8] |= 1U << i;
  }
  r[4] = (mask & 4) ? s.flows[0] : 0xFFFF;
  r[5] = (mask & 4) ? s.flows[1] : 0xFFFF;
  r[6] = (mask & 2) ? s.outputs : 0xFFFF;
  r[7] = mask;
  for (size_t i = 0; i < 3; ++i)
    r[9+i] = s.age_ms[i] / 1000 >= 65535 ? 65535 : uint16_t(s.age_ms[i] / 1000);
  r[12] = uint16_t(s.good_frames >> 16); r[13] = uint16_t(s.good_frames);
  r[14] = uint16_t(s.crc_errors >> 16); r[15] = uint16_t(s.crc_errors);
  r[16] = uint16_t(s.timeouts >> 16); r[17] = uint16_t(s.timeouts);
  for (size_t i = 0; i < 5; ++i)
    r[18+i] = (mask & 2) ? ((s.outputs >> OUTPUT_BITS[i]) & 1) : 0xFFFF;
  r[23] = 1;  // Version of this bridge's map, not REG10 firmware.
  return r;
}
// Build one Modbus TCP ADU; 0 means malformed framing. All operations are reads.
inline size_t modbus_reply(const uint8_t *q, size_t size, uint8_t unit,
                          const std::array<uint16_t, REGISTER_COUNT> &r,
                          uint8_t *out, size_t capacity) {
  if (size < 8 || size > 260 || capacity < 9 || read_be16(q+2) != 0 ||
      read_be16(q+4) != size-6) return 0;
  std::memcpy(out, q, 4);
  out[6] = q[6];
  const uint8_t function = q[7];
  uint8_t exception = 0;
  uint16_t address = 0, count = 0;
  if (q[6] != unit) exception = 0x0B;
  else if (function != 3 && function != 4) exception = 1;
  else if (size != 12) exception = 3;
  else {
    address = read_be16(q+8); count = read_be16(q+10);
    if (count == 0 || count > 125) exception = 3;
    else if (uint32_t(address) + count > r.size()) exception = 2;
  }
  if (exception) {
    write_be16(out+4, 3); out[7] = function | 0x80; out[8] = exception;
    return 9;
  }
  const size_t out_size = 9 + 2 * count;
  if (capacity < out_size) return 0;
  write_be16(out+4, uint16_t(out_size-6)); out[7] = function; out[8] = uint8_t(2*count);
  for (size_t i = 0; i < count; ++i) write_be16(out+9+2*i, r[address+i]);
  return out_size;
}
}}  // namespace esphome::reg10
