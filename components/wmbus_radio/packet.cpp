#include "packet.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "decode3of6.h"

#include "wmbus.h"

#define WMBUS_PREAMBLE_SIZE (3)
#define WMBUS_MODE_C_PREAMBLE (0x54)
#define WMBUS_BLOCK_A_PREAMBLE (0xCD)
#define WMBUS_BLOCK_B_PREAMBLE (0x3D)

namespace esphome {
namespace wmbus_radio {
static const char *TAG = "wmbus";

Packet::Packet() { this->data_.reserve(300); }

// Determine the link mode based on the first byte of the data
LinkMode Packet::link_mode() {
  if (this->link_mode_ == LinkMode::UNKNOWN && this->data_.size())
    if (this->data_[0] == WMBUS_MODE_C_PREAMBLE)
      this->link_mode_ = LinkMode::C1;
    else
      this->link_mode_ = LinkMode::T1;

  return this->link_mode_;
}

void Packet::set_rssi(int8_t rssi) { this->rssi_ = rssi; }

// Get value of L-field
uint8_t Packet::l_field() {
  switch (this->link_mode()) {
  case LinkMode::C1:
    return this->data_[2];
  case LinkMode::T1: {
    auto decoded = decode3of6(this->data_);
    if (decoded)
      return (*decoded)[0];
  }
  }
  return 0;
}

size_t Packet::expected_size() {
  if (!this->expected_size_) {
    // Check if we have enough data to determine the expected size
    if (this->data_.size() < WMBUS_PREAMBLE_SIZE)
      return 0;

    auto l_field = this->l_field();

    // If L-field is 0, we cannot determine the expected size yet
    if (!l_field)
      return 0;

    // The first block has 10 bytes + 2 bytes for CRC
    // Each subsequent block has 16 bytes + 2 bytes for CRC
    // The last block may have fewer bytes, so we calculate the remainder
    // See 10.4.1 / Table 14 of EN 13757-4
    auto total_length = l_field + 1;
    auto blocks = total_length / 16 + 1;
    auto total_length_with_crc = total_length + blocks * 2;

    switch (this->link_mode()) {
    case LinkMode::C1:
      if (this->data_[1] == WMBUS_BLOCK_A_PREAMBLE) {
        // Two bytes for Mode C1 preamble + total length with CRC
        this->expected_size_ = 2 + total_length_with_crc;
        this->frame_format_ = "A";
      } else if (this->data_[1] == WMBUS_BLOCK_B_PREAMBLE) {
        // Two bytes for Mode C1 preamble + 1 byte for L-field + L-field bytes
        // (which includes CRC for B format)
        this->expected_size_ = 2 + 1 + l_field;
        this->frame_format_ = "B";
      }
      break;
    case LinkMode::T1:
      // Mode T1 always uses Block A
      // 3-of-6 encoding takes 3 bytes for every 2 bytes of data
      // Plus 1 byte for remainder if total_length_with_crc is odd
      this->expected_size_ = encoded_size(total_length_with_crc);
      this->frame_format_ = "A";
      break;
    }
  }
  return this->expected_size_;
}

bool Packet::calculate_payload() {
  switch (this->link_mode()) {
  case LinkMode::C1:
    // Remove the two-byte Mode C1 preamble
    this->data_.erase(this->data_.begin(), this->data_.begin() + 2);
    return true;

  case LinkMode::T1: {
    auto decoded = decode3of6(this->data_);
    if (decoded) {
      this->data_ = *decoded;
      return true;
    }
  }
  }
  return false;
}

std::optional<Frame> Packet::convert_to_frame() {
  std::optional<Frame> frame = {};

  ESP_LOGD(TAG, "Have data from radio (%zu bytes)", this->data_.size());
  debugPayload("raw packet", this->data_);

  if (this->expected_size() == this->data_.size()) {
    if (this->calculate_payload()) {
      frame.emplace(this);
    }
  } else {
    ESP_LOGD(TAG, "Invalid packet size: expected %zu, got %zu",
             this->expected_size(), this->data_.size());
  }

  delete this;

  return frame;
}

Frame::Frame(Packet *packet)
    : data_(std::move(packet->data_)), link_mode_(packet->link_mode_),
      rssi_(packet->rssi_), format_(packet->frame_format_) {}

std::vector<uint8_t> &Frame::data() { return this->data_; }
LinkMode Frame::link_mode() { return this->link_mode_; }
int8_t Frame::rssi() { return this->rssi_; }
std::string Frame::format() { return this->format_; }

std::vector<uint8_t> Frame::as_raw() { return this->data_; }
std::string Frame::as_hex() {
  static const char hex[] = "0123456789abcdef";
  std::string out;
  out.reserve(this->data_.size() * 2);
  for (uint8_t b : this->data_) {
    out.push_back(hex[b >> 4]);
    out.push_back(hex[b & 0x0f]);
  }
  return out;
}
std::string Frame::as_rtlwmbus() {
  std::string output;
  // Format:
  // {T1|C1};1;1;YYYY-MM-DD HH:MM:SS.000;RSSI;RSSI;0xREDACTED;HEX_DATA
  // Reserve enough space to avoid multiple reallocations
  // 2 + 5 + 24 + 1 + 4 + 5 + 2*data.size() + 1
  output.reserve(2 + 5 + 24 + 1 + 4 + 5 + 2 * this->data_.size() + 1);

  output += linkModeName(this->link_mode_); // size 2
  output += ";1;1;";                         // size 5
  output += "1970-01-01 00:00:00.000;";     // size 24 (timestamp placeholder)
  output += std::to_string(this->rssi_);    // size up to 4
  output += ";111;";                       // size 5 (RSSI placeholder)
  output += this->as_hex();                 // size 2 * frame.size()
  output += "\n";                           // size 1

  return output;
}

void Frame::mark_as_handled() { this->handlers_count_++; }
uint8_t Frame::handlers_count() { return this->handlers_count_; }

} // namespace wmbus_radio
} // namespace esphome
