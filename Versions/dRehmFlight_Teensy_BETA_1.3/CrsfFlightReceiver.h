#pragma once
#include "TricopterConfig.h"

namespace tricopter {

class CrsfFlightReceiver {
 public:
  enum class Event { None, Channels, LinkStatistics };
  static constexpr unsigned channelCount = 8;
  static constexpr uint32_t statisticsMaxAgeUs = 3000000;

  Event consume(uint8_t byte, uint32_t now) {
    if (used_ && now - lastByteUs_ > 3000) used_ = 0;
    lastByteUs_ = now;
    if (used_ == 0) {
      startFrame(byte);
      return Event::None;
    }
    if (used_ == 1) {
      if (byte < 2 || byte > 62) {
        startFrame(byte);
        return Event::None;
      }
      length_ = byte;
      crc_ = 0;
      ++used_;
      return Event::None;
    }
    if (used_ < length_ + 1) {
      body_[used_ - 2] = byte;
      ++used_;
      crc_ ^= byte;
      for (unsigned bit = 0; bit < 8; ++bit)
        crc_ = (crc_ & 0x80) ? uint8_t((crc_ << 1) ^ 0xd5) : uint8_t(crc_ << 1);
      return Event::None;
    }
    used_ = 0;
    if (byte != crc_ || address_ != 0xc8) return Event::None;
    if (body_[0] == 0x14 && length_ >= 12) {
      uplinkQuality = body_[3];
      uplinkRssi = body_[5] ? body_[2] : body_[1];
      haveStatistics_ = true;
      lastStatisticsUs_ = now;
      return Event::LinkStatistics;
    }
    if (body_[0] != 0x16 || length_ < 24) return Event::None;
    for (unsigned channel = 0; channel < channelCount; ++channel) {
      const unsigned bit = 11 * channel;
      const uint8_t *data = body_ + 1 + bit / 8;
      const uint32_t packed = data[0] | (uint32_t(data[1]) << 8) | (uint32_t(data[2]) << 16);
      const unsigned raw = (packed >> (bit % 8)) & 0x7ff;
      const unsigned pulse = unsigned(raw * 0.62477120195241f + 881);
      channelsUs[channel] = pulse < 1000 ? 1000 : (pulse > 2000 ? 2000 : pulse);
    }
    hasChannels_ = true;
    lastChannelsUs_ = now;
    return Event::Channels;
  }

  bool fresh(uint32_t now) {
    if (now - lastChannelsUs_ > 300000) hasChannels_ = false;
    return hasChannels_;
  }
  bool statisticsFresh(uint32_t now) {
    if (now - lastStatisticsUs_ > statisticsMaxAgeUs) haveStatistics_ = false;
    return haveStatistics_;
  }

  // Last CRC-validated input. Keep CH7/8 neutral until the first channel frame.
  uint16_t channelsUs[channelCount] = {0, 0, 0, 0, 0, 0, 1500, 1500};
  uint8_t uplinkQuality = 0, uplinkRssi = 255;

 private:
  void startFrame(uint8_t byte) {
    address_ = byte;
    switch (byte) {
      case 0x00: case 0x10: case 0x80: case 0x8a: case 0xc0: case 0xc2: case 0xc4:
      case 0xc8: case 0xca: case 0xcc: case 0xea: case 0xec: case 0xee:
        used_ = 1; break;
      default: used_ = 0; break;
    }
  }
  uint8_t body_[61] = {}, length_ = 0, used_ = 0, crc_ = 0, address_ = 0;
  uint32_t lastByteUs_ = 0, lastChannelsUs_ = 0, lastStatisticsUs_ = 0;
  bool hasChannels_ = false, haveStatistics_ = false;
};

class CrsfArmSwitch {
 public:
  static constexpr unsigned armMaxUs = 1300, cutMinUs = 1700, idleMaxUs = 1040;
  void reset() { state_ = State::NeedCut; count_ = 0; }
  bool armed() const { return state_ == State::Armed; }
  void expirePreparation() { if (!armed()) reset(); }

  void onChannels(unsigned throttle, unsigned sa, unsigned spin, uint32_t now, bool statisticsFresh) {
    if (!statisticsFresh && (!armed() || sa > armMaxUs)) { reset(); return; }
    const bool idle = throttle <= idleMaxUs && spinCentered(spin);
    if (sa >= cutMinUs) {
      reset();
      if (idle) state_ = State::Prepared;
      return;
    }
    if (sa > armMaxUs) {
      if (state_ != State::Prepared || !idle) reset();
      return;
    }
    if (armed() || state_ == State::NeedCut) return;
    if (!idle || (count_ && now - previousUs_ > 100000)) { reset(); return; }
    if (!count_) firstUs_ = now;
    previousUs_ = now;
    if (count_ < 3) ++count_;
    state_ = count_ >= 3 && now - firstUs_ >= 150000 ? State::Armed : State::Confirming;
  }

 private:
  enum class State { NeedCut, Prepared, Confirming, Armed };
  State state_ = State::NeedCut;
  uint8_t count_ = 0;
  uint32_t firstUs_ = 0, previousUs_ = 0;
};
} // namespace tricopter
