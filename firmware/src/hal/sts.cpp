#include "sts.h"

namespace sts {
namespace {
constexpr uint8_t kInstPing = 0x01, kInstRead = 0x02, kInstWrite = 0x03, kInstSyncWrite = 0x83;
constexpr uint8_t kBroadcast = 0xFE;
constexpr uint8_t kRegId = 5, kRegLock = 55, kRegMode = 33, kRegTorque = 40, kRegAcc = 41, kRegGoalPos = 42, kRegGoalSpeed = 46,
                  kRegPresentPos = 56;
constexpr uint32_t kTimeoutUs = 4000;
constexpr int16_t kMaxSpeed = 3400;

uint16_t encodeSpeed(int16_t s) {
  if (s > kMaxSpeed) s = kMaxSpeed;
  if (s < -kMaxSpeed) s = -kMaxSpeed;
  return s < 0 ? static_cast<uint16_t>(-s) | 0x8000 : static_cast<uint16_t>(s);
}
}  // namespace

void Bus::begin(HardwareSerial& port, int rx, int tx, bool echoes, uint32_t baud) {
  port_ = &port;
  echoes_ = echoes;
  port_->begin(baud, SERIAL_8N1, rx, tx);
}

void Bus::send(uint8_t id, uint8_t inst, const uint8_t* params, uint8_t n) {
  uint8_t pkt[40];
  const uint8_t len = n + 2;
  pkt[0] = pkt[1] = 0xFF;
  pkt[2] = id;
  pkt[3] = len;
  pkt[4] = inst;
  uint8_t sum = id + len + inst;
  for (uint8_t i = 0; i < n; ++i) {
    pkt[5 + i] = params[i];
    sum += params[i];
  }
  pkt[5 + n] = ~sum;
  while (port_->available()) port_->read();  // drop stale bytes
  port_->write(pkt, n + 6);
  port_->flush();
  if (echoes_) {  // single-wire hardware: our own bytes come back, discard them
    uint8_t seen = 0;
    const uint32_t t0 = micros();
    while (seen < n + 6 && micros() - t0 < kTimeoutUs) {
      if (port_->available()) { port_->read(); ++seen; }
    }
  }
}

bool Bus::receive(uint8_t id, uint8_t* params, uint8_t n) {
  uint8_t buf[24];
  const uint8_t want = n + 6;  // FF FF id len err params... chk
  uint8_t got = 0;
  const uint32_t t0 = micros();
  while (got < want && micros() - t0 < kTimeoutUs) {
    if (port_->available()) buf[got++] = port_->read();
  }
  if (got < want || buf[0] != 0xFF || buf[1] != 0xFF || buf[2] != id || buf[3] != n + 2) return false;
  uint8_t sum = 0;
  for (uint8_t i = 2; i < want - 1; ++i) sum += buf[i];
  if (static_cast<uint8_t>(~sum) != buf[want - 1] || buf[4] != 0) return false;
  for (uint8_t i = 0; i < n; ++i) params[i] = buf[5 + i];
  return true;
}

bool Bus::write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t n) {
  uint8_t p[16];
  p[0] = addr;
  for (uint8_t i = 0; i < n; ++i) p[1 + i] = data[i];
  send(id, kInstWrite, p, n + 1);
  return id == kBroadcast ? true : receive(id, nullptr, 0);
}

bool Bus::read(uint8_t id, uint8_t addr, uint8_t* out, uint8_t n) {
  const uint8_t p[2] = {addr, n};
  send(id, kInstRead, p, 2);
  return receive(id, out, n);
}

bool Bus::ping(uint8_t id) {
  send(id, kInstPing, nullptr, 0);
  return receive(id, nullptr, 0);
}

bool Bus::setMode(uint8_t id, Mode mode) {
  const uint8_t v = static_cast<uint8_t>(mode);
  return write(id, kRegMode, &v, 1);
}

bool Bus::setId(uint8_t id, uint8_t newId) {
  if (newId == kBroadcast || newId == id) return false;
  const uint8_t unlock = 0, lock = 1;
  if (!write(id, kRegLock, &unlock, 1) || !write(id, kRegId, &newId, 1)) return false;
  return write(newId, kRegLock, &lock, 1);
}

bool Bus::setTorque(uint8_t id, bool on) {
  const uint8_t v = on ? 1 : 0;
  return write(id, kRegTorque, &v, 1);
}

bool Bus::moveTo(uint8_t id, uint16_t pos, uint16_t speed, uint8_t acc) {
  // acc(41), goal pos(42-43), goal time(44-45)=0, goal speed(46-47)
  const uint8_t d[7] = {acc, static_cast<uint8_t>(pos), static_cast<uint8_t>(pos >> 8), 0, 0,
                        static_cast<uint8_t>(speed), static_cast<uint8_t>(speed >> 8)};
  return write(id, kRegAcc, d, sizeof(d));
}

bool Bus::setWheelSpeed(uint8_t id, int16_t speed) {
  const uint16_t e = encodeSpeed(speed);
  const uint8_t d[2] = {static_cast<uint8_t>(e), static_cast<uint8_t>(e >> 8)};
  return write(id, kRegGoalSpeed, d, 2);
}

void Bus::setWheelSpeeds(uint8_t idA, int16_t a, uint8_t idB, int16_t b) {
  const uint16_t ea = encodeSpeed(a), eb = encodeSpeed(b);
  const uint8_t p[8] = {kRegGoalSpeed, 2,
                        idA, static_cast<uint8_t>(ea), static_cast<uint8_t>(ea >> 8),
                        idB, static_cast<uint8_t>(eb), static_cast<uint8_t>(eb >> 8)};
  send(kBroadcast, kInstSyncWrite, p, sizeof(p));
}

bool Bus::readPosition(uint8_t id, uint16_t& pos) {
  uint8_t d[2];
  if (!read(id, kRegPresentPos, d, 2)) return false;
  pos = d[0] | (d[1] << 8);
  return true;
}

}  // namespace sts
