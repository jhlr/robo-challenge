#pragma once
#include <Arduino.h>
// Minimal Feetech STS3215 half-duplex bus driver (little-endian register map).
namespace sts {

enum class Mode : uint8_t { Servo = 0, Wheel = 1, Pwm = 2 };

class Bus {
 public:
  void begin(HardwareSerial& port, int rx, int tx, bool echoes, uint32_t baud = 1000000);
  bool ping(uint8_t id);
  bool setMode(uint8_t id, Mode mode);
  bool setTorque(uint8_t id, bool on);
  bool moveTo(uint8_t id, uint16_t pos, uint16_t speed = 1000, uint8_t acc = 50);
  bool setWheelSpeed(uint8_t id, int16_t speed);
  // One broadcast packet, both wheels update together, no ack wait.
  void setWheelSpeeds(uint8_t idA, int16_t a, uint8_t idB, int16_t b);
  bool readPosition(uint8_t id, uint16_t& pos);

 private:
  bool write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t n);
  bool read(uint8_t id, uint8_t addr, uint8_t* out, uint8_t n);
  void send(uint8_t id, uint8_t inst, const uint8_t* params, uint8_t n);
  bool receive(uint8_t id, uint8_t* params, uint8_t n);
  HardwareSerial* port_ = nullptr;
  bool echoes_ = false;
};

}  // namespace sts
