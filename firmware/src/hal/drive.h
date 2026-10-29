#pragma once
#include <Arduino.h>

#include <SCServo.h>
// One drive interface for the two cars being compared: STS3215 on the gear-train laterals (encoders) and yellow TT DC motors through an H-bridge (no encoders).
namespace hal {

// Official SCServo (Waveshare adapter does the half-duplex direction): UART2 on pins::kStsRx/kStsTx at 1 Mbaud.
void beginStsBus(SMS_STS& st);

class Drive {
 public:
  virtual ~Drive() = default;
  virtual void begin() = 0;
  virtual void setSpeeds(float left, float right) = 0;  // -1..1, forward positive
  virtual bool hasEncoders() const = 0;
  virtual void resetTravel() {}
  virtual float forwardMm() { return 0.0f; }  // mean wheel travel since resetTravel() (needs encoders)
  virtual float spinDeg() { return 0.0f; }    // + = turned right, since resetTravel() (needs encoders)
};

// STS3215 in wheel mode, one per lateral; the position counter (12 bit, unwrapped here) gives odometry through params::kGearRatio.
class StsDrive : public Drive {
 public:
  explicit StsDrive(SMS_STS& st) : st_(st) {}
  void begin() override;
  void setSpeeds(float left, float right) override;
  bool hasEncoders() const override { return true; }
  void resetTravel() override;
  float forwardMm() override;
  float spinDeg() override;

 private:
  void poll();
  SMS_STS& st_;
  uint16_t lastL_ = 0, lastR_ = 0;
  float mmL_ = 0.0f, mmR_ = 0.0f;  // signed wheel travel, forward positive
};

// 4 PWM pins to a 2-input H-bridge (TB6612FNG with PWMA/PWMB tied high, DRV8833, L9110S): forward = PWM on A, reverse = PWM on B, both low = coast. STBY on pins::kMotorStby.
// The two motors of each side are wired in parallel on one channel.
class DcDrive : public Drive {
 public:
  void begin() override;
  void setSpeeds(float left, float right) override;
  bool hasEncoders() const override { return false; }

 private:
  static void drive(int pinA, int pinB, float v);
};

}  // namespace hal
