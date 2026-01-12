// © 2025 Hannes Göök
// Licensed under the MIT License

#include <Servo.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

Servo esc1;  // D3 - Back left
Servo esc2;  // D5 - Front left
Servo esc3;  // D9 - Back right
Servo esc4;  // D10 - Front right

#define MPU6050_ADDR 0x68

#if defined(__AVR_ATmega32U4__)
#define RFM95_CS 8
#define RFM95_INT 7
#define RFM95_RST 4
#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_RFM)
#define RFM95_CS 16
#define RFM95_INT 21
#define RFM95_RST 17
#elif defined(__AVR_ATmega328P__)
#define RFM95_CS 4
#define RFM95_INT 2
#define RFM95_RST 6
#elif defined(ESP8266)
#define RFM95_CS 2
#define RFM95_INT 15
#define RFM95_RST 16
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_NRF52840_FEATHER) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
#define RFM95_CS 10
#define RFM95_INT 9
#define RFM95_RST 11
#elif defined(ESP32)
#define RFM95_CS 33
#define RFM95_INT 27
#define RFM95_RST 13
#elif defined(ARDUINO_NRF52832_FEATHER)
#define RFM95_CS 11
#define RFM95_INT 31
#define RFM95_RST 7
#endif

#define RF95_FREQ 433.0

class MyRF95 : public RH_RF95 {
public:
  MyRF95(uint8_t slaveSelectPin, uint8_t interruptPin = RH_INVALID_PIN, RHGenericSPI& spi = hardware_spi)
    : RH_RF95(slaveSelectPin, interruptPin, spi) {}

  void poll() {
    handleInterrupt();
  }
};

MyRF95 rf95(RFM95_CS, RH_INVALID_PIN);
Madgwick filter;

float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float yawDeg = 0.0f;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

float lastGx_dps = 0.0f;
float lastGy_dps = 0.0f;
float lastGz_dps = 0.0f;  // for yaw

float targetPitch = 0.0;
float targetRoll = 0.0;
float targetYaw = 0.0;

int baseThrottleUs = 1000;
unsigned long lastLoRaMs = 0;
float lastDt = 0.004f;

float constrainf(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

int maxThrottle = 2000;

int clampUs(int us) {
  if (us < 1000) return 1000;
  if (us > maxThrottle) return maxThrottle;
  return us;
}

int throttleFromY(float y) {
  if (y < 0) y = 0;
  if (y > 1) y = 1;

  if (y < 0.05f) return 1000;

  int us = 1000 + (int)(y * (maxThrottle - 1000));
  if (us > maxThrottle) us = maxThrottle;
  return us;
}

typedef struct AxisPID {
  float kp;
  float ki;
  float kd;
  float iTerm;
  float outMin;
  float outMax;
} AxisPID_t;

float pidCompute(AxisPID_t &pid, float angle, float targetAngle, float gyroDps, float dt);

float pidCompute(AxisPID_t &pid, float angle, float targetAngle, float gyroDps, float dt) {
  float error = targetAngle - angle;

  pid.iTerm += pid.ki * error * dt;
  if (pid.iTerm > pid.outMax) pid.iTerm = pid.outMax;
  if (pid.iTerm < pid.outMin) pid.iTerm = pid.outMin;

  float out = pid.kp * error - pid.kd * gyroDps + pid.iTerm;
  if (out > pid.outMax) out = pid.outMax;
  if (out < pid.outMin) out = pid.outMin;

  return out;
}

AxisPID_t rollPID = {
  0.0f, 0.0f, 0.0f, 0.0f,
  -50.0f, 50.0f
};

AxisPID_t pitchPID = {
  0.0f, 0.0f, 0.0f, 0.0f,
  -50.0f, 50.0f
};

AxisPID_t yawPID = {
  10.0f, 0.0f, 2.0f, 0.0f,
  -100.0f, 100.0f
};

inline float fastAbs(float v) {
  return v < 0.0f ? -v : v;
}

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void mpuInit() {
  // Wake up
  mpuWrite(0x6B, 0x00);  // PWR_MGMT_1: clear sleep
  delay(100);

  // Set digital low-pass filter: DLPF_CFG = 3
  // Gyro: ~42 Hz, Accel: ~44 Hz, internal sample rate 1 kHz
  mpuWrite(0x1A, 0x03);  // CONFIG

  // Sample rate: 1 kHz / (1 + 3) = 250 Hz
  mpuWrite(0x19, 0x03);  // SMPLRT_DIV

  // Gyro ±250 dps (already what you had)
  mpuWrite(0x1B, 0x00);  // GYRO_CONFIG

  // Accel ±2 g (already what you had)
  mpuWrite(0x1C, 0x00);  // ACCEL_CONFIG

  delay(100);
}

void mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read();

  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

void mpuCalibrateGyro() {
  const int samples = 2000;
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpuReadRaw(ax, ay, az, gx, gy, gz);

    sumX += gx;
    sumY += gy;
    sumZ += gz;

    delay(1);
  }

  gyroBiasX = (float)sumX / samples;
  gyroBiasY = (float)sumY / samples;
  gyroBiasZ = (float)sumZ / samples;
}

void updateIMU() {
  int16_t axr, ayr, azr, gxr, gyr, gzr;
  mpuReadRaw(axr, ayr, azr, gxr, gyr, gzr);

  float ax = axr / 16384.0f;
  float ay = ayr / 16384.0f;
  float az = azr / 16384.0f;

  float gx_dps = (gxr - gyroBiasX) / 131.0f;
  float gy_dps = (gyr - gyroBiasY) / 131.0f;
  float gz_dps = (gzr - gyroBiasZ) / 131.0f;

  // Simple first-order low-pass filters
  static bool first = true;
  static float ax_f, ay_f, az_f;
  static float gx_f, gy_f, gz_f;

  if (first) {
    ax_f = ax;
    ay_f = ay;
    az_f = az;
    gx_f = gx_dps;
    gy_f = gy_dps;
    gz_f = gz_dps;
    first = false;
  }

  const float alphaAcc = 0.15f;   // accel LPF factor (lower = smoother)
  const float alphaGyro = 0.30f;  // gyro LPF factor

  ax_f += alphaAcc * (ax - ax_f);
  ay_f += alphaAcc * (ay - ay_f);
  az_f += alphaAcc * (az - az_f);

  gx_f += alphaGyro * (gx_dps - gx_f);
  gy_f += alphaGyro * (gy_dps - gy_f);
  gz_f += alphaGyro * (gz_dps - gz_f);

  lastGx_dps = gx_f;
  lastGy_dps = gy_f;
  lastGz_dps = gz_f;

  filter.updateIMU(gx_f, gy_f, gz_f, ax_f, ay_f, az_f);

  rollDeg = filter.getRoll();
  pitchDeg = -filter.getPitch();
  yawDeg = -filter.getYaw();
}

void writeMotors(int fl, int fr, int bl, int br) {
  int cFl = clampUs(fl);
  int cFr = clampUs(fr);
  int cBl = clampUs(bl);
  int cBr = clampUs(br);

  esc2.writeMicroseconds(cFl);
  esc4.writeMicroseconds(cFr);
  esc1.writeMicroseconds(cBl);
  esc3.writeMicroseconds(cBr);
}

float x = 0.0;
float y = 0.0;
float t = 0.0;

void updateStabilization() {
  unsigned long nowMs = millis();
  bool failsafe = false;

  if (nowMs - lastLoRaMs > 1000) failsafe = true;

  if (failsafe) {
    writeMotors(1000, 1000, 1000, 1000);
    return;
  }

  int base = baseThrottleUs;
  clampUs(1000);

  float rollTerm = pidCompute(rollPID, rollDeg, targetRoll - (x * 15), lastGx_dps, lastDt);
  float pitchTerm = -pidCompute(pitchPID, pitchDeg, targetPitch + (y * 15), lastGy_dps, lastDt);
  float yawTerm = -pidCompute(yawPID, yawDeg, targetYaw, lastGz_dps, lastDt);

  int fl = base + pitchTerm - rollTerm + yawTerm;
  int fr = base + pitchTerm + rollTerm - yawTerm;
  int bl = base - pitchTerm - rollTerm - yawTerm;
  int br = base - pitchTerm + rollTerm + yawTerm;

  writeMotors(fl, fr, bl, br);
}
uint8_t buf[96];//RH_RF95_MAX_MESSAGE_LEN];
static StaticJsonDocument<JSON_OBJECT_SIZE(9) + 96> loraDoc;

void handleLoRa() {
  rf95.poll();
  uint8_t len = sizeof(buf);

  if (!rf95.available()) return;
  if (!rf95.recv(buf, &len)) return;

  loraDoc.clear();
  DeserializationError err = deserializeJson(loraDoc, buf, len);

  if (err) {
    Serial.print("JSON err: ");
    Serial.println(err.c_str());
    return;
  }

  t = loraDoc["t"] | 0.0f;
  x = loraDoc["x"] | 0.0f;
  y = loraDoc["y"] | 0.0f;

  float p  = loraDoc["p"]  | 0.0f;
  float i  = loraDoc["i"]  | 0.0f;
  float d  = loraDoc["d"]  | 0.0f;

  float p2 = loraDoc["p2"] | 0.0f;
  float i2 = loraDoc["i2"] | 0.0f;
  float d2 = loraDoc["d2"] | 0.0f;

  p *= 5.0f;
  i *= 10.0f;

  p2 *= 10;
  i2 *= 10;
  d2 *= 10;

  rollPID.kp = p;
  rollPID.ki = i;
  rollPID.kd = d;

  pitchPID.kp = p;
  pitchPID.ki = i;
  pitchPID.kd = d;

  yawPID.kp = p2;
  yawPID.ki = i2;
  yawPID.kd = d2;

  baseThrottleUs = throttleFromY(t);
  lastLoRaMs = millis();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  Serial.println("Hej");
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(400000);

  mpuInit();
  delay(200);

  Serial.println("Calibrating gyro, keep quad still...");
  mpuCalibrateGyro();
  Serial.println("Gyro calibration done");

  filter.begin(250.0f);

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("RFM95 init failed");
  } else {
    if (!rf95.setFrequency(RF95_FREQ)) {
      Serial.println("setFrequency failed");
    }
    rf95.setTxPower(23, false);
    rf95.setModeRx();
    Serial.println("RFM95 OK");
  }

  esc1.attach(3);
  esc2.attach(5);
  esc3.attach(9);
  esc4.attach(10);

  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);

  delay(2000);

  lastLoRaMs = millis();

  for (int i = 0; i < 500; i++) {
    updateIMU();
    delay(4);
  }

  targetPitch = pitchDeg;
  targetRoll = rollDeg;
  targetYaw = yawDeg;
}

void loop() {
  long microsLive = micros();

  static unsigned long lastImuMicros = microsLive;
  static unsigned long lastLoRaCheckMs = 0;

  unsigned long nowMicros = microsLive;
  float dt = (nowMicros - lastImuMicros) / 1000000.0f;

  if (dt >= 0.004f) {
    lastImuMicros = nowMicros;
    lastDt = dt;
    updateIMU();
    updateStabilization();
  }

  unsigned long nowMs = millis();
  if (nowMs - lastLoRaCheckMs > 30) {
    lastLoRaCheckMs = nowMs;
    handleLoRa();
  }
}
