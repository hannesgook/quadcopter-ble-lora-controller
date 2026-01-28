// © 2025 Hannes Göök
// Licensed under the MIT License

#include <Servo.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

Servo esc1;  // D3 - Back left (spins counter-clockwise)
Servo esc2;  // D5 - Front left (spins clockwise)
Servo esc3;  // D9 - Back right (spins clockwise)
Servo esc4;  // D10 - Front right (spins counter-clockwise)

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

  void poll() { handleInterrupt(); }
};

MyRF95 rf95(RFM95_CS, RH_INVALID_PIN);
Madgwick filter;

float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float yawDeg = 0.0f;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

float gx_dps = 0.0f;
float gy_dps = 0.0f;
float gz_dps = 0.0f;

float stickRoll = 0.0f;
float stickPitch = 0.0f;
float stickYaw = 0.0f;
float stickThrottle = 0.0f;

int maxThrottle = 2000;

unsigned long lastLoRaMs = 0;
float lastDt = 0.004f;

static const float LOOP_HZ = 250.0f;
static const float LOOP_DT = 1.0f / LOOP_HZ;

static const float STICK_MAX_ANGLE_DEG = 15.0f;
static const float STICK_MAX_YAW_RATE_DPS = 120.0f;

// Angle loop gains (outer loop)
static const float ANGLE_KP = 2.0f;
static const float ANGLE_I_DEADZONE = 0.62f;  // Don't accumulate I-term for errors below this constant
float rollAngleITerm = 0.0f;
float pitchAngleITerm = 0.0f;
float angleKI = 0.0f;

static const unsigned long FAILSAFE_CUTOFF_MS = 1000;
static const int IDLE_CUTOFF_US = 1050;

float rollArmRef = 0.0f;
float pitchArmRef = 0.0f;
bool armed = false;

bool imuHealthy = true;
unsigned long lastImuOkMs = 0;

static const float ACC_MIN_G = 0.5f;
static const float ACC_MAX_G = 1.8f;
static const unsigned long IMU_FAIL_TIMEOUT_MS = 100;

float constrainf(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

int clampUs(int us) {
  if (us < 1000) return 1000;
  if (us > maxThrottle) return maxThrottle;
  return us;
}

int throttleFromY(float y) {
  y = constrainf(y, 0.0f, 1.0f);
  if (y < 0.05f) return 1000;
  int us = 1000 + (int)(y * (maxThrottle - 1000));
  if (us > maxThrottle) us = maxThrottle;
  return us;
}

typedef struct PID {
  float kp;
  float ki;
  float kd;
  float iTerm;
  float lastRate;
  float outMin;
  float outMax;
} PID_t;

float pidRate(PID_t &pid, float targetRateDps, float measuredRateDps, float dt);
float pidRate(PID_t &pid, float targetRateDps, float measuredRateDps, float dt) {
  float error = targetRateDps - measuredRateDps;

  if (pid.ki > 0.0001f) {
    pid.iTerm += pid.ki * error * dt;
    if (pid.iTerm > pid.outMax) pid.iTerm = pid.outMax;
    if (pid.iTerm < pid.outMin) pid.iTerm = pid.outMin;
  }

  float dTerm = 0.0f;
  if (pid.kd > 0.0001f && dt > 0.0f) {
    dTerm = -pid.kd * (measuredRateDps - pid.lastRate) / dt;
  }

  pid.lastRate = measuredRateDps;

  float out = pid.kp * error + pid.iTerm + dTerm;
  if (out > pid.outMax) out = pid.outMax;
  if (out < pid.outMin) out = pid.outMin;

  return out;
}

void pidReset(PID_t &pid);
void pidReset(PID_t &pid) {
  pid.iTerm = 0.0f;
  pid.lastRate = 0.0f;
}

// Rate PIDs (inner loop)
PID_t rollRatePID  = { 0.075f, 0.0f, 0.0f, 0.0f, 0.0f, -250.0f, 250.0f };
PID_t pitchRatePID = { 0.075f, 0.0f, 0.0f, 0.0f, 0.0f, -250.0f, 250.0f };
PID_t yawRatePID   = { 10.0f, 0.5f, 0.0f, 0.0f, 0.0f, -250.0f, 250.0f };

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void mpuInit() {
  mpuWrite(0x6B, 0x00);
  delay(100);
  mpuWrite(0x1A, 0x03);
  mpuWrite(0x19, 0x03);
  mpuWrite(0x1B, 0x00);
  mpuWrite(0x1C, 0x00);
  delay(100);
}

bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  int bytes = Wire.requestFrom(MPU6050_ADDR, 14, true);
  if (bytes != 14) return false;

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read();
  Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  return true;
}

void mpuCalibrateGyro() {
  const int samples = 2000;
  long sumX = 0, sumY = 0, sumZ = 0;

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

  if (!mpuReadRaw(axr, ayr, azr, gxr, gyr, gzr)) {
    return;
  }

  float ax = axr / 16384.0f;
  float ay = ayr / 16384.0f;
  float az = azr / 16384.0f;

  float accMag = sqrtf(ax * ax + ay * ay + az * az);

  if (accMag < ACC_MIN_G || accMag > ACC_MAX_G) {
    return;
  }

  float gxRaw = (gxr - gyroBiasX) / 131.0f;
  float gyRaw = (gyr - gyroBiasY) / 131.0f;
  float gzRaw = (gzr - gyroBiasZ) / 131.0f;

  static bool first = true;
  static float gx_f, gy_f, gz_f;

  if (first) {
    gx_f = gxRaw;
    gy_f = gyRaw;
    gz_f = gzRaw;
    first = false;
  }

  const float alphaGyro = 0.30f;
  gx_f += alphaGyro * (gxRaw - gx_f);
  gy_f += alphaGyro * (gyRaw - gy_f);
  gz_f += alphaGyro * (gzRaw - gz_f);

  gx_dps = -gx_f;
  gy_dps = gy_f;
  gz_dps = gz_f;

  filter.updateIMU(gx_f, gy_f, gz_f, ax, ay, az);

  rollDeg  = -filter.getRoll();
  pitchDeg =  filter.getPitch();
  yawDeg   = -filter.getYaw();

  if (!isfinite(rollDeg) || !isfinite(pitchDeg) || !isfinite(yawDeg)) {
    return;
  }

  imuHealthy = true;
  lastImuOkMs = millis();
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

void stopMotorsAndReset() {
  writeMotors(1000, 1000, 1000, 1000);
  pidReset(rollRatePID);
  pidReset(pitchRatePID);
  pidReset(yawRatePID);
  rollAngleITerm = 0.0f;
  pitchAngleITerm = 0.0f;
}

uint8_t buf[96];
static StaticJsonDocument<JSON_OBJECT_SIZE(12) + 96> loraDoc;

void handleLoRa() {
  rf95.poll();
  uint8_t len = sizeof(buf);

  if (!rf95.available()) return;
  if (!rf95.recv(buf, &len)) return;

  loraDoc.clear();
  DeserializationError err = deserializeJson(loraDoc, buf, len);
  if (err) return;

  stickThrottle = loraDoc["t"] | 0.0f;
  stickRoll     = loraDoc["x"] | 0.0f;
  stickPitch    = loraDoc["y"] | 0.0f;
  stickYaw      = loraDoc["r"] | 0.0f;

  stickThrottle = constrainf(stickThrottle, 0.0f, 1.0f);
  stickRoll     = constrainf(stickRoll, -1.0f, 1.0f);
  stickPitch    = constrainf(stickPitch, -1.0f, 1.0f);
  stickYaw      = constrainf(stickYaw, -1.0f, 1.0f);

  float p   = loraDoc["p"]  | 0.05f;
  float aki = loraDoc["i"]  | 0.0f;
  float d   = loraDoc["d"]  | 0.0f;
  
  d/=10;

  float p2 = loraDoc["p2"] | 1.0f;
  float i2 = loraDoc["i2"] | 0.05f;
  float d2 = loraDoc["d2"] | 0.0f;
  p2*=2;
  i2/=10;

  aki *= 2;

  rollRatePID.kp = p;
  rollRatePID.ki = 0.0f;
  rollRatePID.kd = d;

  pitchRatePID.kp = p;
  pitchRatePID.ki = 0.0f;
  pitchRatePID.kd = d;

  yawRatePID.kp = p2;
  yawRatePID.ki = i2;
  yawRatePID.kd = d2;

  angleKI = aki;

  lastLoRaMs = millis();
}

void updateStabilization() {
  if ((millis() - lastImuOkMs) > IMU_FAIL_TIMEOUT_MS) {
    imuHealthy = false;
  }

  if (!imuHealthy) {
    armed = false;
    stopMotorsAndReset();
    return;
  }

  unsigned long nowMs = millis();
  bool failsafe = (nowMs - lastLoRaMs) > FAILSAFE_CUTOFF_MS;

  int throttleUs = throttleFromY(stickThrottle);

  if ((nowMs - lastLoRaMs) > FAILSAFE_CUTOFF_MS) {
    armed = false;
    stopMotorsAndReset();
    return;
  }

  if (!armed) {
    if (throttleUs < IDLE_CUTOFF_US) {
      rollArmRef = rollDeg;
      pitchArmRef = pitchDeg;
      armed = true;
    } else {
      stopMotorsAndReset();
      return;
    }
  }

  float targetRollDeg  = stickRoll  * STICK_MAX_ANGLE_DEG;
  float targetPitchDeg = stickPitch * STICK_MAX_ANGLE_DEG;

  float rollErrDeg  = targetRollDeg  - (rollDeg - rollArmRef);
  float pitchErrDeg = targetPitchDeg - (pitchDeg - pitchArmRef);

  // Angle integral with deadzone
  if (angleKI > 0.0001f && throttleUs >= IDLE_CUTOFF_US) {
    float rollErrForI = (fabs(rollErrDeg) > ANGLE_I_DEADZONE) ? rollErrDeg : 0.0f;
    float pitchErrForI = (fabs(pitchErrDeg) > ANGLE_I_DEADZONE) ? pitchErrDeg : 0.0f;
    
    rollAngleITerm += angleKI * rollErrForI * lastDt;
    pitchAngleITerm += angleKI * pitchErrForI * lastDt;
    
    rollAngleITerm = constrainf(rollAngleITerm, -60.0f, 60.0f);
    pitchAngleITerm = constrainf(pitchAngleITerm, -60.0f, 60.0f);
  } else {
    rollAngleITerm = 0.0f;
    pitchAngleITerm = 0.0f;
  }

  float targetRollRateDps  = ANGLE_KP * rollErrDeg + rollAngleITerm;
  float targetPitchRateDps = ANGLE_KP * pitchErrDeg + pitchAngleITerm;
  float targetYawRateDps   = stickYaw * STICK_MAX_YAW_RATE_DPS;

  float rollOut  = -pidRate(rollRatePID,  targetRollRateDps,  gx_dps, lastDt);
  float pitchOut = pidRate(pitchRatePID, targetPitchRateDps, gy_dps, lastDt);
  float yawOut   = pidRate(yawRatePID,   targetYawRateDps,   gz_dps, lastDt);

  int fl = throttleUs + (int)pitchOut - (int)rollOut + (int)yawOut;
  int fr = throttleUs + (int)pitchOut + (int)rollOut - (int)yawOut;
  int bl = throttleUs - (int)pitchOut - (int)rollOut - (int)yawOut;
  int br = throttleUs - (int)pitchOut + (int)rollOut + (int)yawOut;

  writeMotors(fl, fr, bl, br);

  static unsigned long lastPrintMs = 0;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Wire.setWireTimeout(25000, true);
  Wire.setClock(400000);

  mpuInit();
  delay(200);

  Serial.println("Calibrating gyro, keep quad still...");
  mpuCalibrateGyro();
  Serial.println("Gyro calibration done");

  filter.begin(LOOP_HZ);

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

  stopMotorsAndReset();
  delay(2000);

  lastLoRaMs = millis();

  for (int i = 0; i < 500; i++) {
    updateIMU();
    delay(4);
  }

  stickThrottle = 0.0f;
  stickRoll = 0.0f;
  stickPitch = 0.0f;
  stickYaw = 0.0f;

  Serial.println("Setup finished");
}

void loop() {
  unsigned long microsLive = micros();
  static unsigned long lastImuMicros = microsLive;
  static unsigned long lastLoRaCheckMs = 0;

  unsigned long nowMicros = microsLive;
  float dt = (nowMicros - lastImuMicros) / 1000000.0f;

  if (dt >= LOOP_DT) {
    lastImuMicros = nowMicros;
    if (dt > 0.02f) dt = 0.02f;
    lastDt = dt;

    updateIMU();
    updateStabilization();
  }

  unsigned long nowMs = millis();
  if (nowMs - lastLoRaCheckMs >= 20) {
    lastLoRaCheckMs = nowMs;
    handleLoRa();
  }
}