#include <Wire.h>
#include <MadgwickAHRS.h>

#define MPU6050_ADDR 0x68
#define LOOP_HZ 250.0f
#define TEST_DURATION_MS 3000
#define SAFETY_MARGIN_DEG 0.3f

Madgwick filter;

float rollDeg = 0.0f;
float pitchDeg = 0.0f;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

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

void calibrateGyroBias() {
  const int samples = 2000;
  long sx = 0, sy = 0, sz = 0;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpuReadRaw(ax, ay, az, gx, gy, gz);
    sx += gx;
    sy += gy;
    sz += gz;
    delay(1);
  }

  gyroBiasX = (float)sx / samples;
  gyroBiasY = (float)sy / samples;
  gyroBiasZ = (float)sz / samples;
}

void updateIMU() {
  int16_t axr, ayr, azr, gxr, gyr, gzr;
  mpuReadRaw(axr, ayr, azr, gxr, gyr, gzr);

  float ax = axr / 16384.0f;
  float ay = ayr / 16384.0f;
  float az = azr / 16384.0f;

  float gx = (gxr - gyroBiasX) / 131.0f;
  float gy = (gyr - gyroBiasY) / 131.0f;
  float gz = (gzr - gyroBiasZ) / 131.0f;

  filter.updateIMU(gx, gy, gz, ax, ay, az);

  rollDeg = filter.getRoll();
  pitchDeg = filter.getPitch();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(400000);

  mpuInit();
  delay(200);

  Serial.println("Calibrating gyro bias, keep IMU still...");
  calibrateGyroBias();
  Serial.println("Gyro bias done");

  filter.begin(LOOP_HZ);

  for (int i = 0; i < 500; i++) {
    updateIMU();
    delay(4);
  }

  Serial.println("Capturing reference attitude...");
  delay(500);

  updateIMU();
  float rollRef = rollDeg;
  float pitchRef = pitchDeg;

  Serial.print("Roll ref: ");
  Serial.print(rollRef, 3);
  Serial.print(" deg, Pitch ref: ");
  Serial.print(pitchRef, 3);
  Serial.println(" deg");

  Serial.println("Measuring angle deviation, do not move the IMU...");
  delay(300);

  unsigned long startMs = millis();
  float maxRollErr = 0.0f;
  float maxPitchErr = 0.0f;

  while (millis() - startMs < TEST_DURATION_MS) {
    updateIMU();

    float rErr = fabs(rollDeg - rollRef);
    float pErr = fabs(pitchDeg - pitchRef);

    if (rErr > maxRollErr) maxRollErr = rErr;
    if (pErr > maxPitchErr) maxPitchErr = pErr;

    delay(4);
  }

  float recommendedDeadzone =
    max(maxRollErr, maxPitchErr) + SAFETY_MARGIN_DEG;

  Serial.println();
  Serial.print("Max roll deviation: ");
  Serial.print(maxRollErr, 3);
  Serial.println(" deg");

  Serial.print("Max pitch deviation: ");
  Serial.print(maxPitchErr, 3);
  Serial.println(" deg");

  Serial.print("Recommended ANGLE_I_DEADZONE: ");
  Serial.print(recommendedDeadzone, 3);
  Serial.println(" deg");

  Serial.println();
  Serial.println("Done. Power cycle to repeat.");
}

void loop() {}
