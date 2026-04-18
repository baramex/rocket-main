#include <BMP280.h>
#include <SD.h>
#include <Wire.h>
#include <MPU9250_WE.h>

// TODO: trigger ejection parachute/payload

// MPU
bool mpuStatus = false;
MPU9250_WE mpu = MPU9250_WE(0x68);
byte mpuSeq = 0;
float verticalAcc[3];
const float verticalAccThreshold = 5.0f;

// BMP
BMP280 bmp = BMP280(0x76);
const unsigned pressure = 1013, updateDelay = 300;
BMP280::eStatus_t bmpStatus = BMP280::eStatusErr;
unsigned long lastStartAlt, apogeeTime, lastUpdate;
float startAlt;
byte bmpSeq = 0;
byte speedSeq = 0;
float altitude[3];
float verticalSpeed[3] = { 0, 0, 0 };

// SD Card
const char filename[] = "main.csv";
File file;
String dataBuffer;
const byte chipSelect = 10;
bool sd = false;

// LED
const uint8_t ledPin = 2;
unsigned long lastLedSeq;
uint8_t ledSeq = 0;
const unsigned fastBlink = 500, slowBlink = 1000;
unsigned ledSeqDelay = slowBlink;

// State
const byte maxTries = 10;
enum State {
  PREPARING = 1,
  READY = 2,
  TAKINGOFF = 3,
  DROPPING = 4
};
State state = PREPARING;

// Ejection
const byte ejectionTries = 2;
const unsigned ejectionDelay = 500;
enum EjectionState {
  WAITING,
  EJECTING,
  EJECTED
};
struct Ejection {
  EjectionState state = WAITING;
  unsigned long time;
  byte tries;
};
Ejection payload;
Ejection mainParachute;

void setup() {
  // Debug
  Serial.begin(9600);

  // SD
  dataBuffer.reserve(128);
  for (unsigned i = 0; i < maxTries && !sd; i++) {
    sd = SD.begin(chipSelect);
    Serial.print(".");
  }
  if (sd) file = SD.open(filename, FILE_WRITE);

  // BMP
  Wire.begin();
  for (unsigned i = 0; i < maxTries && bmpStatus != BMP280::eStatusOK; i++) {
    bmpStatus = bmp.begin();
    Serial.print(",");
    delay(100);
  }
  if (bmpStatus == BMP280::eStatusOK) {
    bmp.setCtrlMeasMode(BMP280::eCtrlMeasModeNormal);
    bmp.setCtrlMeasSamplingTemp(BMP280::eSampling_X2);
    bmp.setCtrlMeasSamplingPress(BMP280::eSampling_X16);
    bmp.setConfigFilter(BMP280::eConfigFilter_X16);
    bmp.setConfigTStandby(BMP280::eConfigTStandby_250);
    startAlt = initialiseAlt();
  }

  // MPU
  for (unsigned i = 0; i < maxTries && !mpuStatus; i++) {
    mpuStatus = mpu.init();
    Serial.print(":");
    delay(100);
  }
  mpu.autoOffsets();
  mpu.setSampleRateDivider(5);
  mpu.setAccRange(MPU9250_ACC_RANGE_16G);
  mpu.enableAccDLPF(true);
  mpu.setAccDLPF(MPU9250_DLPF_4);
  initialiseAcc();

  // LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
}

void loop() {
  const unsigned long now = millis();

  if (sd) {
    const uint16_t chunkSize = file.availableForWrite();
    if (chunkSize && (dataBuffer.length() >= chunkSize || dataBuffer.length() >= 90)) {
      saveToSD(chunkSize);
    }
  }

  if ((now - lastUpdate) >= updateDelay) {
    state = READY;
    checkVerticalSpeed(now);
    checkAcceleration();
    if (sd && (state == TAKINGOFF || state == DROPPING)) {
      dataBuffer += String(now) + "," + String(getMeanAltitude(), 1) + "," + String(getMeanVerticalAcc(), 1) + "\n";
    }
    lastUpdate = now;
  }

  if (now - lastLedSeq >= ledSeqDelay && ledSeq != 2) {
    updateLed();
    lastLedSeq = now;
  }

  // Refresh base alt every 10 minutes
  if ((state == PREPARING || state == READY) && (now - lastStartAlt) >= 1000 * 60 * 10) {
    startAlt = getAltitude();

    lastStartAlt = now;
  }
}

void updateLed() {
  ledSeq = ledSeq == 0 ? 1 : 0;
  digitalWrite(ledPin, ledSeq);
  if (state == PREPARING) {
    ledSeqDelay = slowBlink;
  } else if (state == READY) {
    ledSeq = 2;
    digitalWrite(ledPin, HIGH);
  }
}

float initialiseAlt() {
  delay(1000);
  float lastAlt;
  for (uint8_t i = 0; i < 10; i++) {
    const float currAlt = getAltitude();
    addAltitudeSample(currAlt);
    if (abs(lastAlt - currAlt) <= 1 && i >= 2) {
      return currAlt;
    }
    lastAlt = currAlt;
    delay(500);
  }
  return lastAlt;
}

void initialiseAcc() {
  delay(1000);
  for (uint8_t i = 0; i < 3; i++) {
    addVerticalAccSample(mpu.getCorrectedAccRawValues().z);  // TO CHANGE according to vertical orientation of MPU on the rocket
    delay(500);
  }
}

float getAltitude() {
  return 44330.0f * (1.0f - pow((float)bmp.getPressure() / 100.0f / pressure, 0.1903f));
}

void addAltitudeSample(const float& alt) {
  altitude[bmpSeq] = alt;
  bmpSeq = (bmpSeq + 1) % 3;
}

void addVerticalAccSample(const float& acc) {
  verticalAcc[mpuSeq] = acc;
  mpuSeq = (mpuSeq + 1) % 3;
}

void addVerticalSpeedSample(const float& speed) {
  verticalSpeed[speedSeq] = speed;
  speedSeq = (speedSeq + 1) % 3;
}

float getMeanAltitude() {
  return (altitude[0] + altitude[1] + altitude[2]) / 3;
}

float getMeanVerticalAcc() {
  return (verticalAcc[0] + verticalAcc[1] + verticalAcc[2]) / 3;
}

float getMeanVerticalSpeed() {
  return (verticalSpeed[0] + verticalSpeed[1] + verticalSpeed[2]) / 3;
}

void checkVerticalSpeed(const unsigned long& now) {
  const float vz = (getAltitude() - altitude[(bmpSeq - 1) % 3]) / ((float)(now - lastUpdate) / 1000.0f);  // m/s
  addVerticalSpeedSample(vz);
  addAltitudeSample(getAltitude());

  if (state == READY || state == PREPARING) {
    if (getMeanVerticalSpeed() > 5 || getMeanAltitude() - startAlt > 20) {
      state = TAKINGOFF;
    }

    if (getMeanVerticalSpeed() < -5) {
      state = DROPPING;
    }
  }

  if (state == TAKINGOFF) {
    if (getMeanVerticalSpeed() < -0.1) {
      state = DROPPING;
    }
  }
}

void checkAcceleration() {
  addVerticalAccSample(mpu.getCorrectedAccRawValues().z);  // TO CHANGE according to vertical orientation of MPU on the rocket

  if (state == READY || state == PREPARING) {
    if (getMeanVerticalAcc() >= verticalAccThreshold) {
      state = TAKINGOFF;
    }
  }
}

void saveToSD(uint16_t chunkSize) {
  if (chunkSize > dataBuffer.length()) {
    chunkSize = dataBuffer.length();
  }
  file.write(dataBuffer.c_str(), chunkSize);
  file.flush();
  dataBuffer.remove(0, chunkSize);
}