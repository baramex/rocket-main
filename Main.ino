#include <BMP280.h>
#include <SD.h>
#include <Wire.h>
#include <MPU9250_WE.h>

// MPU
bool mpuStatus = false;
MPU9250_WE mpu = MPU9250_WE(0x68);
byte mpuSeq = 0;
float verticalAcc[3];
const float verticalAccThreshold = 4.0f / 5.0f;

// BMP
BMP280 bmp = BMP280(0x76);
const unsigned pressure = 1013, updateDelay = 150, vSpeedDelay = 500;
BMP280::eStatus_t bmpStatus = BMP280::eStatusErr;
unsigned long lastStartAlt, apogeeTime, lastUpdate, lastVspeed;
float startAlt, apogeeHeight, lastAlt;
byte bmpSeq = 0;
byte speedSeq = 0;
float altitude[3];
float verticalSpeed[3] = { 0, 0, 0 };
const float verticalSpeedThreshold = 5.0f, altitudeThreshold = 20.0f, altitudeDeltaMax = 50.0f;

// SD Card
const char filename[] = "main.csv";
File file;
String dataBuffer;
const byte chipSelect = 10;
bool sd = false;

// LED
const uint8_t ledPin = 4;
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
const unsigned ejectionDelay = 100, ejectionTryDelay = 500, delayBetweenEjection = 2000;
const float criticalHeightLimit = 150.0f;  // Do not eject payload
const float mediumHeightLimit = 300.0f;    // Eject payload lower than 300m
const float nominalPayloadEjection = 300.0f;
const float nominalMainEjection = 150.0f;
enum EjectionState {
  WAITING,
  EJECTING_LOW,
  EJECTING_HIGH,
  EJECTED
};
struct Ejection {
  EjectionState state = WAITING;
  unsigned long time;
  byte tries = 0;
  const byte pin;
  Ejection(byte pin)
    : pin(pin) {}
};
Ejection payload(7);
Ejection mainParachute(6);

void setup() {
  // Relay
  pinMode(payload.pin, OUTPUT);
  pinMode(mainParachute.pin, OUTPUT);
  digitalWrite(payload.pin, HIGH);
  digitalWrite(mainParachute.pin, HIGH);

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
    bmp.setCtrlMeasSamplingTemp(BMP280::eSampling_no);
    bmp.setCtrlMeasSamplingPress(BMP280::eSampling_X16);
    bmp.setConfigFilter(BMP280::eConfigFilter_X16);
    bmp.setConfigTStandby(BMP280::eConfigTStandby_125);
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

  if (payload.state == EJECTING_LOW || payload.state == EJECTING_HIGH) {
    eject(payload, now);
  }
  if (mainParachute.state == EJECTING_LOW || mainParachute.state == EJECTING_HIGH) {
    eject(mainParachute, now);
  }

  if (sd) {
    const uint16_t chunkSize = file.availableForWrite();
    if (chunkSize && (dataBuffer.length() >= chunkSize || dataBuffer.length() >= 90)) {
      saveToSD(chunkSize);
    }
  }

  if (now - lastUpdate >= updateDelay) {
    if (state == PREPARING) state = READY;
    checkVerticalSpeed(now);
    checkAcceleration();
    updateEjection(now);
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
  if ((state == PREPARING || state == READY) && now - lastStartAlt >= 1000 * 60 * 10) {
    startAlt = getMeanAltitude();

    lastStartAlt = now;
  }

  //Serial.println(state);
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
  for (uint8_t i = 0; i < 10; i++) {
    const float alt = getAltitude();
    addAltitudeSample(alt);
    if (i >= 1) {
      addVerticalSpeedSample((alt - lastAlt) / ((float)(millis() - lastVspeed) / 1000.0f));
      lastAlt = alt;
    }
    lastVspeed = millis();
    if (i >= 3 && abs(lastAlt - getMeanAltitude()) <= 1.0f) {
      return getMeanAltitude();
    }
    lastAlt = alt;
    delay(100);
  }
  return lastAlt;
}

void initialiseAcc() {
  delay(1000);
  for (uint8_t i = 0; i < 10; i++) {
    addVerticalAccSample(mpu.getGValues().x);        // TO CHANGE according to vertical orientation of MPU on the rocket
    if (i >= 2 && abs(getMeanVerticalAcc()) < 0.5f)  // TO CHECK if considerates g acceleration
      return;
    delay(100);
  }
}

float getAltitude() {
  return 44330.0f * (1.0f - pow((float)bmp.getPressure() / 100.0f / pressure, 0.1903f));
}

bool addAltitudeSample(const float& alt) {
  if (altitude[(bmpSeq + 2) % 3] && abs(alt - altitude[(bmpSeq + 2) % 3]) > altitudeDeltaMax) return false;
  altitude[bmpSeq] = alt;
  bmpSeq = (bmpSeq + 1) % 3;
  return true;
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
  const float alt = getAltitude();
  if (addAltitudeSample(alt) && now - lastVspeed >= vSpeedDelay) {
    addVerticalSpeedSample((alt - lastAlt) / ((float)(now - lastVspeed) / 1000.0f));
    lastAlt = alt;
    lastVspeed = now;
  }

  /*Serial.print("Altitude: ");
  Serial.println(getMeanAltitude());
  Serial.print("Vertical Speed: ");
  Serial.println(getMeanVerticalSpeed());*/

  if (state == READY || state == PREPARING) {
    if (getMeanVerticalSpeed() >= verticalSpeedThreshold || getMeanAltitude() - startAlt >= altitudeThreshold) {
      state = TAKINGOFF;
    }
  }

  if (state == TAKINGOFF) {
    if (getMeanVerticalSpeed() <= 0.0f && getMeanAltitude() - startAlt >= altitudeThreshold) {
      state = DROPPING;
      apogeeTime = now;
      apogeeHeight = getMeanAltitude() - startAlt;
    }
  }
}

void checkAcceleration() {
  addVerticalAccSample(mpu.getGValues().x);  // TO CHANGE according to vertical orientation of MPU on the rocket

  if (state == READY || state == PREPARING) {
    if (abs(getMeanVerticalAcc()) >= verticalAccThreshold) {
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

void updateEjection(const unsigned long& now) {
  if (state == DROPPING) {
    if (now - apogeeTime < delayBetweenEjection) return;

    if (payload.state == WAITING && apogeeHeight > criticalHeightLimit) {
      if (apogeeHeight <= mediumHeightLimit) {
        eject(payload, now);
      } else if (getMeanAltitude() - startAlt <= nominalPayloadEjection) {
        eject(payload, now);
      }
    }

    if (mainParachute.state == WAITING) {
      if (apogeeHeight <= criticalHeightLimit) {
        eject(mainParachute, now);
      } else if (getMeanAltitude() - startAlt <= nominalMainEjection) {
        if (now - payload.time >= delayBetweenEjection) eject(mainParachute, now);
      }
    }
  }
}

bool eject(Ejection& eject, const unsigned long& now) {
  if (eject.state == EJECTED) return false;
  if (eject.state == EJECTING_HIGH && now - eject.time < ejectionDelay) return false;
  if (eject.state == EJECTING_LOW && now - eject.time < ejectionTryDelay) return false;

  if (eject.state == WAITING || eject.state == EJECTING_HIGH) {
    eject.state = EJECTING_LOW;
    digitalWrite(eject.pin, LOW);
    eject.tries++;
    eject.time = now;
  } else {
    if (eject.tries >= ejectionTries) eject.state = EJECTED;
    else eject.state = EJECTING_HIGH;
    digitalWrite(eject.pin, HIGH);
  }
  return true;
}