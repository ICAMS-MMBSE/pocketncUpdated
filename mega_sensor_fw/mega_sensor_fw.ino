// ==== 7/6/2026 Memphis Burroughs ====
// - Combined "current_final.ino" and "3sensor50hz.ino" (provided by Dr. Osho)
// - Combined the 2 Arduinos into 1 Arduino Mega 2560
// - Implemented Ethernet sheild and MQTT publishing
//
// === Required Libraries: ===
// - Adafruit MPU605
// - Ethernet
// - PubSubClient 

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MQTT_MAX_PACKET_SIZE  256
#define VIBRATION_SENSOR      A0
#define CURRENT_SENSOR        A1
#define IR_SENSOR             2
#define SERIAL_COMMAND_BUFFER_SIZE 48
                                  // 0 = silent, numbers only

const uint8_t  PULSES_PER_REV = 1;

// No-delay print rate
const uint32_t PRINT_MS = 20;      // 50 Hz output
const float    RPM_ALPHA = 0.25f;  // smoothing

// Safe for 1000–10000 RPM
const uint32_t MIN_PULSE_INTERVAL_US = 3000;
const uint32_t RPM_TIMEOUT_US = 250000; // 250 ms
Adafruit_MPU6050 mpu;

// ISR-updated variables
volatile uint32_t lastPulseUs  = 0;
volatile uint32_t lastPeriodUs = 0;
float rpmFiltered = 0.0f;
uint32_t lastPrintMs = 0;

// SCT-013 30A/1V Current Sensor — Arduino Uno/Nano — 50 Hz
// Piecewise linear calibration — accurate across full range
const double SENSOR_RATIO = 30.0;
const double VREF         = 5.0;
const double ADC_STEPS    = 1023.0;
const double V_PER_CNT    = VREF / ADC_STEPS;

// SAMPLING
const uint32_t FRAME_US      = 20000UL;
const uint16_t WINDOW_FRAMES = 10;

// CALIBRATION TABLE
// ------------------------------------------------------------
// Each row = { net_raw, true_amps }
// net_raw = (raw reading with load) - I_ZERO_RAW
//
// HOW TO ADD A POINT:
//   1. With no load connected, send "calibrate 0" over Serial to find I_ZERO_RAW.
//   2. Add that value below and upload the sketch.
//   3. Connect a measured load, let it stabilize, then send
//      "calibrate <actual_amps>" over Serial.
//   4. Add the suggested CAL_TABLE row below in ascending net_raw order.
//
// MINIMUM: 2 points. More points = more accurate.
// Points MUST be in ascending net_raw order.
// Measured with the PocketNC off: `calibrate 0` reported 0.0000 A raw.
const double I_ZERO_RAW = 0.0000;
const double DEADBAND   = 0.03;

struct CalPoint {
  double net_raw;    // raw current minus I_ZERO_RAW
  double true_amps;
};

// --- ADD / EDIT YOUR CALIBRATION POINTS HERE ---
const CalPoint CAL_TABLE[] = {
  { 0.3071, 0.257 },
  { 0.3240, 0.268 },
  { 0.3650, 0.303 },
  { 0.4030, 0.338 },
  { 0.4285, 0.362 },
};

// Set I raw (PNC off)
// Update cal table:
// actual, sensor raw read
// 0.268, 0.3240
// 0.303, 0.3650
// 0.338, 0.4030
// 0.362, 0.4285
// 0.

const uint8_t CAL_POINTS = sizeof(CAL_TABLE) / sizeof(CAL_TABLE[0]);

// ROLLING WINDOW BUFFERS
static uint16_t idx         = 0;
static uint64_t sum_buf     [WINDOW_FRAMES];
static uint64_t sumsq_buf   [WINDOW_FRAMES];
static uint32_t n_buf       [WINDOW_FRAMES];
static uint64_t sum_total   = 0;
static uint64_t sumsq_total = 0;
static uint32_t n_total     = 0;

// Ethernet and MQTT
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0xB3, 0x02 };  // eth shield MAC
IPAddress ip(192, 168, 1, 177);                       // static IP fallback if DHCP fails
const char* mqtt_server = "192.168.1.8";             // broker address
const char* mqtt_topic  = "pocketnc/sensors";
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

// ---------- Runtime controls ----------
// These are intentionally runtime settings. They return to these defaults after
// a reset or power cycle.
bool debugEnabled = false;  // Use raw current instead of calibrated current.

enum SerialOutputMode {
  SERIAL_OUTPUT_OFF,
  SERIAL_OUTPUT_ALL,
  SERIAL_OUTPUT_SINGLE,
};

enum SingleReading {
  READING_VIB_ADC,
  READING_RPM,
  READING_ACCEL_X,
  READING_ACCEL_Y,
  READING_ACCEL_Z,
  READING_GYRO_X,
  READING_GYRO_Y,
  READING_GYRO_Z,
  READING_TEMP_C,
  READING_CURRENT_A,
};

SerialOutputMode serialOutputMode = SERIAL_OUTPUT_ALL;
SingleReading selectedReading = READING_RPM;
bool mqttPublishingEnabled = true;

const uint32_t MQTT_RETRY_MS = 5000UL;
uint32_t lastMqttAttemptMs = 0;

// Keep a rolling set of raw readings for the interactive calibration command.
const uint16_t CALIBRATION_SAMPLE_COUNT = 100;
float calibrationRawSamples[CALIBRATION_SAMPLE_COUNT];
uint16_t calibrationSampleIndex = 0;
uint16_t calibrationSamplesStored = 0;

char serialCommandBuffer[SERIAL_COMMAND_BUFFER_SIZE];
uint8_t serialCommandLength = 0;

// ---------- Accelerometer calibration ----------
// Defaults are identity corrections. After a six-position calibration, copy
// the reported values here to retain the correction across a reset.
const float GRAVITY_MS2 = 9.80665f;
const float ACCEL_OFFSET_DEFAULT[3] = { 0.0f, 0.0f, 0.0f };
const float ACCEL_SCALE_DEFAULT[3]  = { 1.0f, 1.0f, 1.0f };

float accelOffset[3] = { 0.0f, 0.0f, 0.0f };
float accelScale[3]  = { 1.0f, 1.0f, 1.0f };

const uint16_t ACCEL_CAL_SAMPLE_COUNT = 100;
float accelCalPositive[3];
float accelCalNegative[3];
bool accelCalPositiveCaptured[3] = { false, false, false };
bool accelCalNegativeCaptured[3] = { false, false, false };
int8_t accelCalActiveAxis = -1;
int8_t accelCalActiveSign = 0;
uint16_t accelCalSamplesCollected = 0;
double accelCalSums[3] = { 0.0, 0.0, 0.0 };

// A machine reference is a quick operational zero taken with the machine
// homed. It is separate from the six-face sensor calibration above.
bool accelMachineReferenceActive = false;
bool accelMachineCaptureActive = false;
float accelMachineReference[3] = { 0.0f, 0.0f, 0.0f };
uint16_t accelMachineSamplesCollected = 0;
double accelMachineSums[3] = { 0.0, 0.0, 0.0 };

// The gyroscope is expected to read zero while the homed machine is still.
// Its bias is captured alongside the machine accelerometer reference.
const float GYRO_OFFSET_DEFAULT[3] = { 0.0f, 0.0f, 0.0f };
float gyroOffset[3] = { 0.0f, 0.0f, 0.0f };
bool gyroMachineCaptureActive = false;
uint16_t gyroMachineSamplesCollected = 0;
double gyroMachineSums[3] = { 0.0, 0.0, 0.0 };

void pulseCounter() {
  uint32_t nowUs = micros();
  uint32_t dt = nowUs - lastPulseUs;

  // Debounce / chatter rejection
  if (dt < MIN_PULSE_INTERVAL_US) return;

  if (lastPulseUs != 0) {
    lastPeriodUs = dt;   // period between valid pulses
  }
  lastPulseUs = nowUs;
}

static inline void atomicReadPulseTiming(uint32_t &lpUs, uint32_t &perUs) {
  noInterrupts();
  lpUs  = lastPulseUs;
  perUs = lastPeriodUs;
  interrupts();
}

void resetBuffer() {
  idx = 0;
  for (uint16_t i = 0; i < WINDOW_FRAMES; i++) {
    sum_buf[i] = sumsq_buf[i] = 0;
    n_buf[i] = 0;
  }
  sum_total = sumsq_total = n_total = 0;
}

void collectFrame() {
  uint32_t t0 = micros();
  uint64_t sum = 0, sumsq = 0;
  uint32_t n = 0;

  while ((uint32_t)(micros() - t0) < FRAME_US) {
    uint16_t x = analogRead(CURRENT_SENSOR);
    sum   += x;
    sumsq += (uint32_t)x * x;
    n++;
  }

  sum_total   -= sum_buf[idx];
  sumsq_total -= sumsq_buf[idx];
  n_total     -= n_buf[idx];

  sum_buf[idx]   = sum;
  sumsq_buf[idx] = sumsq;
  n_buf[idx]     = n;

  sum_total   += sum;
  sumsq_total += sumsq;
  n_total     += n;

  if (++idx >= WINDOW_FRAMES) idx = 0;
}

double rmsCountsAC() {
  if (n_total == 0) return 0.0;
  double mean = (double)sum_total   / (double)n_total;
  double ex2  = (double)sumsq_total / (double)n_total;
  double var  = ex2 - mean * mean;
  return (var > 0.0) ? sqrt(var) : 0.0;
}

double getRawCurrent() {
  return rmsCountsAC() * V_PER_CNT * SENSOR_RATIO;
}

// Piecewise linear interpolation through CAL_TABLE
// Extrapolates using the nearest slope outside the table range
double piecewiseCalibrate(double net) {
  if (net <= 0.0) return 0.0;

  // Below first point — extrapolate using first segment slope
  if (net <= CAL_TABLE[0].net_raw) {
    double slope = CAL_TABLE[1].true_amps - CAL_TABLE[0].true_amps;
    slope /= CAL_TABLE[1].net_raw - CAL_TABLE[0].net_raw;
    return CAL_TABLE[0].true_amps + slope * (net - CAL_TABLE[0].net_raw);
  }

  // Above last point — extrapolate using last segment slope
  if (net >= CAL_TABLE[CAL_POINTS - 1].net_raw) {
    double slope = CAL_TABLE[CAL_POINTS-1].true_amps - CAL_TABLE[CAL_POINTS-2].true_amps;
    slope /= CAL_TABLE[CAL_POINTS-1].net_raw - CAL_TABLE[CAL_POINTS-2].net_raw;
    return CAL_TABLE[CAL_POINTS-1].true_amps + slope * (net - CAL_TABLE[CAL_POINTS-1].net_raw);
  }

  // Interpolate between surrounding points
  for (uint8_t i = 0; i < CAL_POINTS - 1; i++) {
    if (net >= CAL_TABLE[i].net_raw && net <= CAL_TABLE[i+1].net_raw) {
      double t = (net - CAL_TABLE[i].net_raw) /
                 (CAL_TABLE[i+1].net_raw - CAL_TABLE[i].net_raw);
      return CAL_TABLE[i].true_amps + t * (CAL_TABLE[i+1].true_amps - CAL_TABLE[i].true_amps);
    }
  }

  return 0.0;
}

double getCalibratedCurrent(double raw) {
  double net = raw - I_ZERO_RAW;
  if (net < 0.0) net = 0.0;
  double cal = piecewiseCalibrate(net);
  return (cal < DEADBAND) ? 0.0 : cal;
}

void recordCalibrationSample(double rawCurrent) {
  calibrationRawSamples[calibrationSampleIndex] = (float)rawCurrent;
  calibrationSampleIndex =
    (calibrationSampleIndex + 1) % CALIBRATION_SAMPLE_COUNT;

  if (calibrationSamplesStored < CALIBRATION_SAMPLE_COUNT) {
    ++calibrationSamplesStored;
  }
}

double medianCalibrationRaw() {
  float sorted[CALIBRATION_SAMPLE_COUNT];
  for (uint16_t i = 0; i < CALIBRATION_SAMPLE_COUNT; ++i) {
    sorted[i] = calibrationRawSamples[i];
  }

  // A small insertion sort is deterministic and avoids dynamic allocation.
  for (uint16_t i = 1; i < CALIBRATION_SAMPLE_COUNT; ++i) {
    const float value = sorted[i];
    int16_t j = (int16_t)i - 1;
    while (j >= 0 && sorted[j] > value) {
      sorted[j + 1] = sorted[j];
      --j;
    }
    sorted[j + 1] = value;
  }

  const uint16_t upper = CALIBRATION_SAMPLE_COUNT / 2;
  return ((double)sorted[upper - 1] + (double)sorted[upper]) * 0.5;
}

void showCalibrationResult(double actualCurrent) {
  if (calibrationSamplesStored < CALIBRATION_SAMPLE_COUNT) {
    Serial.print("# calibration needs ");
    Serial.print(CALIBRATION_SAMPLE_COUNT - calibrationSamplesStored);
    Serial.println(" more raw-current samples");
    return;
  }

  const double medianRaw = medianCalibrationRaw();
  double medianNetRaw = medianRaw - I_ZERO_RAW;
  if (medianNetRaw < 0.0) medianNetRaw = 0.0;

  Serial.println("# calibration result");
  Serial.print("# median_raw_current_A=");
  Serial.println(medianRaw, 4);

  if (actualCurrent == 0.0) {
    Serial.print("# suggested I_ZERO_RAW=");
    Serial.println(medianRaw, 4);
    Serial.println("# update I_ZERO_RAW and upload before collecting load points");
  } else {
    Serial.print("# suggested CAL_TABLE row: { ");
    Serial.print(medianNetRaw, 4);
    Serial.print(", ");
    Serial.print(actualCurrent, 4);
    Serial.println(" },");
  }
}

const char *accelAxisName(uint8_t axis) {
  static const char *const names[] = { "x", "y", "z" };
  return names[axis];
}

void resetAccelCalibration() {
  accelCalActiveAxis = -1;
  accelCalActiveSign = 0;
  accelCalSamplesCollected = 0;
  accelMachineReferenceActive = false;
  accelMachineCaptureActive = false;
  accelMachineSamplesCollected = 0;
  gyroMachineCaptureActive = false;
  gyroMachineSamplesCollected = 0;

  for (uint8_t axis = 0; axis < 3; ++axis) {
    accelCalPositiveCaptured[axis] = false;
    accelCalNegativeCaptured[axis] = false;
    accelOffset[axis] = ACCEL_OFFSET_DEFAULT[axis];
    accelScale[axis] = ACCEL_SCALE_DEFAULT[axis];
    gyroOffset[axis] = GYRO_OFFSET_DEFAULT[axis];
  }
  Serial.println("# accelerometer and gyroscope calibration reset; default correction restored");
}

void startMachineReferenceCapture() {
  if (accelCalActiveAxis >= 0) {
    Serial.println("# accelcal is collecting a six-face sample; wait for it to finish first");
    return;
  }

  accelMachineSamplesCollected = 0;
  accelMachineCaptureActive = true;
  for (uint8_t axis = 0; axis < 3; ++axis) accelMachineSums[axis] = 0.0;
  Serial.print("# accelcal machine: home the machine and keep it still; collecting ");
  Serial.print(ACCEL_CAL_SAMPLE_COUNT);
  Serial.println(" samples");
}

void collectMachineReferenceSample(const sensors_event_t &rawAccel) {
  if (!accelMachineCaptureActive) return;

  // Apply the sensor's current offset/scale correction, but not a previous
  // machine reference. This allows `accelcal machine` to re-home at any time.
  accelMachineSums[0] += (rawAccel.acceleration.x - accelOffset[0]) * accelScale[0];
  accelMachineSums[1] += (rawAccel.acceleration.y - accelOffset[1]) * accelScale[1];
  accelMachineSums[2] += (rawAccel.acceleration.z - accelOffset[2]) * accelScale[2];
  ++accelMachineSamplesCollected;

  if (accelMachineSamplesCollected < ACCEL_CAL_SAMPLE_COUNT) return;

  for (uint8_t axis = 0; axis < 3; ++axis) {
    accelMachineReference[axis] =
      (float)(accelMachineSums[axis] / ACCEL_CAL_SAMPLE_COUNT);
  }
  accelMachineReferenceActive = true;
  accelMachineCaptureActive = false;

  Serial.println("# machine accelerometer reference applied");
  Serial.print("# home reference x=");
  Serial.print(accelMachineReference[0], 4);
  Serial.print(" y=");
  Serial.print(accelMachineReference[1], 4);
  Serial.print(" z=");
  Serial.println(accelMachineReference[2], 4);
  Serial.println("# output at home: x=0, y=0, z=0");
}

void startGyroReferenceCapture() {
  gyroMachineSamplesCollected = 0;
  gyroMachineCaptureActive = true;
  for (uint8_t axis = 0; axis < 3; ++axis) gyroMachineSums[axis] = 0.0;
}

void collectGyroReferenceSample(const sensors_event_t &rawGyro) {
  if (!gyroMachineCaptureActive) return;

  gyroMachineSums[0] += rawGyro.gyro.x;
  gyroMachineSums[1] += rawGyro.gyro.y;
  gyroMachineSums[2] += rawGyro.gyro.z;
  ++gyroMachineSamplesCollected;

  if (gyroMachineSamplesCollected < ACCEL_CAL_SAMPLE_COUNT) return;

  for (uint8_t axis = 0; axis < 3; ++axis) {
    gyroOffset[axis] = (float)(gyroMachineSums[axis] / ACCEL_CAL_SAMPLE_COUNT);
  }
  gyroMachineCaptureActive = false;

  Serial.println("# gyroscope bias applied for this session");
  Serial.print("# GYRO_OFFSET_DEFAULT = { ");
  Serial.print(gyroOffset[0], 6); Serial.print(", ");
  Serial.print(gyroOffset[1], 6); Serial.print(", ");
  Serial.print(gyroOffset[2], 6); Serial.println(" };");
}

void startMachineCalibration() {
  if (accelCalActiveAxis >= 0) {
    Serial.println("# accelcal is collecting a six-face sample; wait for it to finish first");
    return;
  }

  startMachineReferenceCapture();
  startGyroReferenceCapture();
  Serial.println("# machine calibration will zero accel X/Y/Z and gyro X/Y/Z at home");
}

void printAccelCalibrationStatus() {
  Serial.print("# accelcal faces x=");
  Serial.print(accelCalPositiveCaptured[0] ? "+" : ".");
  Serial.print(accelCalNegativeCaptured[0] ? "-" : ".");
  Serial.print(" y=");
  Serial.print(accelCalPositiveCaptured[1] ? "+" : ".");
  Serial.print(accelCalNegativeCaptured[1] ? "-" : ".");
  Serial.print(" z=");
  Serial.print(accelCalPositiveCaptured[2] ? "+" : ".");
  Serial.println(accelCalNegativeCaptured[2] ? "-" : ".");

  Serial.print("# machine reference=");
  Serial.println(accelMachineReferenceActive ? "active" : "not set");

  if (accelCalActiveAxis >= 0) {
    Serial.print("# collecting ");
    Serial.print(accelCalActiveSign > 0 ? "+" : "-");
    Serial.print(accelAxisName(accelCalActiveAxis));
    Serial.print(": ");
    Serial.print(accelCalSamplesCollected);
    Serial.print("/");
    Serial.println(ACCEL_CAL_SAMPLE_COUNT);
  } else if (accelMachineCaptureActive) {
    Serial.print("# collecting machine reference: ");
    Serial.print(accelMachineSamplesCollected);
    Serial.print("/");
    Serial.println(ACCEL_CAL_SAMPLE_COUNT);
  }

  if (gyroMachineCaptureActive) {
    Serial.print("# collecting gyroscope bias: ");
    Serial.print(gyroMachineSamplesCollected);
    Serial.print("/");
    Serial.println(ACCEL_CAL_SAMPLE_COUNT);
  }
}

bool accelCalibrationComplete() {
  for (uint8_t axis = 0; axis < 3; ++axis) {
    if (!accelCalPositiveCaptured[axis] || !accelCalNegativeCaptured[axis]) {
      return false;
    }
  }
  return true;
}

void applyAccelCalibration() {
  if (!accelCalibrationComplete()) {
    Serial.println("# accelcal needs all six faces before it can be applied");
    return;
  }

  for (uint8_t axis = 0; axis < 3; ++axis) {
    const float span = accelCalPositive[axis] - accelCalNegative[axis];
    if (span < GRAVITY_MS2) {
      Serial.print("# accelcal ");
      Serial.print(accelAxisName(axis));
      Serial.println(" failed: + and - face values are inconsistent; recapture both faces");
      return;
    }
    accelOffset[axis] = (accelCalPositive[axis] + accelCalNegative[axis]) * 0.5f;
    accelScale[axis] = (2.0f * GRAVITY_MS2) / span;
  }

  // A machine reference is expressed in the previous calibrated coordinates.
  // It must be recaptured after changing the underlying sensor correction.
  accelMachineReferenceActive = false;

  Serial.println("# accelerometer calibration applied for this session");
  Serial.print("# ACCEL_OFFSET_DEFAULT = { ");
  Serial.print(accelOffset[0], 6); Serial.print(", ");
  Serial.print(accelOffset[1], 6); Serial.print(", ");
  Serial.print(accelOffset[2], 6); Serial.println(" };");
  Serial.print("# ACCEL_SCALE_DEFAULT  = { ");
  Serial.print(accelScale[0], 6); Serial.print(", ");
  Serial.print(accelScale[1], 6); Serial.print(", ");
  Serial.print(accelScale[2], 6); Serial.println(" };");
  Serial.println("# copy these values into the sketch to retain them after reset");
  Serial.println("# run accelcal machine again to set the machine-home reference");
}

void startAccelCalibrationCapture(const char *face) {
  if (face == NULL || strlen(face) != 2 ||
      (face[0] != '+' && face[0] != '-') ||
      (face[1] != 'x' && face[1] != 'y' && face[1] != 'z')) {
    Serial.println("# usage: accelcal +x|-x|+y|-y|+z|-z | accelcal status|reset|apply");
    return;
  }

  accelCalActiveAxis = face[1] - 'x';
  accelCalActiveSign = face[0] == '+' ? 1 : -1;
  accelCalSamplesCollected = 0;
  for (uint8_t axis = 0; axis < 3; ++axis) accelCalSums[axis] = 0.0;

  Serial.print("# accelcal ");
  Serial.print(face);
  Serial.print(": hold that sensor axis straight up and still; collecting ");
  Serial.print(ACCEL_CAL_SAMPLE_COUNT);
  Serial.println(" samples");
}

void collectAccelCalibrationSample(const sensors_event_t &accel) {
  if (accelCalActiveAxis < 0) return;

  accelCalSums[0] += accel.acceleration.x;
  accelCalSums[1] += accel.acceleration.y;
  accelCalSums[2] += accel.acceleration.z;
  ++accelCalSamplesCollected;

  if (accelCalSamplesCollected < ACCEL_CAL_SAMPLE_COUNT) return;

  const uint8_t axis = (uint8_t)accelCalActiveAxis;
  float *target = accelCalActiveSign > 0
    ? accelCalPositive
    : accelCalNegative;
  for (uint8_t i = 0; i < 3; ++i) {
    target[i] = (float)(accelCalSums[i] / ACCEL_CAL_SAMPLE_COUNT);
  }

  if (accelCalActiveSign > 0) {
    accelCalPositiveCaptured[axis] = true;
  } else {
    accelCalNegativeCaptured[axis] = true;
  }

  Serial.print("# accelcal ");
  Serial.print(accelCalActiveSign > 0 ? "+" : "-");
  Serial.print(accelAxisName(axis));
  Serial.print(" captured: axis reading=");
  Serial.println(target[axis], 4);
  accelCalActiveAxis = -1;
  accelCalActiveSign = 0;

  if (accelCalibrationComplete()) applyAccelCalibration();
}

void correctedAcceleration(const sensors_event_t &raw,
                           float &x, float &y, float &z) {
  x = (raw.acceleration.x - accelOffset[0]) * accelScale[0];
  y = (raw.acceleration.y - accelOffset[1]) * accelScale[1];
  z = (raw.acceleration.z - accelOffset[2]) * accelScale[2];

  if (accelMachineReferenceActive) {
    x -= accelMachineReference[0];
    y -= accelMachineReference[1];
    z -= accelMachineReference[2];
  }
}

void correctedGyroscope(const sensors_event_t &raw,
                        float &x, float &y, float &z) {
  x = raw.gyro.x - gyroOffset[0];
  y = raw.gyro.y - gyroOffset[1];
  z = raw.gyro.z - gyroOffset[2];
}

void printCommandHelp() {
  Serial.println("# commands: help | status | debug on|off | mqtt on|off");
  Serial.println("#           serial on|off|all | serial single <reading>");
  Serial.println("# readings: vib_adc rpm accel_x accel_y accel_z gyro_x gyro_y gyro_z temp_c current_a");
  Serial.println("#           calibrate <actual_amps>  (use 0 for I_ZERO_RAW)");
  Serial.println("#           machinecal | accelcal machine | accelcal +x|-x|+y|-y|+z|-z");
  Serial.println("#           accelcal status|reset|apply");
}

const char *selectedReadingName() {
  switch (selectedReading) {
    case READING_VIB_ADC:   return "vib_adc";
    case READING_RPM:       return "rpm";
    case READING_ACCEL_X:   return "accel_x";
    case READING_ACCEL_Y:   return "accel_y";
    case READING_ACCEL_Z:   return "accel_z";
    case READING_GYRO_X:    return "gyro_x";
    case READING_GYRO_Y:    return "gyro_y";
    case READING_GYRO_Z:    return "gyro_z";
    case READING_TEMP_C:    return "temp_c";
    case READING_CURRENT_A: return "current_a";
  }
  return "unknown";
}

void printStatus() {
  Serial.print("# status debug=");
  Serial.print(debugEnabled ? "on" : "off");
  Serial.print(" serial=");
  if (serialOutputMode == SERIAL_OUTPUT_OFF) {
    Serial.print("off");
  } else if (serialOutputMode == SERIAL_OUTPUT_SINGLE) {
    Serial.print("single:");
    Serial.print(selectedReadingName());
  } else {
    Serial.print("all");
  }
  Serial.print(" mqtt=");
  Serial.print(mqttPublishingEnabled ? "on" : "off");
  Serial.print(" mqtt_connected=");
  Serial.print(mqttClient.connected() ? "yes" : "no");
  Serial.print(" calibration_samples=");
  Serial.print(calibrationSamplesStored);
  Serial.print("/");
  Serial.println(CALIBRATION_SAMPLE_COUNT);
}

void lowercase(char *text) {
  while (*text != '\0') {
    if (*text >= 'A' && *text <= 'Z') {
      *text = *text - 'A' + 'a';
    }
    ++text;
  }
}

bool setOnOffCommand(const char *setting, const char *value, bool &target) {
  if (value == NULL || (strcmp(value, "on") != 0 && strcmp(value, "off") != 0)) {
    Serial.print("# usage: ");
    Serial.print(setting);
    Serial.println(" on|off");
    return false;
  }

  target = strcmp(value, "on") == 0;
  Serial.print("# ");
  Serial.print(setting);
  Serial.print("=");
  Serial.println(target ? "on" : "off");
  return true;
}

bool setSelectedReading(const char *name) {
  if (name == NULL) return false;

  if (strcmp(name, "vib") == 0 || strcmp(name, "vib_adc") == 0) {
    selectedReading = READING_VIB_ADC;
  } else if (strcmp(name, "rpm") == 0) {
    selectedReading = READING_RPM;
  } else if (strcmp(name, "accel_x") == 0) {
    selectedReading = READING_ACCEL_X;
  } else if (strcmp(name, "accel_y") == 0) {
    selectedReading = READING_ACCEL_Y;
  } else if (strcmp(name, "accel_z") == 0) {
    selectedReading = READING_ACCEL_Z;
  } else if (strcmp(name, "gyro_x") == 0) {
    selectedReading = READING_GYRO_X;
  } else if (strcmp(name, "gyro_y") == 0) {
    selectedReading = READING_GYRO_Y;
  } else if (strcmp(name, "gyro_z") == 0) {
    selectedReading = READING_GYRO_Z;
  } else if (strcmp(name, "temp") == 0 || strcmp(name, "temp_c") == 0) {
    selectedReading = READING_TEMP_C;
  } else if (strcmp(name, "current") == 0 || strcmp(name, "current_a") == 0) {
    selectedReading = READING_CURRENT_A;
  } else {
    return false;
  }
  return true;
}

void setSerialOutputCommand(const char *mode, const char *reading) {
  if (mode == NULL) {
    Serial.println("# usage: serial on|off|all | serial single <reading>");
    return;
  }

  if (strcmp(mode, "on") == 0 || strcmp(mode, "all") == 0) {
    serialOutputMode = SERIAL_OUTPUT_ALL;
    Serial.println("# serial=all");
  } else if (strcmp(mode, "off") == 0) {
    serialOutputMode = SERIAL_OUTPUT_OFF;
    Serial.println("# serial=off");
  } else if (strcmp(mode, "single") == 0 && setSelectedReading(reading)) {
    serialOutputMode = SERIAL_OUTPUT_SINGLE;
    Serial.print("# serial=single:");
    Serial.println(selectedReadingName());
  } else {
    Serial.println("# usage: serial on|off|all | serial single <reading>");
  }
}

void processSerialCommand(char *command) {
  lowercase(command);
  char *name = strtok(command, " \t");
  if (name == NULL) return;

  if (strcmp(name, "help") == 0 || strcmp(name, "?") == 0) {
    printCommandHelp();
  } else if (strcmp(name, "status") == 0) {
    printStatus();
  } else if (strcmp(name, "machinecal") == 0) {
    startMachineCalibration();
  } else if (strcmp(name, "debug") == 0) {
    setOnOffCommand("debug", strtok(NULL, " \t"), debugEnabled);
  } else if (strcmp(name, "serial") == 0) {
    char *mode = strtok(NULL, " \t");
    setSerialOutputCommand(mode, strtok(NULL, " \t"));
  } else if (strcmp(name, "mqtt") == 0) {
    const bool wasEnabled = mqttPublishingEnabled;
    if (setOnOffCommand("mqtt", strtok(NULL, " \t"), mqttPublishingEnabled) &&
        wasEnabled != mqttPublishingEnabled) {
      if (!mqttPublishingEnabled && mqttClient.connected()) {
        mqttClient.disconnect();
        Serial.println("# MQTT disconnected");
      } else if (mqttPublishingEnabled) {
        // Allow a newly enabled MQTT connection attempt on the next loop.
        lastMqttAttemptMs = millis() - MQTT_RETRY_MS;
      }
    }
  } else if (strcmp(name, "accelcal") == 0) {
    char *option = strtok(NULL, " \t");
    if (option != NULL && strcmp(option, "status") == 0) {
      printAccelCalibrationStatus();
    } else if (option != NULL && strcmp(option, "reset") == 0) {
      resetAccelCalibration();
    } else if (option != NULL && strcmp(option, "machine") == 0) {
      startMachineCalibration();
    } else if (option != NULL && strcmp(option, "apply") == 0) {
      applyAccelCalibration();
    } else {
      startAccelCalibrationCapture(option);
    }
  } else if (strcmp(name, "calibrate") == 0 || strcmp(name, "calibration") == 0) {
    char *value = strtok(NULL, " \t");
    char *endPointer;
    const double actualCurrent = value == NULL ? -1.0 : strtod(value, &endPointer);
    if (value == NULL || *endPointer != '\0' || !isfinite(actualCurrent) || actualCurrent < 0.0) {
      Serial.println("# usage: calibrate <actual_amps>; actual_amps must be non-negative");
    } else {
      showCalibrationResult(actualCurrent);
    }
  } else {
    Serial.print("# unknown command: ");
    Serial.println(name);
    printCommandHelp();
  }
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char incoming = (char)Serial.read();

    if (incoming == '\n' || incoming == '\r') {
      if (serialCommandLength > 0) {
        serialCommandBuffer[serialCommandLength] = '\0';
        processSerialCommand(serialCommandBuffer);
        serialCommandLength = 0;
      }
      continue;
    }

    if (incoming >= 32 && incoming <= 126) {
      if (serialCommandLength < SERIAL_COMMAND_BUFFER_SIZE - 1) {
        serialCommandBuffer[serialCommandLength++] = incoming;
      } else {
        serialCommandLength = 0;
        Serial.println("# command too long; input cleared");
      }
    }
  }
}

void setupNetwork() {
  Serial.println("Starting Ethernet...");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("DHCP failed, using static IP");
    Ethernet.begin(mac, ip);
  }
  delay(1000);

  Serial.print("My IP: ");
  Serial.println(Ethernet.localIP());

  mqttClient.setServer(mqtt_server, 1883);
}

void maintainMQTT(uint32_t nowMs) {
  if (!mqttPublishingEnabled || mqttClient.connected() ||
      (uint32_t)(nowMs - lastMqttAttemptMs) < MQTT_RETRY_MS) {
    return;
  }

  lastMqttAttemptMs = nowMs;
  if (debugEnabled) Serial.println("# attempting MQTT connection");

  if (mqttClient.connect("PocketNC_Mega")) {
    if (debugEnabled) Serial.println("# MQTT connected");
  } else if (debugEnabled) {
    Serial.print("# MQTT connection failed, rc=");
    Serial.println(mqttClient.state());
  }
}

void printSelectedReading(int vibration, float rpm, float accelX, float accelY, float accelZ,
                          float gyroX, float gyroY, float gyroZ,
                          const sensors_event_t &temperature, double current) {
  // Single-reading mode intentionally outputs one unlabeled numeric value per
  // line. The command acknowledgement identifies which reading is selected.
  switch (selectedReading) {
    case READING_VIB_ADC:   Serial.println(vibration); break;
    case READING_RPM:       Serial.println(rpm, 2); break;
    case READING_ACCEL_X:   Serial.println(accelX, 3); break;
    case READING_ACCEL_Y:   Serial.println(accelY, 3); break;
    case READING_ACCEL_Z:   Serial.println(accelZ, 3); break;
    case READING_GYRO_X:    Serial.println(gyroX, 3); break;
    case READING_GYRO_Y:    Serial.println(gyroY, 3); break;
    case READING_GYRO_Z:    Serial.println(gyroZ, 3); break;
    case READING_TEMP_C:    Serial.println(temperature.temperature, 2); break;
    case READING_CURRENT_A: Serial.println(current, 4); break;
  }
}

void setup() {
  Serial.begin(115200);
  // Load any saved accelerometer and gyroscope correction constants.
  resetAccelCalibration();

  pinMode(VIBRATION_SENSOR, INPUT);
  pinMode(IR_SENSOR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(IR_SENSOR), pulseCounter, FALLING);
  // If your TCRT5000 output is inverted, switch FALLING -> RISING

  Wire.begin();
  Wire.setClock(400000);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {}
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  setupNetwork();
  lastMqttAttemptMs = millis() - MQTT_RETRY_MS;

  // CSV header. Lines beginning with '#' are command/status messages.
  Serial.println("vib_adc,rpm,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp_c,current_A");
  printCommandHelp();
  printStatus();

  lastPrintMs = millis();

  delay(500);

  resetBuffer();
  for (uint16_t i = 0; i < WINDOW_FRAMES + 2; i++) collectFrame();

}

void loop() {
  uint32_t nowMs = millis();
  uint32_t nowUs = micros();

  handleSerialCommands();

  // Read pulse timing atomically
  uint32_t lpUs, perUs;
  atomicReadPulseTiming(lpUs, perUs);

  // Compute RPM from pulse-to-pulse period
  float rpmInstant = 0.0f;
  if (lpUs != 0 && (nowUs - lpUs) <= RPM_TIMEOUT_US && perUs > 0) {
    rpmInstant = (60.0f * 1000000.0f) / ((float)perUs * (float)PULSES_PER_REV);
  } else {
    rpmInstant = 0.0f;
  }

  // Smooth RPM
  rpmFiltered = RPM_ALPHA * rpmInstant + (1.0f - RPM_ALPHA) * rpmFiltered;

  collectFrame();

  maintainMQTT(nowMs);
  if (mqttPublishingEnabled && mqttClient.connected()) mqttClient.loop();

  // Print at fixed interval (no delay)
  if ((uint32_t)(nowMs - lastPrintMs) >= PRINT_MS) {
    lastPrintMs = nowMs;
    int vib = analogRead(VIBRATION_SENSOR);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    collectAccelCalibrationSample(a);
    collectMachineReferenceSample(a);
    collectGyroReferenceSample(g);
    float accelX, accelY, accelZ;
    correctedAcceleration(a, accelX, accelY, accelZ);
    float gyroX, gyroY, gyroZ;
    correctedGyroscope(g, gyroX, gyroY, gyroZ);

    const double rawCurrent = getRawCurrent();
    recordCalibrationSample(rawCurrent);
    const double current = debugEnabled
      ? rawCurrent
      : getCalibratedCurrent(rawCurrent);

    if (serialOutputMode == SERIAL_OUTPUT_ALL) {
      // One-line CSV (NO t_ms, NO pulse_count)
      Serial.print(vib);                      Serial.print(',');
      Serial.print(rpmFiltered,       2);     Serial.print(',');
      Serial.print(accelX,            3);     Serial.print(',');
      Serial.print(accelY,            3);     Serial.print(',');
      Serial.print(accelZ,            3);     Serial.print(',');
      Serial.print(gyroX,             3);     Serial.print(',');
      Serial.print(gyroY,             3);     Serial.print(',');
      Serial.print(gyroZ,             3);     Serial.print(',');
      Serial.print(temp.temperature,  2);     Serial.print(',');
      Serial.print(current,           4);     Serial.println();
    } else if (serialOutputMode == SERIAL_OUTPUT_SINGLE) {
      printSelectedReading(vib, rpmFiltered, accelX, accelY, accelZ,
                           gyroX, gyroY, gyroZ, temp, current);
    }

    char vibStr[8], rpmStr[10], axStr[10], ayStr[10], azStr[10];
    char gxStr[10], gyStr[10], gzStr[10], tempStr[10], curStr[10];

    // MQTT Output
    itoa(vib, vibStr, 10);
    dtostrf(rpmFiltered,        1, 2, rpmStr);
    dtostrf(accelX,             1, 3, axStr);
    dtostrf(accelY,             1, 3, ayStr);
    dtostrf(accelZ,             1, 3, azStr);
    dtostrf(gyroX,              1, 3, gxStr);
    dtostrf(gyroY,              1, 3, gyStr);
    dtostrf(gyroZ,              1, 3, gzStr);
    dtostrf(temp.temperature,   1, 2, tempStr);
    dtostrf(current,            1, 4, curStr);

    char payload[200];
    snprintf(payload, sizeof(payload),
      "{\"vib_adc\":%s,\"rpm\":%s,\"accel_x\":%s,\"accel_y\":%s,\"accel_z\":%s,"
      "\"gyro_x\":%s,\"gyro_y\":%s,\"gyro_z\":%s,\"temp_c\":%s,\"current_A\":%s}",
      vibStr, rpmStr, axStr, ayStr, azStr, gxStr, gyStr, gzStr, tempStr, curStr);

    if (mqttPublishingEnabled && mqttClient.connected()) {
      mqttClient.publish(mqtt_topic, payload);
    }
  }
}
