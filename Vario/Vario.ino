/*
  Variometer for paragliding — Arduino MKR Zero + BMP280 + LSM6DS3 + buzzer.

  The barometer alone can only see a climb after it has already happened: it
  measures altitude, and velocity has to be differentiated out of it, which
  costs about a second of lag. The IMU measures the acceleration directly, so
  the filter can predict forward at 104 Hz and use the barometer only to keep
  that prediction honest. Response to a sharp gust drops from ~1.3 s to ~0.1 s
  without getting noisier.

  Wiring (MKR Zero, 3.3 V logic — do NOT feed these sensors 5 V):
    BMP280 / AHT20+BMP280 module    LSM6DS3 module
      VCC -> VCC (3.3 V)              VCC -> VCC (3.3 V)
      GND -> GND                      GND -> GND
      SDA -> D11                      SDA -> D11
      SCL -> D12                      SCL -> D12
    Buzzer -> D5 through a 100 ohm resistor, other leg to GND.

    Both sensors share the I2C bus. The AHT20 on the combo module sits at 0x38
    and is simply ignored. Mount the IMU rigidly to the same body as the
    barometer; the orientation does not matter, as long as the vario is held
    still during the startup calibration (see setup()).

  Libraries: "Adafruit BMP280 Library", "Adafruit LSM6DS", "Adafruit BusIO",
             "Adafruit Unified Sensor". Board: "Arduino SAMD Boards".

  NOTE ON THE BOARD CHANGE: NewTone is an AVR-only library (it writes ATmega
  timer registers directly) and cannot build for the SAMD21. The SAMD core
  provides its own tone()/noTone() using TC5, which is what this sketch uses.

  MKR Zero pin budget, so the planned additions do not collide:
    D5            buzzer (here)
    D6            LED_BUILTIN
    D8/D9/D10     SPI MOSI/SCK/MISO   -> future SPI display (GMG12864-06d)
    D11/D12       I2C SDA/SCL         -> barometer + IMU (here)
    D13/D14       Serial1 RX/TX       -> future GPS (NEO-6M / ATGM336H)
    onboard SD    SPI1 + SDCARD_SS_PIN, uses no external pins
    D0-D4, D7, A0-A6 free -> display CS/DC/RST, buttons

  Audio design:
    - Climb: discontinuous beeps. The pitch is a *note* proportional to the
      climb rate, i.e. the frequency is exponential in the vertical speed
      (CLIMB_M_S_PER_OCTAVE m/s of extra climb = one octave up). The note
      tracks the vertical speed continuously, so it slides up or down within
      a beep as the climb changes.
      The pause between beeps is constant; the beep length is decided when
      the beep starts (shorter the stronger the climb) and held for that
      whole beep, so the cadence speeds up as the climb gets stronger.
    - Sink: below SINK_ON a continuous low tone, also pitched exponentially
      (going down as the sink deepens) so it cannot be confused with a climb.
*/

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LSM6DS3.h>
#include <Adafruit_LSM6DS3TRC.h>

#if !defined(ARDUINO_ARCH_SAMD)
#error "This sketch targets the Arduino MKR Zero (SAMD21). Select Arduino SAMD Boards > Arduino MKR Zero."
#endif

// ----------------------------------------------------------------- hardware
const uint8_t BUZZER_PIN = 5;

Adafruit_BMP280     bmp;      // I2C barometer

// Breakouts sold as "LSM6DS3" are usually the LSM6DS3TR-C, which has a
// different WHO_AM_I. Both are tried, at both possible addresses, and the one
// that answers is used through the common base class.
Adafruit_LSM6DS3    imuDS3;
Adafruit_LSM6DS3TRC imuTRC;
Adafruit_LSM6DS    *imu   = nullptr;
bool                imuOk = false;

// ------------------------------------------------------------------- tuning
// --- sampling
constexpr float    SEA_LEVEL_HPA      = 1013.25f;  // only differences matter
constexpr uint16_t BARO_INTERVAL_MS   = 20;        // 50 Hz
constexpr uint32_t IMU_INTERVAL_US    = 9615;      // ~104 Hz, matches the ODR
constexpr float    GRAVITY            = 9.80665f;

// --- attitude (Mahony complementary filter, gyro corrected by gravity)
// KP pulls the estimate towards the accelerometer, KI learns the gyro bias.
//
// KP is deliberately very low. An accelerometer cannot tell gravity from
// acceleration, so under load - a banked turn above all - it points along the
// load vector rather than down. A low KP means the gyro carries the attitude
// through the manoeuvre and the accelerometer only trims it over ~20 s, by
// which time the load is gone. Whatever slow error survives is absorbed by
// the Kalman accelerometer-bias state, and the barometer anchors the result,
// so the usual penalty of a low KP - drift - does not appear: six minutes of
// continuous 35 degree turns with 0.004 rad/s of leftover gyro bias comes out
// the same at KP 0.05 as at KP 0.8. Roll-in error in a 40 degree turn:
// KP 0.05 -> 0.26 m/s, KP 0.3 -> 0.65 m/s, KP 0.8 -> 1.01 m/s.
constexpr float    MAHONY_KP          = 0.05f;
constexpr float    MAHONY_KI          = 0.02f;
constexpr uint16_t GYRO_CAL_MS        = 2000;      // hold still at power-up
constexpr float    GYRO_CAL_MAX_SPREAD = 0.06f;    // rad/s peak-to-peak allowed

// --- Kalman filter (altitude, vertical speed, accelerometer bias)
// BARO_VARIANCE  = barometer noise [m^2]. Bigger = smoother, laggier.
// ACCEL_VARIANCE = noise on the vertical acceleration [(m/s^2)^2]. In practice
//                  this is dominated by attitude error, not by the sensor.
// BIAS_VARIANCE  = how fast the accelerometer bias is allowed to wander
//                  [(m/s^2)^2 per second]. This state is what stops a small
//                  residual tilt error from integrating into a phantom climb.
constexpr float BARO_VARIANCE  = 0.10f;
constexpr float ACCEL_VARIANCE = 0.05f;
constexpr float BIAS_VARIANCE  = 0.010f;

// Rolling into a banked turn tilts the accelerometer onto the load vector,
// and until the bias state catches up that reads as a phantom climb. The
// accelerometer magnitude says when that is happening (it is no longer 1 g),
// so let the bias adapt faster exactly then, and stay quiet in level flight.
constexpr float BIAS_MANOEUVRE_GAIN = 30.0f;

// Without an IMU the filter degenerates to the old barometer-only one: the
// acceleration input becomes zero, the bias state is frozen, and the process
// noise has to stand in for the unknown acceleration.
constexpr float NO_IMU_ACCEL_VARIANCE = 0.60f;

// WHEN THE GPS ARRIVES: the textbook fix for the load-vector problem is to
// subtract the inertial acceleration from the gravity reference before the
// Mahony sees it, classically as  f - omega x v.  Do NOT do that with the
// gyro alone: omega x v assumes the flight path turns with the body, which is
// true in a turn but false when the pilot swings under the wing, and it made
// the pitch-oscillation error jump from 0.12 to 1.60 m/s and sextupled the
// noise in level flight when tried here. Done properly it needs the rotation
// of the GPS *velocity vector*, not of the body - i.e. differentiate the GPS
// velocity and subtract that. With a real velocity reference the roll-in
// error drops to about 0.17 m/s.

// --- thresholds in m/s. Each has a slightly lower release value (hysteresis)
// so the audio does not chatter when the vertical speed sits on the edge.
const float CLIMB_ON  =  0.10f;
const float CLIMB_OFF =  0.05f;
const float SINK_ON   = -2.50f;
const float SINK_OFF  = -2.20f;

// Climb rate that maps to the shortest beep / top of the pitch scale.
const float CLIMB_FULL = 8.0f;

// Pitch mapping: freq = BASE * 2^(v / M_S_PER_OCTAVE)  -> the note (not the
// frequency) is linear in the vertical speed.
const float CLIMB_BASE_HZ        = 550.0f;  // frequency extrapolated at 0 m/s
const float CLIMB_M_S_PER_OCTAVE = 3.5f;    // +3.5 m/s = one octave higher
const float SINK_BASE_HZ         = 400.0f;  // frequency extrapolated at 0 m/s
const float SINK_M_S_PER_OCTAVE  = 4.0f;    // -4.0 m/s = one octave lower
const float FREQ_MIN_HZ          = 90.0f;
const float FREQ_MAX_HZ          = 3500.0f;

// Smallest change worth re-programming the timer for. The tone is refreshed
// at the sample rate; a small deadband keeps a steady climb from restarting
// the waveform for a fraction of a hertz, which clicks.
const float TONE_DEADBAND_HZ = 1.0f;

// Rhythm. The pause is constant by design; the beep shrinks with the climb.
const uint16_t PAUSE_MS     = 70;   // silence between beeps, speed independent
const uint16_t BEEP_MS_SLOW = 380;  // beep length at CLIMB_ON
const uint16_t BEEP_MS_FAST = 70;   // beep length at CLIMB_FULL

// Serial plotter / monitor output at 115200 baud.
#define DEBUG_SERIAL 0

// -------------------------------------------------------------------- state
// Kalman state and its covariance, stored as the 6 unique elements of the
// symmetric 3x3 P = [[p00,p01,p02],[p01,p11,p12],[p02,p12,p22]].
float kAlt = 0.0f, kVel = 0.0f, kBias = 0.0f;
float p00 = 1.0f, p01 = 0.0f, p02 = 0.0f;
float p11 = 1.0f, p12 = 0.0f, p22 = 0.5f;

// Attitude quaternion (body -> earth) and the gyro bias the Mahony Ki term
// keeps learning in flight, on top of the startup calibration.
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float gyroIx = 0.0f, gyroIy = 0.0f, gyroIz = 0.0f;
float gyroBx = 0.0f, gyroBy = 0.0f, gyroBz = 0.0f;   // startup calibration

uint32_t lastBaroMs = 0;
uint32_t lastImuUs  = 0;

enum AudioState { AUDIO_SILENT, AUDIO_BEEP, AUDIO_PAUSE, AUDIO_SINK };
AudioState audioState = AUDIO_SILENT;
uint32_t   audioSince = 0;     // millis() when the current state started
uint16_t   beepLenMs  = 0;     // length of the beep in progress, latched at its start
float      toneHz     = 0.0f;  // frequency currently sent to the buzzer

// ------------------------------------------------------------ attitude core
// Initialise the attitude from a single accelerometer reading: the shortest
// rotation that takes the measured "up" onto the earth vertical. This is why
// the IMU can be mounted at any fixed angle.
void attitudeFromAccel(float ax, float ay, float az) {
  float n = sqrtf(ax * ax + ay * ay + az * az);
  if (n < 1e-6f) return;
  ax /= n; ay /= n; az /= n;

  float w = 1.0f + az;
  if (w < 1e-4f) {              // pointing straight down: 180 deg about x
    q0 = 0.0f; q1 = 1.0f; q2 = 0.0f; q3 = 0.0f;
    return;
  }
  q0 = w; q1 = ay; q2 = -ax; q3 = 0.0f;
  n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= n; q1 /= n; q2 /= n; q3 /= n;
}

// Mahony 6-DOF update. Gyro in rad/s, accelerometer in m/s^2 (any scale: it
// gets normalised, only its direction is used).
void attitudeUpdate(float gx, float gy, float gz,
                    float ax, float ay, float az, float dt) {
  float n = sqrtf(ax * ax + ay * ay + az * az);
  if (n > 1e-6f) {
    ax /= n; ay /= n; az /= n;

    // Earth "up" as the current attitude predicts it in body axes, i.e. the
    // third row of the body->earth rotation matrix.
    const float vx = 2.0f * (q1 * q3 - q0 * q2);
    const float vy = 2.0f * (q2 * q3 + q0 * q1);
    const float vz = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

    // Error = where gravity actually is, crossed with where we thought it was.
    const float ex = ay * vz - az * vy;
    const float ey = az * vx - ax * vz;
    const float ez = ax * vy - ay * vx;

    gyroIx += MAHONY_KI * ex * dt;
    gyroIy += MAHONY_KI * ey * dt;
    gyroIz += MAHONY_KI * ez * dt;

    gx += MAHONY_KP * ex + gyroIx;
    gy += MAHONY_KP * ey + gyroIy;
    gz += MAHONY_KP * ez + gyroIz;
  }

  // Integrate the quaternion, then renormalise.
  const float h  = 0.5f * dt;
  const float a0 = q0, a1 = q1, a2 = q2;
  q0 += (-a1 * gx - a2 * gy - q3 * gz) * h;
  q1 += ( a0 * gx + a2 * gz - q3 * gy) * h;
  q2 += ( a0 * gy - a1 * gz + q3 * gx) * h;
  q3 += ( a0 * gz + a1 * gy - a2 * gx) * h;

  n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (n > 1e-6f) { q0 /= n; q1 /= n; q2 /= n; q3 /= n; }
}

// Body acceleration projected onto the earth vertical, with gravity removed.
float verticalAccel(float ax, float ay, float az) {
  const float vx = 2.0f * (q1 * q3 - q0 * q2);
  const float vy = 2.0f * (q2 * q3 + q0 * q1);
  const float vz = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  return ax * vx + ay * vy + az * vz - GRAVITY;
}

// -------------------------------------------------------------- Kalman core
// States: altitude, vertical speed, accelerometer bias. The acceleration is a
// control input rather than a measurement, so the prediction runs at IMU rate
// and the barometer only corrects it.
void kalmanReset(float alt) {
  kAlt = alt; kVel = 0.0f; kBias = 0.0f;
  p00 = 1.0f; p01 = 0.0f; p02 = 0.0f;
  p11 = 1.0f; p12 = 0.0f; p22 = imuOk ? 0.5f : 0.0f;
}

// `manoeuvre` is how far the accelerometer magnitude is from 1 g, as a
// fraction: 0 in steady flight, ~0.3 in a 40 degree banked turn.
void kalmanPredict(float dt, float accel, float manoeuvre) {
  const float a = accel - kBias;
  const float h = 0.5f * dt * dt;

  kAlt += kVel * dt + a * h;
  kVel += a * dt;

  // M = F * P, with F = [[1, dt, -h], [0, 1, -dt], [0, 0, 1]]
  const float m00 = p00 + dt * p01 - h * p02;
  const float m01 = p01 + dt * p11 - h * p12;
  const float m02 = p02 + dt * p12 - h * p22;
  const float m11 = p11 - dt * p12;
  const float m12 = p12 - dt * p22;

  // P = M * F^T + Q
  const float dt2 = dt * dt;
  const float av  = imuOk ? ACCEL_VARIANCE : NO_IMU_ACCEL_VARIANCE;
  const float bv  = imuOk ? BIAS_VARIANCE * (1.0f + BIAS_MANOEUVRE_GAIN * manoeuvre)
                          : 0.0f;

  p00 = m00 + dt * m01 - h * m02 + av * dt2 * dt2 * 0.25f;
  p01 = m01 - dt * m02           + av * dt2 * dt  * 0.50f;
  p02 = m02;
  p11 = m11 - dt * m12           + av * dt2;
  p12 = m12;
  p22 = p22                      + bv * dt;
}

void kalmanUpdate(float altMeasured) {
  const float y  = altMeasured - kAlt;
  const float s  = p00 + BARO_VARIANCE;
  const float k0 = p00 / s;
  const float k1 = p01 / s;
  const float k2 = p02 / s;

  kAlt  += k0 * y;
  kVel  += k1 * y;
  kBias += k2 * y;

  const float o01 = p01, o02 = p02;
  p00 -= k0 * p00;
  p01 -= k0 * o01;
  p02 -= k0 * o02;
  p11 -= k1 * o01;
  p12 -= k1 * o02;
  p22 -= k2 * o02;
}

// ------------------------------------------------------------ audio mapping
// octaves above the base note -> frequency
float noteToFreq(float baseHz, float octaves) {
  return constrain(baseHz * powf(2.0f, octaves), FREQ_MIN_HZ, FREQ_MAX_HZ);
}

float climbFreq(float v) {
  return noteToFreq(CLIMB_BASE_HZ, constrain(v, 0.0f, CLIMB_FULL) / CLIMB_M_S_PER_OCTAVE);
}

float sinkFreq(float v) {
  return noteToFreq(SINK_BASE_HZ, constrain(v, -CLIMB_FULL, 0.0f) / SINK_M_S_PER_OCTAVE);
}

// Faster climb -> shorter beep. The pause stays PAUSE_MS, so the cadence rises.
uint16_t climbBeepLen(float v) {
  float t = (v - CLIMB_ON) / (CLIMB_FULL - CLIMB_ON);
  t = constrain(t, 0.0f, 1.0f);
  return (uint16_t)((float)BEEP_MS_SLOW + t * ((float)BEEP_MS_FAST - (float)BEEP_MS_SLOW));
}

// ------------------------------------------------------------ audio machine
// Retune the buzzer, ignoring changes too small to hear.
void setTone(float hz) {
  if (fabsf(hz - toneHz) < TONE_DEADBAND_HZ) return;
  toneHz = hz;
  tone(BUZZER_PIN, (unsigned int)hz);
}

void startBeep(uint32_t now, float v) {
  // Only the length is latched here: it stays fixed for this whole beep even
  // if the climb changes. The note itself keeps following the vertical speed.
  beepLenMs = climbBeepLen(v);
  toneHz    = 0.0f;               // force the first tone() of the beep
  setTone(climbFreq(v));
  audioState = AUDIO_BEEP;
  audioSince = now;
}

void startSink(uint32_t now, float v) {
  toneHz = 0.0f;
  setTone(sinkFreq(v));
  audioState = AUDIO_SINK;
  audioSince = now;
}

void stopAudio(uint32_t now) {
  noTone(BUZZER_PIN);
  toneHz     = 0.0f;
  audioState = AUDIO_SILENT;
  audioSince = now;
}

// Called every loop for the millisecond-accurate rhythm; `fresh` says whether
// a new vertical speed arrived, which is the only time the slide has anything
// new to say (and powf() is not worth running more often than that).
void updateAudio(uint32_t now, float v, bool fresh) {
  switch (audioState) {

    case AUDIO_SILENT:
      if (v <= SINK_ON)       startSink(now, v);
      else if (v >= CLIMB_ON) startBeep(now, v);
      break;

    case AUDIO_BEEP:
      if (v <= SINK_ON) {            // sink alarm wins, cut the beep short
        startSink(now, v);
      } else if (now - audioSince >= beepLenMs) {
        noTone(BUZZER_PIN);
        toneHz     = 0.0f;
        audioState = AUDIO_PAUSE;
        audioSince = now;
      } else if (fresh) {
        setTone(climbFreq(v));       // let the note slide with the climb rate
      }
      break;

    case AUDIO_PAUSE:
      if (v <= SINK_ON) {
        startSink(now, v);
      } else if (now - audioSince >= PAUSE_MS) {
        if (v >= CLIMB_OFF) startBeep(now, v);
        else                stopAudio(now);
      }
      break;

    case AUDIO_SINK:
      if (v >= SINK_OFF) stopAudio(now);
      else if (fresh)    setTone(sinkFreq(v));  // slides down as the sink deepens
      break;
  }
}

// -------------------------------------------------------------------- setup
void chirp(unsigned int hz, uint16_t ms, uint16_t gap = 120) {
  tone(BUZZER_PIN, hz, ms);
  delay(ms + gap);
  noTone(BUZZER_PIN);
}

void fatalBeep() {  // barometer missing: no vario is possible, say so forever
  while (true) {
    for (uint8_t i = 0; i < 3; i++) chirp(1200, 80, 120);
    delay(1000);
  }
}

// Try both chip variants at both addresses.
bool imuBegin() {
  for (uint8_t addr = 0x6A; addr <= 0x6B; addr++) {
    if (imuTRC.begin_I2C(addr)) { imu = &imuTRC; return true; }
    if (imuDS3.begin_I2C(addr)) { imu = &imuDS3; return true; }
  }
  return false;
}

// Average the gyro while the vario sits still, and start the attitude from
// the measured gravity direction. Retried if the thing is obviously moving.
void calibrateImu() {
  sensors_event_t a, g, t;

  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    float sx = 0, sy = 0, sz = 0;
    float loX = 1e9, hiX = -1e9, loY = 1e9, hiY = -1e9, loZ = 1e9, hiZ = -1e9;
    float ax = 0, ay = 0, az = 0;
    uint16_t n = 0;

    uint32_t start = millis();
    while (millis() - start < GYRO_CAL_MS) {
      imu->getEvent(&a, &g, &t);
      sx += g.gyro.x; sy += g.gyro.y; sz += g.gyro.z;
      loX = min(loX, g.gyro.x); hiX = max(hiX, g.gyro.x);
      loY = min(loY, g.gyro.y); hiY = max(hiY, g.gyro.y);
      loZ = min(loZ, g.gyro.z); hiZ = max(hiZ, g.gyro.z);
      ax += a.acceleration.x; ay += a.acceleration.y; az += a.acceleration.z;
      n++;
      delayMicroseconds(IMU_INTERVAL_US);
    }
    if (n == 0) return;

    const float spread = max(hiX - loX, max(hiY - loY, hiZ - loZ));
    if (spread <= GYRO_CAL_MAX_SPREAD || attempt == 4) {
      gyroBx = sx / n; gyroBy = sy / n; gyroBz = sz / n;
      attitudeFromAccel(ax / n, ay / n, az / n);
      return;
    }
    chirp(400, 60, 60);   // moved: nag and try again
    chirp(400, 60, 240);
  }
}

void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);   // USB CDC: do not wait for it, the vario must fly
#endif
  pinMode(BUZZER_PIN, OUTPUT);

  Wire.begin();
  Wire.setClock(400000);

  // Plain BMP280 boards answer at 0x76, the AHT20+BMP280 combo usually 0x77.
  if (!bmp.begin(0x76) && !bmp.begin(0x77)) fatalBeep();

  // Fast conversions: the filter does the smoothing, not the sensor. A long
  // STANDBY would make the loop re-read the same sample over and over.
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X1,   // temperature
                  Adafruit_BMP280::SAMPLING_X4,   // pressure
                  Adafruit_BMP280::FILTER_X4,
                  Adafruit_BMP280::STANDBY_MS_1);

  imuOk = imuBegin();
  if (imuOk) {
    imu->setAccelRange(LSM6DS_ACCEL_RANGE_4_G);      // a paraglider never gets near 4 g
    imu->setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    imu->setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu->setGyroDataRate(LSM6DS_RATE_104_HZ);
    delay(100);
    chirp(1000, 60, 60);            // "hold still"
    calibrateImu();
  } else {
    // Not fatal. The filter falls back to barometer-only, which is exactly
    // the behaviour of the previous version, just laggier than with the IMU.
    chirp(300, 200, 150);
    chirp(300, 200, 150);
  }

  delay(100);

  // Seed the filter with an average, so it does not start with a fake climb.
  float sum = 0.0f;
  for (uint8_t i = 0; i < 20; i++) {
    sum += bmp.readAltitude(SEA_LEVEL_HPA);
    delay(20);
  }
  kalmanReset(sum / 20.0f);

  lastBaroMs = millis();
  lastImuUs  = micros();

  chirp(800, 120, 60);              // ready
  chirp(1200, 120, 0);
}

// --------------------------------------------------------------------- loop
void loop() {
  bool fresh = false;

  // --- prediction, at IMU rate
  uint32_t nowUs = micros();
  if (nowUs - lastImuUs >= IMU_INTERVAL_US) {
    float dt = (nowUs - lastImuUs) * 1e-6f;   // unsigned math, rollover safe
    lastImuUs = nowUs;
    if (dt <= 0.0f || dt > 0.5f) dt = IMU_INTERVAL_US * 1e-6f;

    float aVert = 0.0f, manoeuvre = 0.0f;
    if (imuOk) {
      sensors_event_t a, g, t;
      imu->getEvent(&a, &g, &t);
      const float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
      attitudeUpdate(g.gyro.x - gyroBx, g.gyro.y - gyroBy, g.gyro.z - gyroBz,
                     ax, ay, az, dt);
      aVert     = verticalAccel(ax, ay, az);
      manoeuvre = fabsf(sqrtf(ax * ax + ay * ay + az * az) - GRAVITY) / GRAVITY;
    }
    kalmanPredict(dt, aVert, manoeuvre);
    fresh = true;
  }

  // --- correction, at barometer rate
  uint32_t nowMs = millis();
  if (nowMs - lastBaroMs >= BARO_INTERVAL_MS) {
    lastBaroMs = nowMs;
    kalmanUpdate(bmp.readAltitude(SEA_LEVEL_HPA));
    fresh = true;

#if DEBUG_SERIAL
    static uint8_t n = 0;
    if (++n >= 5) {                 // ~10 Hz
      n = 0;
      Serial.print(F("alt:"));    Serial.print(kAlt, 2);
      Serial.print(F("\tvel:"));  Serial.print(kVel, 2);
      Serial.print(F("\tbias:")); Serial.println(kBias, 3);
    }
#endif
  }

  updateAudio(millis(), kVel, fresh);
}
