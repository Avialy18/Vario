/*
  Variometer for paragliding — Arduino Nano + BMP280 + passive buzzer.

  Wiring (Nano):
    BMP280 VCC -> 3.3V (or 5V if the module has a regulator)
    BMP280 GND -> GND
    BMP280 SDA -> A4
    BMP280 SCL -> A5
    Buzzer     -> D5 through a 100 ohm resistor, other leg to GND
                               (passive buzzer / piezo; a magnetic buzzer
                                needs a transistor, the pin cannot drive it)

  Libraries: "Adafruit BMP280 Library" (+ Adafruit Unified Sensor), "NewTone".
  NewTone uses Timer1, so PWM on D9/D10 and the Servo library are unavailable.

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
#include <NewTone.h>

// ----------------------------------------------------------------- hardware
const uint8_t BUZZER_PIN = 5;

Adafruit_BMP280 bmp;  // I2C

// ------------------------------------------------------------------- tuning
// Sampling
const float    SEA_LEVEL_HPA = 1013.25f;  // only differences matter, not the absolute value
const uint16_t SAMPLE_MS     = 20;        // 50 Hz sensor read + filter update

// Kalman filter (altitude + vertical speed).
// ALT_VARIANCE   = how noisy the barometer is  [m^2]       -> bigger = smoother, laggier
// ACCEL_VARIANCE = how brisk the air is        [(m/s^2)^2] -> bigger = faster, twitchier
const float ALT_VARIANCE   = 0.10f;
const float ACCEL_VARIANCE = 0.60f;

// Thresholds in m/s. Each has a slightly lower release value (hysteresis) so
// the audio does not chatter when the vertical speed sits right on the edge.
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
// Kalman state: kAlt [m], kVel [m/s], P = [[p00, p01], [p01, p11]]
float kAlt = 0.0f, kVel = 0.0f;
float p00 = 0.5f, p01 = 0.0f, p11 = 0.5f;

uint32_t lastSampleMs = 0;
uint32_t lastStepUs   = 0;

enum AudioState { AUDIO_SILENT, AUDIO_BEEP, AUDIO_PAUSE, AUDIO_SINK };
AudioState audioState = AUDIO_SILENT;
uint32_t   audioSince = 0;     // millis() when the current state started
uint16_t   beepLenMs  = 0;     // length of the beep in progress, latched at its start
float      toneHz     = 0.0f;  // frequency currently sent to the buzzer

// -------------------------------------------------------------- Kalman core
void kalmanReset(float alt) {
  kAlt = alt;
  kVel = 0.0f;
  p00  = 0.5f;
  p01  = 0.0f;
  p11  = 0.5f;
}

// Constant-velocity model driven by unknown acceleration; the measurement is
// the barometric altitude. Two states, so it stays cheap on an ATmega328.
void kalmanStep(float dt, float altMeasured) {
  // --- predict
  kAlt += kVel * dt;

  const float dt2 = dt * dt;
  const float dt3 = dt2 * dt;
  const float dt4 = dt2 * dt2;

  p00 += dt * (2.0f * p01 + dt * p11) + 0.25f * ACCEL_VARIANCE * dt4;
  p01 += dt * p11                     + 0.50f * ACCEL_VARIANCE * dt3;
  p11 +=                                        ACCEL_VARIANCE * dt2;

  // --- update
  const float innovation = altMeasured - kAlt;
  const float s  = p00 + ALT_VARIANCE;
  const float k0 = p00 / s;
  const float k1 = p01 / s;

  kAlt += k0 * innovation;
  kVel += k1 * innovation;

  const float oldP00 = p00;
  const float oldP01 = p01;
  p00 -= k0 * oldP00;
  p01 -= k0 * oldP01;
  p11 -= k1 * oldP01;
}

// ------------------------------------------------------------ audio mapping
// octaves above the base note -> frequency
float noteToFreq(float baseHz, float octaves) {
  return constrain(baseHz * pow(2.0f, octaves), FREQ_MIN_HZ, FREQ_MAX_HZ);
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
  if (fabs(hz - toneHz) < TONE_DEADBAND_HZ) return;
  toneHz = hz;
  NewTone(BUZZER_PIN, (unsigned long)hz);
}

void startBeep(uint32_t now, float v) {
  // Only the length is latched here: it stays fixed for this whole beep even
  // if the climb changes. The note itself keeps following the vertical speed.
  beepLenMs = climbBeepLen(v);
  toneHz    = 0.0f;               // force the first NewTone() of the beep
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
  noNewTone(BUZZER_PIN);
  toneHz     = 0.0f;
  audioState = AUDIO_SILENT;
  audioSince = now;
}

// Called every loop for the millisecond-accurate rhythm; `fresh` says whether
// a new vertical speed arrived, which is the only time the slide has anything
// new to say (and pow() is not worth running more often than that).
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
        noNewTone(BUZZER_PIN);
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
void errorBeep() {  // sensor missing: three short chirps, forever
  while (true) {
    for (uint8_t i = 0; i < 3; i++) {
      NewTone(BUZZER_PIN, 1200, 80);
      delay(200);
    }
    delay(1000);
  }
}

void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
#endif
  pinMode(BUZZER_PIN, OUTPUT);

  Wire.begin();
  Wire.setClock(400000);  // keep the I2C reads well under the 20 ms budget

  // GY-BMP280 boards sit at 0x76 or 0x77 depending on the SDO pull.
  if (!bmp.begin(0x76) && !bmp.begin(0x77)) errorBeep();

  // Fast conversions: a vario needs a high refresh rate, the Kalman filter
  // does the smoothing. (A long STANDBY here would make the loop re-read the
  // same sample over and over.)
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X1,   // temperature
                  Adafruit_BMP280::SAMPLING_X4,   // pressure
                  Adafruit_BMP280::FILTER_X4,
                  Adafruit_BMP280::STANDBY_MS_1);

  delay(100);

  // Seed the filter with an average, so it does not start with a fake climb.
  float sum = 0.0f;
  for (uint8_t i = 0; i < 20; i++) {
    sum += bmp.readAltitude(SEA_LEVEL_HPA);
    delay(20);
  }
  kalmanReset(sum / 20.0f);

  lastSampleMs = millis();
  lastStepUs   = micros();

  NewTone(BUZZER_PIN, 800, 120);  // ready
  delay(180);
  NewTone(BUZZER_PIN, 1200, 120);
  delay(180);
  noNewTone(BUZZER_PIN);
}

// --------------------------------------------------------------------- loop
void loop() {
  uint32_t now = millis();
  bool fresh = false;

  if (now - lastSampleMs >= SAMPLE_MS) {
    lastSampleMs = now;
    fresh = true;

    uint32_t us = micros();
    float dt = (us - lastStepUs) * 1e-6f;   // unsigned math, rollover safe
    lastStepUs = us;
    if (dt <= 0.0f || dt > 0.5f) dt = SAMPLE_MS * 1e-3f;

    kalmanStep(dt, bmp.readAltitude(SEA_LEVEL_HPA));

#if DEBUG_SERIAL
    static uint8_t n = 0;
    if (++n >= 5) {                 // ~10 Hz
      n = 0;
      Serial.print(F("alt:"));   Serial.print(kAlt, 2);
      Serial.print(F("\tvel:")); Serial.println(kVel, 2);
    }
#endif
  }

  updateAudio(millis(), kVel, fresh);
}
