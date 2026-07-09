// Gradi Calibration wearable firmware
// Streams orientation data from the BNO08x (UART-RVC) and cooperates with the
// desktop controller over USB serial. First pass: terminal-only bucket output.

#include <Adafruit_BNO08x_RVC.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <I2S.h>

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / PI)
#endif

constexpr uint32_t SERIAL_BAUD = 921600;
constexpr uint32_t GUIDANCE_INTERVAL_DEFAULT_MS = 1500;
constexpr float ORIENTATION_SMOOTH_ALPHA = 0.2f;
constexpr float MOTION_START_THRESHOLD_DEG = 1.0f;
constexpr float MOTION_CONTINUE_THRESHOLD_DEG = 0.15f;
constexpr uint32_t MOTION_IDLE_TIMEOUT_MS = 7000;
constexpr uint32_t MOTION_START_DELAY_MS = 3000;
constexpr bool ALLOW_DIAGONAL_BUCKETS = false;
constexpr float MAG_DECLINATION_DEG = -8.28f;  // Update for install location.
constexpr float MIN_DIAGONAL_DEG = 12.0f;
constexpr float MICRO_ADJUST_DEG = 1.0f;
constexpr float DOCK_ALIGNMENT_THRESHOLD_DEG = 18.0f;
constexpr uint32_t DOCK_ALIGNMENT_DWELL_MS = 2500;
constexpr float DOCK_EXIT_THRESHOLD_DEG = 12.0f;
constexpr uint32_t DOCK_EXIT_MIN_MS = 500;
constexpr float NEAR_TARGET_ANGLE_DEG = 5.0f;

// --- Accel-based "returned to holder" detection (Change 2) ---
// Sole discriminator is rigid stillness on the accelerometer channel: a docked
// device is mechanically grounded (very low accel-magnitude sigma) while an
// unsupported held arm always exhibits tremor/sway (higher sigma). Pose match
// (gravity-referenced pitch/roll) narrows the window; yaw/magnetometer is NOT used.
// Decision: hasLeftDock && poseMatch && rigidStill, sustained for DOCK_CONFIRM_MS.
// NOTE: a per-sample "placement-transition arming" gate was tried and removed — a
// gentle, deliberate set-down (e.g. lowering the device into hanging holder arms)
// produces no sharp jolt, so the arming detector missed real placements and blocked
// legitimate docking. It also gave zero protection against the held-still collision
// (rigid stillness carries that). The collision defense rests entirely on the sigma
// separation, verified in situ (see TUNABLES.md / the LOG dock calibration step).
constexpr bool USE_ACCEL_DOCK_DETECT = true;       // false = legacy dock-cone path (instant reflash rollback)
// PLACEHOLDER — MUST be measured in situ with the installation audio playing; do not
// treat as final. Set between racked-device noise floor and quietest held-arm value
// only if they separate >=3x (see TUNABLES.md / the LOG dock calibration step).
constexpr float DOCK_RIGID_SIGMA = 0.04f;          // m/s^2 accel-magnitude sigma ceiling for "rigid"
                                                   // BENCH-PROVISIONAL: geo-mean of home floor (~0.0195)
                                                   // and held-still (~0.058) measurements, no show audio.
                                                   // RE-MEASURE in situ with the subwoofer playing before the show.
constexpr uint32_t DOCK_RIGID_WINDOW_MS = 1500;    // rolling window for the accel-sigma estimate
constexpr uint32_t DOCK_RVC_HZ = 100;              // nominal BNO08x RVC sample rate (for window sizing)
constexpr float DOCK_POSE_TOL_DEG = 10.0f;         // pitch/roll match tolerance vs. captured holder pose
constexpr uint32_t DOCK_CONFIRM_MS = 2000;         // sustain time with all terms true before idling
// Accel-magnitude sanity band (m/s^2): samples outside this are treated as corrupt
// RVC/UART frames and dropped before entering the sigma window, so a single garbage
// reading can't inflate sigma and spuriously break "rigid" for a full window. Gravity
// is ~9.8; gentle handling stays well inside. A real 22 g-ish glitch (seen in testing)
// or a dropout-to-zero falls outside and is rejected.
constexpr float DOCK_ACCEL_MIN_MS2 = 2.0f;
constexpr float DOCK_ACCEL_MAX_MS2 = 30.0f;
// UNVERIFIED fallbacks — replace with the measured holder pitch/roll (open question #3).
// Only used before the first TARE; the runtime dockPitch/dockRoll auto-capture at tare.
constexpr float DOCK_PITCH_FALLBACK_DEG = 0.0f;
constexpr float DOCK_ROLL_FALLBACK_DEG = 0.0f;

constexpr int PIN_I2S_BCLK = 3;      // D10 / P3 -> MAX98357A BCLK
constexpr int PIN_I2S_LRCLK = 4;     // D9  / P4 -> MAX98357A LRC
constexpr int PIN_I2S_DATA = 2;      // D8  / P2 -> MAX98357A DIN
constexpr int PIN_AMP_ENABLE = 6;    // D6  / P0 -> MAX98357A SD (active high)
constexpr size_t AUDIO_RING_SAMPLES      = 36864;  // mono samples, ~768 ms @ 48 kHz, 72 KB RAM
constexpr size_t AUDIO_PREBUFFER_SAMPLES = 19200;  // ~400 ms: gate before I2S output starts
constexpr size_t AUDIO_I2S_CHUNK_FRAMES  = 256;    // = AUDIO_DMA_WORDS; per-pass write granularity
constexpr uint8_t AUDIO_SAMPLE_BITS = 16;
constexpr uint8_t AUDIO_BYTES_PER_SAMPLE = AUDIO_SAMPLE_BITS / 8;
constexpr size_t AUDIO_DMA_BUFFERS = 16;           // ~85 ms DMA-level cushion (rides loop stalls)
constexpr size_t AUDIO_DMA_WORDS = 256;

static_assert(
    PIN_I2S_LRCLK == PIN_I2S_BCLK + 1,
    "RP2040 I2S requires LRCLK to be placed on BCLK+1.");

static I2S i2s(OUTPUT);

enum EqFilterType {
  EQ_TYPE_HIGHPASS,
  EQ_TYPE_PEAKING,
  EQ_TYPE_LOWPASS
};

struct EqStageConfig {
  EqFilterType type;
  float freqHz;
  float gainDb;
  float q;
};

struct BiquadFilter {
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
  float z1;
  float z2;
  bool active;
};

constexpr size_t EQ_FILTER_COUNT = 4;

const EqStageConfig EQ_STAGE_CONFIGS[EQ_FILTER_COUNT] = {
    {EQ_TYPE_HIGHPASS, 250.0f, 0.0f, 0.707f},
    {EQ_TYPE_PEAKING, 1000.0f, -3.0f, 1.0f},
    {EQ_TYPE_PEAKING, 5000.0f, -2.0f, 1.2f},
    {EQ_TYPE_LOWPASS, 7000.0f, 0.0f, 0.707f},
};

BiquadFilter eqFilters[EQ_FILTER_COUNT];

struct Vector3 {
  float east;
  float north;
  float up;
};

struct EulerAngles {
  float yaw;
  float pitch;
  float roll;
};

struct Mat3 {
  float m[3][3];
};

constexpr Vector3 V_ARM_SENS = {0.0f, 1.0f, 0.0f};               // Board +Y points down the wearer’s arm.
constexpr Vector3 DOCK_FORWARD_WORLD = {0.0f, 1.0f, 0.0f};       // Device points north in the dock pose.
constexpr Vector3 WORLD_UP = {0.0f, 0.0f, 1.0f};
const Mat3 MAT3_IDENTITY = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};

enum DeviceMode {
  MODE_IDLE,
  MODE_WAITING_FOR_START_ACK,
  MODE_GUIDING,
};

Adafruit_BNO08x_RVC rvc;

// State
DeviceMode deviceMode = MODE_IDLE;
bool targetValid = false;
Vector3 targetVector = {0.0f, 0.0f, 1.0f};
Vector3 smoothedOrientation = {0.0f, 0.0f, 1.0f};
bool hasOrientation = false;
EulerAngles lastAngles = {0.0f, 0.0f, 0.0f};
EulerAngles currentAngles = {0.0f, 0.0f, 0.0f};
bool hasAngles = false;
uint32_t lastMotionMs = 0;
uint32_t lastGuidanceMs = 0;
uint32_t stillnessStartMs = 0;
uint32_t guidanceIntervalMs = GUIDANCE_INTERVAL_DEFAULT_MS;
bool motionStartPending = false;
uint32_t motionStartQueuedMs = 0;
bool audioEnabled = false;
char currentPlanet[16] = "unknown";
Mat3 R_align = MAT3_IDENTITY;
size_t microCycleIndex = 0;
bool hasLeftDock = false;
uint32_t dockExitStartMs = 0;
uint32_t dockAlignStartMs = 0;

// Accel-based dock detection state (Change 2).
float dockPitch = DOCK_PITCH_FALLBACK_DEG;   // captured from live pitch at TARE (device docked then)
float dockRoll = DOCK_ROLL_FALLBACK_DEG;     // captured from live roll at TARE
float accelMag = 0.0f;                        // latest |accel| (m/s^2), gravity included
bool hasAccel = false;
constexpr size_t DOCK_ACCEL_WINDOW = (DOCK_RIGID_WINDOW_MS * DOCK_RVC_HZ) / 1000;  // ~150 samples
float accelWindow[DOCK_ACCEL_WINDOW] = {0.0f};
size_t accelWindowCount = 0;
size_t accelWindowHead = 0;
double accelSum = 0.0;
double accelSumSq = 0.0;
float accelSigma = 0.0f;
bool accelSigmaValid = false;                 // true only once the window is full (~1.5 s)
uint32_t dockConfirmStartMs = 0;              // start of the current sustained-docked window
uint32_t lastDockLogMs = 0;                   // throttle for the LOG dock diagnostic

enum AudioStreamMode {
  AUDIO_STREAM_IDLE,
  AUDIO_STREAM_RECEIVING,
  AUDIO_STREAM_DRAINING
};

AudioStreamMode audioStreamMode = AUDIO_STREAM_IDLE;
uint32_t audioSamplesRemaining = 0;
// Producer (USB) and consumer (I2S) both run in loop(); no ISR touches the ring,
// so plain size_t state is safe without volatile / critical sections.
int16_t  audioRing[AUDIO_RING_SAMPLES];
size_t   audioRingHead = 0;                 // next write index (from USB)
size_t   audioRingTail = 0;                 // next read index (to I2S)
size_t   audioRingCount = 0;                // samples currently buffered
bool     audioPlaybackStarted = false;
uint32_t audioTrueUnderflows = 0;           // true I2S starvation events (getUnderflow)
size_t   audioCushionMin = SIZE_MAX;        // low-water of ring + DMA fill after start (frames)
size_t   audioDmaCapacityBytes = 0;         // availableForWrite() baseline captured after begin()
uint32_t audioStartMs = 0;                  // millis() when first payload byte of a clip arrived
bool     audioStartMsSet = false;
uint32_t audioPrebufMs = 0;                 // wall time from first byte to prebuffer gate open
bool audioOutputConfigured = false;
uint32_t audioCurrentSampleRateHz = 0;

float clampf(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

float angularDifference(float a, float b) {
  float diff = a - b;
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

Vector3 normalize(const Vector3 &v) {
  float mag = sqrtf(v.east * v.east + v.north * v.north + v.up * v.up);
  if (mag <= 1e-6f) {
    return {0.0f, 0.0f, 0.0f};
  }
  return {v.east / mag, v.north / mag, v.up / mag};
}

float dot(const Vector3 &a, const Vector3 &b) {
  return a.east * b.east + a.north * b.north + a.up * b.up;
}

Vector3 cross(const Vector3 &a, const Vector3 &b) {
  return {a.north * b.up - a.up * b.north, a.up * b.east - a.east * b.up, a.east * b.north - a.north * b.east};
}

float angleBetween(const Vector3 &a, const Vector3 &b) {
  float d = clampf(dot(normalize(a), normalize(b)), -1.0f, 1.0f);
  return acosf(d) * 180.0f / PI;
}

Mat3 matMul(const Mat3 &A, const Mat3 &B) {
  Mat3 C = {{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      C.m[i][j] = A.m[i][0] * B.m[0][j] + A.m[i][1] * B.m[1][j] + A.m[i][2] * B.m[2][j];
    }
  }
  return C;
}

Vector3 matMulVec(const Mat3 &R, const Vector3 &v) {
  return {
      R.m[0][0] * v.east + R.m[0][1] * v.north + R.m[0][2] * v.up,
      R.m[1][0] * v.east + R.m[1][1] * v.north + R.m[1][2] * v.up,
      R.m[2][0] * v.east + R.m[2][1] * v.north + R.m[2][2] * v.up,
  };
}

Mat3 axisAngleMatrix(const Vector3 &axisIn, float angleRad) {
  Vector3 axis = normalize(axisIn);
  float x = axis.east;
  float y = axis.north;
  float z = axis.up;
  float c = cosf(angleRad);
  float s = sinf(angleRad);
  float t = 1.0f - c;
  Mat3 R = {{{t * x * x + c, t * x * y - s * z, t * x * z + s * y},
             {t * x * y + s * z, t * y * y + c, t * y * z - s * x},
             {t * x * z - s * y, t * y * z + s * x, t * z * z + c}}};
  return R;
}

Mat3 rotmatZYX_deg(float yawDeg, float pitchDeg, float rollDeg) {
  float cy = cosf(yawDeg * DEG_TO_RAD);
  float sy = sinf(yawDeg * DEG_TO_RAD);
  float cp = cosf(pitchDeg * DEG_TO_RAD);
  float sp = sinf(pitchDeg * DEG_TO_RAD);
  float cr = cosf(rollDeg * DEG_TO_RAD);
  float sr = sinf(rollDeg * DEG_TO_RAD);

  Mat3 Rz = {{{cy, -sy, 0.0f}, {sy, cy, 0.0f}, {0.0f, 0.0f, 1.0f}}};
  Mat3 Ry = {{{cp, 0.0f, sp}, {0.0f, 1.0f, 0.0f}, {-sp, 0.0f, cp}}};
  Mat3 Rx = {{{1.0f, 0.0f, 0.0f}, {0.0f, cr, -sr}, {0.0f, sr, cr}}};

  return matMul(Rz, matMul(Ry, Rx));
}

Mat3 rotationBetween(const Vector3 &from, const Vector3 &to) {
  Vector3 a = normalize(from);
  Vector3 b = normalize(to);
  Vector3 v = cross(a, b);
  float s = sqrtf(dot(v, v));
  float c = clampf(dot(a, b), -1.0f, 1.0f);
  if (s < 1e-6f) {
    if (c > 0.999999f) {
      return MAT3_IDENTITY;
    }
    Vector3 axis = cross(a, {1.0f, 0.0f, 0.0f});
    if (fabsf(axis.east) + fabsf(axis.north) + fabsf(axis.up) < 1e-6f) {
      axis = cross(a, {0.0f, 1.0f, 0.0f});
    }
    return axisAngleMatrix(axis, PI);
  }
  Vector3 axis = {v.east / s, v.north / s, v.up / s};
  float angle = atan2f(s, c);
  return axisAngleMatrix(axis, angle);
}

void resetEqFilters() {
  for (size_t i = 0; i < EQ_FILTER_COUNT; ++i) {
    eqFilters[i].z1 = 0.0f;
    eqFilters[i].z2 = 0.0f;
  }
}

void configureBiquad(BiquadFilter *filter, EqFilterType type, float freqHz, float gainDb, float q, float sampleRate) {
  if (!filter || sampleRate <= 0.0f || freqHz <= 0.0f || freqHz >= sampleRate * 0.48f) {
    if (filter) {
      filter->active = false;
      filter->b0 = 1.0f;
      filter->b1 = 0.0f;
      filter->b2 = 0.0f;
      filter->a1 = 0.0f;
      filter->a2 = 0.0f;
      filter->z1 = 0.0f;
      filter->z2 = 0.0f;
    }
    return;
  }

  float omega = 2.0f * PI * (freqHz / sampleRate);
  float sinOmega = sinf(omega);
  float cosOmega = cosf(omega);
  float alpha = sinOmega / (2.0f * q);

  float b0 = 1.0f;
  float b1 = 0.0f;
  float b2 = 0.0f;
  float a0 = 1.0f;
  float a1 = 0.0f;
  float a2 = 0.0f;

  if (type == EQ_TYPE_HIGHPASS) {
    b0 = (1.0f + cosOmega) * 0.5f;
    b1 = -(1.0f + cosOmega);
    b2 = (1.0f + cosOmega) * 0.5f;
    a0 = 1.0f + alpha;
    a1 = -2.0f * cosOmega;
    a2 = 1.0f - alpha;
  } else if (type == EQ_TYPE_LOWPASS) {
    b0 = (1.0f - cosOmega) * 0.5f;
    b1 = 1.0f - cosOmega;
    b2 = (1.0f - cosOmega) * 0.5f;
    a0 = 1.0f + alpha;
    a1 = -2.0f * cosOmega;
    a2 = 1.0f - alpha;
  } else if (type == EQ_TYPE_PEAKING) {
    float A = powf(10.0f, gainDb / 40.0f);
    b0 = 1.0f + alpha * A;
    b1 = -2.0f * cosOmega;
    b2 = 1.0f - alpha * A;
    a0 = 1.0f + alpha / A;
    a1 = -2.0f * cosOmega;
    a2 = 1.0f - alpha / A;
  }

  filter->b0 = b0 / a0;
  filter->b1 = b1 / a0;
  filter->b2 = b2 / a0;
  filter->a1 = a1 / a0;
  filter->a2 = a2 / a0;
  filter->z1 = 0.0f;
  filter->z2 = 0.0f;
  filter->active = true;
}

void configureEqFilters(uint32_t sampleRate) {
  float sampleRateF = static_cast<float>(sampleRate);
  for (size_t i = 0; i < EQ_FILTER_COUNT; ++i) {
    const EqStageConfig &cfg = EQ_STAGE_CONFIGS[i];
    configureBiquad(&eqFilters[i], cfg.type, cfg.freqHz, cfg.gainDb, cfg.q, sampleRateF);
  }
}

float processEqSample(float input) {
  float x = input;
  for (size_t i = 0; i < EQ_FILTER_COUNT; ++i) {
    BiquadFilter &stage = eqFilters[i];
    if (!stage.active) {
      continue;
    }
    float y = stage.b0 * x + stage.z1;
    stage.z1 = stage.b1 * x + stage.z2 - stage.a1 * y;
    stage.z2 = stage.b2 * x - stage.a2 * y;
    x = y;
  }
  return x;
}

bool configureAudioOutput(uint32_t sampleRate) {
  if (sampleRate == 0) {
    return false;
  }

  if (audioOutputConfigured && audioCurrentSampleRateHz == sampleRate) {
    return true;
  }

  if (audioOutputConfigured) {
    i2s.end();
    audioOutputConfigured = false;
    audioCurrentSampleRateHz = 0;
  }

  i2s.setBCLK(PIN_I2S_BCLK);
  i2s.setDATA(PIN_I2S_DATA);
  i2s.setBitsPerSample(AUDIO_SAMPLE_BITS);
  i2s.setBuffers(AUDIO_DMA_BUFFERS, AUDIO_DMA_WORDS);
  // NB: i2s.setSysClk() is called once in setup() before the UARTs start, not here,
  // to avoid re-parenting the system clock mid-session (see proposal §5.3).

  if (!i2s.begin(static_cast<long>(sampleRate))) {
    Serial.println(F("LOG audio_i2s_begin_failed"));
    return false;
  }

  audioOutputConfigured = true;
  audioCurrentSampleRateHz = sampleRate;
  audioDmaCapacityBytes = (size_t)i2s.availableForWrite();  // empty-DMA baseline (F-1)
  return true;
}

void resetAudioStreamState() {
  audioStreamMode = AUDIO_STREAM_IDLE;
  audioSamplesRemaining = 0;
  audioRingHead = 0;
  audioRingTail = 0;
  audioRingCount = 0;
  audioPlaybackStarted = false;
  audioTrueUnderflows = 0;
  audioCushionMin = SIZE_MAX;
  audioStartMs = 0;
  audioStartMsSet = false;
  audioPrebufMs = 0;
}

// Receive half: drain USB PCM bytes into the ring. When the ring is full we simply
// stop reading, so TinyUSB NAKs the host (natural flow control, no data loss). Returns
// true if any samples were consumed. When output is not configured (skip case) the
// bytes are discarded rather than buffered, so the ring stays empty and drains cleanly.
bool pumpAudioReceive() {
  if (audioStreamMode != AUDIO_STREAM_RECEIVING) {
    return false;
  }

  bool consumed = false;
  while (audioSamplesRemaining > 0 &&
         audioRingCount < AUDIO_RING_SAMPLES &&
         Serial.available() >= AUDIO_BYTES_PER_SAMPLE) {
    int first = Serial.read();
    int second = Serial.read();
    if (first < 0 || second < 0) {
      break;
    }
    if (!audioStartMsSet) {   // F-5: mark arrival of the clip's first payload byte
      audioStartMs = millis();
      audioStartMsSet = true;
    }
    if (audioOutputConfigured) {
      audioRing[audioRingHead] = static_cast<int16_t>(first | (second << 8));
      audioRingHead = (audioRingHead + 1) % AUDIO_RING_SAMPLES;
      audioRingCount++;
    }
    audioSamplesRemaining--;
    consumed = true;
  }

  return consumed;
}

// Playback half: feed I2S strictly non-blocking via availableForWrite(). Gated by a
// prebuffer so output only starts once the ring holds a comfortable cushion (or the
// whole clip, for very short clips). Counts true starvation via getUnderflow().
void pumpAudioPlayback() {
  if (!audioOutputConfigured) return;

  if (!audioPlaybackStarted) {
    bool enough = audioRingCount >= AUDIO_PREBUFFER_SAMPLES;
    bool clipComplete = (audioSamplesRemaining == 0);   // short clip fully received
    if (!enough && !clipComplete) return;
    audioPlaybackStarted = true;
    audioPrebufMs = audioStartMsSet ? (millis() - audioStartMs) : 0;   // F-5 throughput probe
    (void)i2s.getUnderflow();   // clear the flag: DMA was playing silence pre-start
  }

  static int16_t stereoScratch[AUDIO_I2S_CHUNK_FRAMES * 2];
  while (audioRingCount > 0) {
    size_t framesFree = (size_t)i2s.availableForWrite() / 4;  // bytes -> stereo frames
    if (framesFree == 0) break;
    size_t frames = min(min(framesFree, audioRingCount), AUDIO_I2S_CHUNK_FRAMES);
    for (size_t i = 0, j = 0; i < frames; ++i) {
      int16_t s = audioRing[audioRingTail];
      audioRingTail = (audioRingTail + 1) % AUDIO_RING_SAMPLES;
      stereoScratch[j++] = s;
      stereoScratch[j++] = s;
    }
    audioRingCount -= frames;
    i2s.write((const uint8_t *)stereoScratch, frames * 4);
    // availableForWrite() >= frames*4 was checked, so this cannot come up short.
  }

  if (audioPlaybackStarted) {
    if (i2s.getUnderflow()) audioTrueUnderflows++;               // TRUE starvation events
    // Real cushion = ring + queued DMA (F-4). Skip the natural end-of-clip drain so it
    // doesn't drag the low-water mark to 0 once the clip is fully consumed.
    if (audioSamplesRemaining > 0 || audioRingCount > 0) {
      size_t dmaFillFrames = (audioDmaCapacityBytes - (size_t)i2s.availableForWrite()) / 4;
      size_t cushionFrames = audioRingCount + dmaFillFrames;
      if (cushionFrames < audioCushionMin) audioCushionMin = cushionFrames;
    }
  }
}

// Called from loop() once the ring has fully drained while DRAINING.
void finishAudioDrain() {
  Serial.println(F("LOG audio_end"));
  Serial.print(F("LOG audio_stats underruns="));
  Serial.print(audioTrueUnderflows);
  Serial.print(F(" mincushion="));
  Serial.print(audioCushionMin == SIZE_MAX ? 0UL : (unsigned long)audioCushionMin);
  Serial.print(F(" prebuf_ms="));
  Serial.println((unsigned long)audioPrebufMs);
  resetAudioStreamState();
}

void handleAudioStartCommand(uint32_t sampleRate, uint32_t frameCount, const char *label) {
  if (audioStreamMode == AUDIO_STREAM_DRAINING && audioRingCount > 0) {
    // Host started the next clip before the previous tail finished draining. Dropping
    // the tail is the correct degradation; the host end-sleep (§4.2) makes it rare.
    Serial.print(F("LOG audio_drain_cut remaining="));
    Serial.println((unsigned long)audioRingCount);
  }
  resetAudioStreamState();
  if (frameCount == 0 || sampleRate == 0) {
    return;
  }

  bool shouldPlay = audioEnabled && configureAudioOutput(sampleRate);
  if (!shouldPlay) {
    if (audioOutputConfigured) {
      i2s.end();
      audioCurrentSampleRateHz = 0;
    }
    audioOutputConfigured = false;
  }

  audioSamplesRemaining = frameCount;
  audioStreamMode = AUDIO_STREAM_RECEIVING;
  if (shouldPlay) {
    Serial.print(F("LOG audio_start "));
    if (label && *label) {
      Serial.println(label);
    } else {
      Serial.println(sampleRate);
    }
  } else {
    Serial.println(F("LOG audio_skip"));
  }
}

void handleAudioEndCommand() {
  if (audioStreamMode != AUDIO_STREAM_RECEIVING) {
    return;
  }
  // Pad the ring up to the next full I2S chunk so a sub-chunk tail is not stuck forever
  // in a partial DMA buffer (§1.3 / §3.5 step 1). Only meaningful when we're buffering.
  if (audioOutputConfigured) {
    size_t remainder = audioRingCount % AUDIO_I2S_CHUNK_FRAMES;
    if (remainder != 0) {
      size_t pad = AUDIO_I2S_CHUNK_FRAMES - remainder;
      for (size_t i = 0; i < pad && audioRingCount < AUDIO_RING_SAMPLES; ++i) {
        audioRing[audioRingHead] = 0;
        audioRingHead = (audioRingHead + 1) % AUDIO_RING_SAMPLES;
        audioRingCount++;
      }
    }
  }
  audioStreamMode = AUDIO_STREAM_DRAINING;
}

void sendState(const char *state) {
  Serial.print(F("STATE "));
  Serial.println(state);
}

void sendBucket(const char *label) {
  Serial.print(F("BUCKET "));
  Serial.println(label);
}

void sendOrientation(const Vector3 &v, const EulerAngles &angles) {
  Serial.print(F("ORIENT "));
  Serial.print(angles.yaw, 1);
  Serial.print(' ');
  Serial.print(angles.pitch, 1);
  Serial.print(' ');
  Serial.print(angles.roll, 1);
  Serial.print(' ');
  Serial.print(v.east, 4);
  Serial.print(' ');
  Serial.print(v.north, 4);
  Serial.print(' ');
  Serial.println(v.up, 4);
}

void setMode(DeviceMode mode) {
  deviceMode = mode;
  if (mode != MODE_IDLE) {
    stillnessStartMs = 0;
  }
  if (mode == MODE_IDLE) {
    sendState("idle");
  } else if (mode == MODE_WAITING_FOR_START_ACK) {
    sendState("motion_start");
  } else if (mode == MODE_GUIDING) {
    sendState("guiding");
  }
}

void transitionToIdle() {
  targetValid = false;
  currentPlanet[0] = '\0';
  guidanceIntervalMs = GUIDANCE_INTERVAL_DEFAULT_MS;
  lastGuidanceMs = 0;
  stillnessStartMs = 0;
  microCycleIndex = 0;
  motionStartPending = false;
  motionStartQueuedMs = 0;
  hasLeftDock = false;
  dockExitStartMs = 0;
  dockAlignStartMs = 0;
  dockConfirmStartMs = 0;
  setMode(MODE_IDLE);
}

Vector3 eulerToForward(const EulerAngles &angles) {
  float yawTrue = MAG_DECLINATION_DEG - angles.yaw;
  Mat3 R_world_from_sensor = rotmatZYX_deg(yawTrue, angles.pitch, angles.roll);
  Vector3 forwardWorld = matMulVec(R_world_from_sensor, V_ARM_SENS);
  return normalize(forwardWorld);
}

bool readOrientation(EulerAngles *anglesOut, Vector3 *forwardOut) {
  BNO08x_RVC_Data data;
  if (!rvc.read(&data)) {
    return false;
  }
  // COMPILE-TIME CHECK (open question #6): the Adafruit BNO08x_RVC library was not
  // installed on the authoring machine, so these field names are unverified there.
  // If the build errors, adjust these three reads to the library's accel field names.
  accelMag = sqrtf(data.x_accel * data.x_accel + data.y_accel * data.y_accel +
                   data.z_accel * data.z_accel);
  hasAccel = true;
  EulerAngles angles = {data.yaw, data.pitch, data.roll};
  Vector3 forward = eulerToForward(angles);
  Vector3 corrected = matMulVec(R_align, forward);
  corrected = normalize(corrected);
  if (!hasOrientation) {
    smoothedOrientation = corrected;
    hasOrientation = true;
  } else {
    smoothedOrientation.east =
        ORIENTATION_SMOOTH_ALPHA * corrected.east + (1.0f - ORIENTATION_SMOOTH_ALPHA) * smoothedOrientation.east;
    smoothedOrientation.north =
        ORIENTATION_SMOOTH_ALPHA * corrected.north + (1.0f - ORIENTATION_SMOOTH_ALPHA) * smoothedOrientation.north;
    smoothedOrientation.up =
        ORIENTATION_SMOOTH_ALPHA * corrected.up + (1.0f - ORIENTATION_SMOOTH_ALPHA) * smoothedOrientation.up;
    smoothedOrientation = normalize(smoothedOrientation);
  }
  if (anglesOut) {
    *anglesOut = angles;
  }
  if (forwardOut) {
    *forwardOut = smoothedOrientation;
  }
  return true;
}

void handleMotion(const EulerAngles &angles, uint32_t nowMs) {
  if (!hasAngles) {
    lastAngles = angles;
    currentAngles = angles;
    hasAngles = true;
    lastMotionMs = nowMs;
    stillnessStartMs = nowMs;
    return;
  }
  float yawDiff = fabsf(angularDifference(angles.yaw, lastAngles.yaw));
  float pitchDiff = fabsf(angularDifference(angles.pitch, lastAngles.pitch));
  float rollDiff = fabsf(angularDifference(angles.roll, lastAngles.roll));
  float motionScore = fmaxf(fmaxf(yawDiff, pitchDiff), rollDiff);

  if (motionScore >= MOTION_CONTINUE_THRESHOLD_DEG) {
    lastMotionMs = nowMs;
    stillnessStartMs = 0;
  } else if (stillnessStartMs == 0) {
    stillnessStartMs = nowMs;
  }

  if (deviceMode == MODE_IDLE) {
    // Motion-start keys on the fused orientation delta, which is jittered by vibration
    // (the device's own outro playing right after docking, or a room subwoofer coupling
    // into a compliant holder) — that can false-start a phantom session while the device
    // sits docked, and because it never left the dock it can't self-recover. Two guards,
    // both required for a real hands-on don:
    //  - accel magnitude is rotation-invariant and immune to that vibration, so require
    //    genuine accel motion (NOT rigid) — a real don always spikes accel well past this.
    //  - never arm while the device's own speaker is playing.
    bool rigid = accelSigmaValid && accelSigma < DOCK_RIGID_SIGMA;
    bool audioPlaying = (audioStreamMode != AUDIO_STREAM_IDLE);
    if (rigid || audioPlaying) {
      motionStartPending = false;
      motionStartQueuedMs = 0;
    } else {
      if (!motionStartPending && motionScore >= MOTION_START_THRESHOLD_DEG) {
        // Give the wearer time to don the device before signaling the host.
        motionStartPending = true;
        motionStartQueuedMs = nowMs;
      }
      if (motionStartPending && (nowMs - motionStartQueuedMs) >= MOTION_START_DELAY_MS) {
        setMode(MODE_WAITING_FOR_START_ACK);
        motionStartPending = false;
        motionStartQueuedMs = 0;
        lastMotionMs = nowMs;
        stillnessStartMs = 0;
      }
    }
  } else {
    motionStartPending = false;
    motionStartQueuedMs = 0;
  }

  lastAngles = angles;
  currentAngles = angles;
}

const char *selectBucket(const Vector3 &forward, const Vector3 &target) {
  Vector3 f = normalize(forward);
  Vector3 t = normalize(target);

  Vector3 right = normalize(cross(f, WORLD_UP));
  float rightMagSq = dot(right, right);
  if (rightMagSq < 1e-6f) {
    right = normalize(cross(f, {0.0f, 1.0f, 0.0f}));
    if (dot(right, right) < 1e-6f) {
      right = {1.0f, 0.0f, 0.0f};
    }
  }
  Vector3 upLocal = normalize(cross(right, f));

  float targetRight = dot(t, right);
  float targetUp = dot(t, upLocal);
  float targetForward = dot(t, f);

  float yawErrorDeg = atan2f(targetRight, targetForward) * RAD_TO_DEG;
  float pitchErrorDeg = atan2f(targetUp, targetForward) * RAD_TO_DEG;

  float absYaw = fabsf(yawErrorDeg);
  float absPitch = fabsf(pitchErrorDeg);

  if (absYaw < MICRO_ADJUST_DEG && absPitch < MICRO_ADJUST_DEG) {
    static const char *MICRO_PROMPTS[] = {"up", "right", "down", "left"};
    const char *bucket = MICRO_PROMPTS[microCycleIndex % (sizeof(MICRO_PROMPTS) / sizeof(MICRO_PROMPTS[0]))];
    microCycleIndex = (microCycleIndex + 1) % (sizeof(MICRO_PROMPTS) / sizeof(MICRO_PROMPTS[0]));
    return bucket;
  }

  if (ALLOW_DIAGONAL_BUCKETS && absYaw >= MIN_DIAGONAL_DEG && absPitch >= MIN_DIAGONAL_DEG) {
    if (yawErrorDeg >= 0.0f) {
      return (pitchErrorDeg >= 0.0f) ? "up_right" : "down_right";
    }
    return (pitchErrorDeg >= 0.0f) ? "up_left" : "down_left";
  }

  if (absYaw >= absPitch) {
    return (yawErrorDeg >= 0.0f) ? "right" : "left";
  }
  return (pitchErrorDeg >= 0.0f) ? "up" : "down";
}

void guidanceTick(uint32_t nowMs) {
  if (deviceMode != MODE_GUIDING || !targetValid || !hasOrientation) {
    return;
  }
  if (nowMs - lastGuidanceMs < guidanceIntervalMs) {
    return;
  }
  lastGuidanceMs = nowMs;

  sendOrientation(smoothedOrientation, currentAngles);
  const char *bucket = selectBucket(smoothedOrientation, targetVector);
  sendBucket(bucket);
}

void updateDockTracking(uint32_t nowMs) {
  if (!hasOrientation) {
    dockExitStartMs = 0;
    dockAlignStartMs = 0;
    return;
  }

  float dockAngle = angleBetween(smoothedOrientation, DOCK_FORWARD_WORLD);
  bool beyondExit = dockAngle >= DOCK_EXIT_THRESHOLD_DEG;
  bool inDockCone = dockAngle <= DOCK_ALIGNMENT_THRESHOLD_DEG;

  if (!hasLeftDock) {
    if (beyondExit) {
      if (dockExitStartMs == 0) {
        dockExitStartMs = nowMs;
      } else if ((nowMs - dockExitStartMs) >= DOCK_EXIT_MIN_MS) {
        hasLeftDock = true;
      }
    } else {
      dockExitStartMs = 0;
    }
  }

  if (inDockCone) {
    if (dockAlignStartMs == 0) {
      dockAlignStartMs = nowMs;
    }
  } else {
    dockAlignStartMs = 0;
  }
}

// Accel-based rigid-stillness detector (Change 2). Runs once per RVC sample and
// maintains a rolling accel-magnitude window (O(1) update) whose sigma is the "rigid"
// discriminator. Corrupt samples outside the gravity-plausible band are dropped so a
// single garbage RVC/UART frame can't inflate sigma and break "rigid" for a window.
void updateAccelDetect() {
  if (!hasAccel) {
    return;
  }
  float mag = accelMag;
  if (mag < DOCK_ACCEL_MIN_MS2 || mag > DOCK_ACCEL_MAX_MS2) {
    return;  // implausible magnitude → corrupt frame; leave the window untouched
  }

  // Rolling window sum / sum-of-squares (subtract the evicted sample when full).
  if (accelWindowCount == DOCK_ACCEL_WINDOW) {
    float old = accelWindow[accelWindowHead];
    accelSum -= old;
    accelSumSq -= (double)old * old;
  } else {
    accelWindowCount++;
  }
  accelWindow[accelWindowHead] = mag;
  accelSum += mag;
  accelSumSq += (double)mag * mag;
  accelWindowHead = (accelWindowHead + 1) % DOCK_ACCEL_WINDOW;

  if (accelWindowCount == DOCK_ACCEL_WINDOW) {
    double mean = accelSum / (double)accelWindowCount;
    double var = accelSumSq / (double)accelWindowCount - mean * mean;
    if (var < 0.0) {
      var = 0.0;  // guard tiny negatives from float round-off
    }
    accelSigma = (float)sqrt(var);
    accelSigmaValid = true;
  } else {
    accelSigmaValid = false;  // require a full ~1.5 s window before trusting sigma
  }
}

void handleSerialLine(char *line) {
  if (strncmp(line, "START", 5) == 0) {
    audioEnabled = false;
    guidanceIntervalMs = GUIDANCE_INTERVAL_DEFAULT_MS;
    currentPlanet[0] = '\0';

    char *cursor = line + 5;
    while (*cursor == ' ') cursor++;
    char *token = strtok(cursor, " ");
    while (token != nullptr) {
      char *equals = strchr(token, '=');
      if (equals) {
        *equals = '\0';
        const char *key = token;
        const char *value = equals + 1;
        if (strcmp(key, "planet") == 0) {
          strncpy(currentPlanet, value, sizeof(currentPlanet) - 1);
          currentPlanet[sizeof(currentPlanet) - 1] = '\0';
        } else if (strcmp(key, "audio") == 0) {
          audioEnabled = atoi(value) != 0;
        } else if (strcmp(key, "cadence") == 0) {
          float cadence = atof(value);
          if (cadence > 0.1f) {
            guidanceIntervalMs = (uint32_t)(cadence * 1000.0f);
          }
        }
      }
      token = strtok(nullptr, " ");
    }
    if (deviceMode == MODE_WAITING_FOR_START_ACK || deviceMode == MODE_IDLE) {
      setMode(MODE_GUIDING);
    } else {
      setMode(MODE_GUIDING);
    }
    microCycleIndex = 0;
    lastMotionMs = millis();
    stillnessStartMs = 0;
    return;
  }

  if (strncmp(line, "TARGET", 6) == 0) {
    float east = 0.0f;
    float north = 0.0f;
    float up = 0.0f;
    if (sscanf(line + 6, "%f %f %f", &east, &north, &up) == 3) {
      targetVector = normalize({east, north, up});
      targetValid = true;
    }
    return;
  }

  if (strncmp(line, "END", 3) == 0) {
    transitionToIdle();
    return;
  }

  if (strncmp(line, "TARE", 4) == 0) {
    if (hasAngles) {
      Vector3 forwardNow = eulerToForward(currentAngles);
      R_align = rotationBetween(forwardNow, DOCK_FORWARD_WORLD);
      Vector3 corrected = matMulVec(R_align, forwardNow);
      smoothedOrientation = normalize(corrected);
      hasOrientation = true;
      microCycleIndex = 0;
      // Device is physically docked at tare: capture the holder's gravity-referenced
      // pitch/roll for the drift-free pose-match term (Change 2).
      dockPitch = currentAngles.pitch;
      dockRoll = currentAngles.roll;
      Serial.println(F("EVENT tared"));
    } else {
      Serial.println(F("EVENT tare_wait"));
    }
    return;
  }

  if (strncmp(line, "IDLE", 4) == 0) {
    transitionToIdle();
    return;
  }

  if (strncmp(line, "AUDIO", 5) == 0) {
    const char *args = line + 5;
    while (*args == ' ') {
      ++args;
    }
    if (strncmp(args, "START", 5) == 0) {
      unsigned long sampleRate = 0;
      unsigned long frameCount = 0;
      char label[24] = {0};
      int matched = sscanf(args + 5, "%lu %lu %23s", &sampleRate, &frameCount, label);
      if (matched >= 2) {
        const char *labelPtr = (matched >= 3) ? label : nullptr;
        handleAudioStartCommand(static_cast<uint32_t>(sampleRate), static_cast<uint32_t>(frameCount), labelPtr);
      } else {
        Serial.println(F("LOG audio_start_parse_error"));
      }
    } else if (strncmp(args, "END", 3) == 0) {
      handleAudioEndCommand();
    }
    return;
  }
}

void pollSerial() {
  static char buffer[160];
  static size_t index = 0;
  while (true) {
    if (audioStreamMode == AUDIO_STREAM_RECEIVING && audioSamplesRemaining > 0) {
      pumpAudioReceive();
      if (audioSamplesRemaining > 0) {
        // Either out of serial bytes or the ring is full; in both cases every pending
        // inbound byte is still PCM, so don't try to parse it as a line. Break and let
        // loop() pump playback; the host is stalled by flow control if the ring is full.
        break;
      }
      // Payload complete: remaining inbound bytes are line protocol (AUDIO END).
    }

    if (!Serial.available()) {
      break;
    }

    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (index == 0) {
        continue;
      }
      buffer[index] = '\0';
      handleSerialLine(buffer);
      index = 0;
      continue;
    }
    if (index < sizeof(buffer) - 1) {
      buffer[index++] = c;
    }
  }
}

void setup() {
  // Set the system clock for clean 48 kHz division once, before the UARTs compute their
  // divisors, so audio never re-parents clk_peri mid-session (proposal §5.3).
  i2s.setSysClk(48000);

  Serial.begin(SERIAL_BAUD);
  Serial.ignoreFlowControl(true);
  unsigned long start = millis();
  while (!Serial && (millis() - start) < 500) {
    delay(10);
  }
  sendState("idle");

  pinMode(PIN_AMP_ENABLE, OUTPUT);
  digitalWrite(PIN_AMP_ENABLE, HIGH);
  resetAudioStreamState();

  Serial1.begin(115200);
  while (!Serial1) {
    delay(10);
  }

  if (!rvc.begin(&Serial1)) {
    Serial.println(F("STATE error"));
    while (true) {
      delay(100);
    }
  }
}

void loop() {
  if (audioStreamMode == AUDIO_STREAM_RECEIVING) {
    pumpAudioReceive();
  }

  pollSerial();

  if (audioStreamMode == AUDIO_STREAM_RECEIVING || audioStreamMode == AUDIO_STREAM_DRAINING) {
    pumpAudioPlayback();
    if (audioStreamMode == AUDIO_STREAM_DRAINING) {
      // Device-authoritative end (F-1): finish only when the ring is empty AND every DMA
      // buffer has been returned (available() back to its empty baseline), so audio_end
      // marks true playback completion, not just ring drain. The unconfigured skip path
      // has an empty ring and no DMA, so it completes immediately (F-2).
      bool drained = !audioOutputConfigured ||
                     (audioRingCount == 0 &&
                      (size_t)i2s.availableForWrite() >= audioDmaCapacityBytes);
      if (drained) {
        finishAudioDrain();
      }
    }
  }

  uint32_t now = millis();
  EulerAngles angles;
  Vector3 orientationVec;
  if (readOrientation(&angles, &orientationVec)) {
    updateAccelDetect();          // refresh accel sigma first; handleMotion reads it
    handleMotion(angles, now);
    updateDockTracking(now);
  }

  if (deviceMode != MODE_IDLE) {
    if (USE_ACCEL_DOCK_DETECT) {
      // Rigid-stillness (accel sigma) is the sole discriminator; pose match narrows the
      // window. nearTarget is intentionally NOT consulted: a genuinely docked device must
      // idle even when the holder pose coincides with the target. Deterministic: once the
      // device is picked up (hasLeftDock), then held mechanically still at the dock pose
      // for DOCK_CONFIRM_MS, it idles — no dependence on how the placement was performed.
      bool poseMatch =
          hasAngles &&
          fabsf(angularDifference(currentAngles.pitch, dockPitch)) < DOCK_POSE_TOL_DEG &&
          fabsf(angularDifference(currentAngles.roll, dockRoll)) < DOCK_POSE_TOL_DEG;
      bool rigidStill = accelSigmaValid && accelSigma < DOCK_RIGID_SIGMA;
      bool docked = hasLeftDock && poseMatch && rigidStill;
      if (docked) {
        if (dockConfirmStartMs == 0) {
          dockConfirmStartMs = now;
        } else if ((now - dockConfirmStartMs) >= DOCK_CONFIRM_MS) {
          transitionToIdle();
        }
      } else {
        dockConfirmStartMs = 0;  // a term dropped; restart the sustain window
      }

      // Throttled (~1 Hz) diagnostic so the artist can see exactly which term blocks
      // or falsely passes during a live show. Skip if we just idled this cycle.
      if (deviceMode != MODE_IDLE && (now - lastDockLogMs) >= 1000) {
        lastDockLogMs = now;
        uint32_t confirmMs = (dockConfirmStartMs != 0) ? (now - dockConfirmStartMs) : 0;
        Serial.print(F("LOG dock pitch="));
        Serial.print(currentAngles.pitch, 1);
        Serial.print(F(" roll="));
        Serial.print(currentAngles.roll, 1);
        Serial.print(F(" accsig="));
        Serial.print(accelSigmaValid ? accelSigma : -1.0f, 3);
        Serial.print(F(" pose="));
        Serial.print(poseMatch ? 1 : 0);
        Serial.print(F(" rigid="));
        Serial.print(rigidStill ? 1 : 0);
        Serial.print(F(" confirm_ms="));
        Serial.println(confirmMs);
      }
    } else {
      // Legacy yaw-referenced dock-cone path, unchanged — kept for instant reflash
      // rollback via USE_ACCEL_DOCK_DETECT = false.
      if (stillnessStartMs != 0 && (now - stillnessStartMs) >= MOTION_IDLE_TIMEOUT_MS) {
        float dockAngle = angleBetween(smoothedOrientation, DOCK_FORWARD_WORLD);
        bool inDockCone = dockAngle <= DOCK_ALIGNMENT_THRESHOLD_DEG;
        bool dockDwellMet =
            inDockCone && dockAlignStartMs != 0 && (now - dockAlignStartMs) >= DOCK_ALIGNMENT_DWELL_MS;
        bool nearTarget =
            targetValid && angleBetween(smoothedOrientation, targetVector) <= NEAR_TARGET_ANGLE_DEG;
        if (dockDwellMet && hasLeftDock) {
          transitionToIdle();
        } else if (nearTarget) {
          stillnessStartMs = now;
        } else {
          // Wearer is still hunting or has not returned to the dock; keep guiding instead of dropping to idle.
          stillnessStartMs = now;
        }
      }
    }
  }

  guidanceTick(now);
}
