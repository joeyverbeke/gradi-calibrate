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
constexpr float MOTION_START_THRESHOLD_DEG = 0.4f;
constexpr float MOTION_CONTINUE_THRESHOLD_DEG = 0.15f;
constexpr uint32_t MOTION_IDLE_TIMEOUT_MS = 7000;
constexpr bool ALLOW_DIAGONAL_BUCKETS = false;
constexpr float MAG_DECLINATION_DEG = -8.28f;  // Update for install location.
constexpr float MIN_DIAGONAL_DEG = 12.0f;
constexpr float MICRO_ADJUST_DEG = 1.0f;
constexpr float DOCK_ALIGNMENT_THRESHOLD_DEG = 8.0f;
constexpr float NEAR_TARGET_ANGLE_DEG = 5.0f;

constexpr int PIN_I2S_BCLK = 3;      // D10 / P3 -> MAX98357A BCLK
constexpr int PIN_I2S_LRCLK = 4;     // D9  / P4 -> MAX98357A LRC
constexpr int PIN_I2S_DATA = 2;      // D8  / P2 -> MAX98357A DIN
constexpr int PIN_AMP_ENABLE = 6;    // D6  / P0 -> MAX98357A SD (active high)
constexpr size_t AUDIO_BUFFER_SAMPLES = 256;
constexpr uint8_t AUDIO_SAMPLE_BITS = 16;
constexpr uint8_t AUDIO_BYTES_PER_SAMPLE = AUDIO_SAMPLE_BITS / 8;

static I2S i2s(OUTPUT);

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

constexpr Vector3 V_ARM_SENS = {0.0f, -1.0f, 0.0f};              // Board -Y points down the wearer’s arm.
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
bool audioEnabled = false;
char currentPlanet[16] = "unknown";
Mat3 R_align = MAT3_IDENTITY;
size_t microCycleIndex = 0;

enum AudioStreamMode {
  AUDIO_STREAM_IDLE,
  AUDIO_STREAM_RECEIVING,
  AUDIO_STREAM_WAITING_END
};

AudioStreamMode audioStreamMode = AUDIO_STREAM_IDLE;
uint32_t audioSamplesRemaining = 0;
size_t audioBufferFill = 0;
int16_t audioBuffer[AUDIO_BUFFER_SAMPLES];
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

  if (!i2s.begin(static_cast<long>(sampleRate))) {
    Serial.println(F("LOG audio_i2s_begin_failed"));
    return false;
  }

  audioOutputConfigured = true;
  audioCurrentSampleRateHz = sampleRate;
  return true;
}

void resetAudioStreamState() {
  audioStreamMode = AUDIO_STREAM_IDLE;
  audioSamplesRemaining = 0;
  audioBufferFill = 0;
}

void flushAudioBuffer() {
  if (audioBufferFill == 0) {
    return;
  }
  if (!audioOutputConfigured) {
    audioBufferFill = 0;
    return;
  }
  for (size_t i = 0; i < audioBufferFill; ++i) {
    int16_t s = audioBuffer[i];
    while (!i2s.write16(s, s)) {
      delayMicroseconds(50);
    }
  }
  audioBufferFill = 0;
}

bool pumpAudioStream() {
  if (audioStreamMode != AUDIO_STREAM_RECEIVING) {
    return false;
  }

  bool consumed = false;
  while (audioSamplesRemaining > 0 && Serial.available() >= AUDIO_BYTES_PER_SAMPLE) {
    int first = Serial.read();
    int second = Serial.read();
    if (first < 0 || second < 0) {
      break;
    }
    int16_t sample = static_cast<int16_t>(first | (second << 8));
    audioBuffer[audioBufferFill++] = sample;
    audioSamplesRemaining--;
    consumed = true;
    if (audioBufferFill >= AUDIO_BUFFER_SAMPLES) {
      flushAudioBuffer();
    }
  }

  if (audioSamplesRemaining == 0 && audioStreamMode == AUDIO_STREAM_RECEIVING) {
    flushAudioBuffer();
    audioStreamMode = AUDIO_STREAM_WAITING_END;
  }

  return consumed;
}

void handleAudioStartCommand(uint32_t sampleRate, uint32_t frameCount, const char *label) {
  resetAudioStreamState();
  if (frameCount == 0 || sampleRate == 0) {
    return;
  }

  bool shouldPlay = audioEnabled && configureAudioOutput(sampleRate);
  if (!shouldPlay) {
    audioOutputConfigured = false;
  }

  audioSamplesRemaining = frameCount;
  audioStreamMode = AUDIO_STREAM_RECEIVING;
  audioBufferFill = 0;
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
  flushAudioBuffer();
  Serial.println(F("LOG audio_end"));
  resetAudioStreamState();
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
  setMode(MODE_IDLE);
}

Vector3 eulerToForward(const EulerAngles &angles) {
  float yawTrue = angles.yaw + MAG_DECLINATION_DEG;
  Mat3 R_world_from_sensor = rotmatZYX_deg(yawTrue, angles.pitch, angles.roll);
  Vector3 forwardWorld = matMulVec(R_world_from_sensor, V_ARM_SENS);
  return normalize(forwardWorld);
}

bool readOrientation(EulerAngles *anglesOut, Vector3 *forwardOut) {
  BNO08x_RVC_Data data;
  if (!rvc.read(&data)) {
    return false;
  }
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

  if (deviceMode == MODE_IDLE && motionScore >= MOTION_START_THRESHOLD_DEG) {
    setMode(MODE_WAITING_FOR_START_ACK);
    lastMotionMs = nowMs;
    stillnessStartMs = 0;
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
    if (audioStreamMode == AUDIO_STREAM_RECEIVING) {
      bool consumed = pumpAudioStream();
      if (!consumed) {
        break;
      }
      if (audioStreamMode == AUDIO_STREAM_RECEIVING) {
        continue;
      }
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
  Serial.begin(SERIAL_BAUD);
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
    pumpAudioStream();
  }

  pollSerial();

  uint32_t now = millis();
  EulerAngles angles;
  Vector3 orientationVec;
  if (readOrientation(&angles, &orientationVec)) {
    handleMotion(angles, now);
  }

  if (deviceMode != MODE_IDLE) {
    if (stillnessStartMs != 0 && (now - stillnessStartMs) >= MOTION_IDLE_TIMEOUT_MS) {
      bool docked = angleBetween(smoothedOrientation, DOCK_FORWARD_WORLD) <= DOCK_ALIGNMENT_THRESHOLD_DEG;
      bool nearTarget =
          targetValid && angleBetween(smoothedOrientation, targetVector) <= NEAR_TARGET_ANGLE_DEG;
      if (docked || !targetValid) {
        transitionToIdle();
      } else if (nearTarget) {
        stillnessStartMs = now;
      } else {
        // Wearer is still hunting; keep guiding instead of dropping to idle.
        stillnessStartMs = now;
      }
    }
  }

  guidanceTick(now);
}
