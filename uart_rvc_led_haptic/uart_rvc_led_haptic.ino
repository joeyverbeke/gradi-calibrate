/* Orientation target game using Adafruit BNO08x in UART-RVC mode with LED and haptic feedback */

#include "Adafruit_BNO08x_RVC.h"
#include <Adafruit_NeoPixel.h>
#include <Adafruit_DRV2605.h>
#include <math.h>

#undef PIN_NEOPIXEL
#define PIN_NEOPIXEL 26  // GPIO26 / D0 on the XIAO RP2040 header

constexpr uint8_t NUM_PIXELS = 3;
constexpr uint16_t MIN_BLINK_MS = 100;
constexpr uint16_t MAX_BLINK_MS = 1000;
constexpr uint16_t MIN_PULSE_MS = 80;
constexpr uint16_t MAX_PULSE_MS = 1200;
constexpr float TARGET_TOLERANCE_DEG = 12.0f;
constexpr float TARGET_STEP_DEG = 15.0f;

Adafruit_BNO08x_RVC rvc;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_DRV2605 drv;

float targetYaw = 0.0f;
float targetPitch = 0.0f;
float targetRoll = 0.0f;

uint32_t axisColors[NUM_PIXELS] = {0, 0, 0};
unsigned long lastBlinkToggle[NUM_PIXELS] = {0, 0, 0};
uint16_t blinkInterval[NUM_PIXELS] = {MAX_BLINK_MS, MAX_BLINK_MS, MAX_BLINK_MS};
bool blinkOn[NUM_PIXELS] = {true, true, true};
bool serialReady = false;

unsigned long lastHapticToggle = 0;
uint16_t hapticInterval = MAX_PULSE_MS;
bool hapticOn = false;

float angularDifference(float target, float current) {
  float diff = target - current;
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

void pickNewTarget() {
  targetYaw = random(-12, 13) * TARGET_STEP_DEG;
  targetPitch = random(-6, 7) * TARGET_STEP_DEG;
  targetRoll = random(-12, 13) * TARGET_STEP_DEG;

  if (serialReady) {
    Serial.print("New target -> Yaw: ");
    Serial.print(targetYaw, 1);
    Serial.print(" Pitch: ");
    Serial.print(targetPitch, 1);
    Serial.print(" Roll: ");
    Serial.println(targetRoll, 1);
  }
}

uint32_t colorForDifference(float diffDegrees) {
  if (diffDegrees < 0.0f) {
    diffDegrees = -diffDegrees;
  }
  if (diffDegrees > 180.0f) {
    diffDegrees = 180.0f;
  }
  const float normalized = diffDegrees / 180.0f;
  const uint8_t red = static_cast<uint8_t>(200.0f * normalized);
  const uint8_t green = static_cast<uint8_t>(200.0f * (1.0f - normalized));
  return pixels.Color(red, green, 0);
}

void refreshPixel(uint8_t index) {
  const uint32_t color = blinkOn[index] ? axisColors[index] : 0;
  pixels.setPixelColor(index, color);
  pixels.show();
}

float updateAxisFeedback(const BNO08x_RVC_Data &heading) {
  const float yawDiff = angularDifference(targetYaw, heading.yaw);
  const float pitchDiff = angularDifference(targetPitch, heading.pitch);
  const float rollDiff = angularDifference(targetRoll, heading.roll);
  const float diffs[NUM_PIXELS] = {yawDiff, pitchDiff, rollDiff};

  float totalDiff = 0.0f;
  for (uint8_t i = 0; i < NUM_PIXELS; i++) {
    axisColors[i] = colorForDifference(diffs[i]);

    float absDiff = fabsf(diffs[i]);
    if (absDiff > 180.0f) {
      absDiff = 180.0f;
    }
    float normalized = absDiff / 180.0f;
    blinkInterval[i] = MIN_BLINK_MS + static_cast<uint16_t>((MAX_BLINK_MS - MIN_BLINK_MS) * normalized);

    totalDiff += absDiff;

    if (!blinkOn[i]) {
      pixels.setPixelColor(i, 0);
    } else {
      pixels.setPixelColor(i, axisColors[i]);
    }
  }
  pixels.show();

  return totalDiff / NUM_PIXELS;
}

void updateBlink(uint8_t index, unsigned long now) {
  if (now - lastBlinkToggle[index] < blinkInterval[index]) {
    return;
  }
  blinkOn[index] = !blinkOn[index];
  lastBlinkToggle[index] = now;
  refreshPixel(index);
}

void updateHaptic(float avgDiff, unsigned long now) {
  float normalized = avgDiff / 180.0f;
  if (normalized > 1.0f) {
    normalized = 1.0f;
  }
  hapticInterval = MIN_PULSE_MS + static_cast<uint16_t>((MAX_PULSE_MS - MIN_PULSE_MS) * normalized);

  if (now - lastHapticToggle >= hapticInterval) {
    lastHapticToggle = now;
    hapticOn = !hapticOn;

    if (hapticOn) {
      drv.setWaveform(0, 1); // strong click
      drv.setWaveform(1, 0); // end waveform
      drv.go();
    } else {
      drv.stop();
    }
  }
}

bool targetReached(const BNO08x_RVC_Data &heading) {
  return fabsf(angularDifference(targetYaw, heading.yaw)) <= TARGET_TOLERANCE_DEG &&
         fabsf(angularDifference(targetPitch, heading.pitch)) <= TARGET_TOLERANCE_DEG &&
         fabsf(angularDifference(targetRoll, heading.roll)) <= TARGET_TOLERANCE_DEG;
}

void celebrateTarget() {
  if (serialReady) {
    Serial.println(F("Target found!"));
  }
  const uint32_t blue = pixels.Color(0, 0, 150);
  for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t p = 0; p < NUM_PIXELS; p++) {
      pixels.setPixelColor(p, blue);
    }
    pixels.show();
    delay(120);
    pixels.clear();
    pixels.show();
    delay(120);
  }

  for (uint8_t i = 0; i < NUM_PIXELS; i++) {
    blinkOn[i] = true;
    lastBlinkToggle[i] = millis();
    axisColors[i] = 0;
    blinkInterval[i] = MAX_BLINK_MS;
  }
  pixels.clear();
  pixels.show();

  hapticOn = false;
  drv.stop();
  lastHapticToggle = millis();
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while ((millis() - start) < 500) {
    if (Serial) {
      serialReady = true;
      break;
    }
  }
  if (Serial) {
    serialReady = true;
  }

  if (serialReady) {
    Serial.println("Orientation target game - UART-RVC mode");
  }

  Serial1.begin(115200);
  while (!Serial1) {
    delay(10);
  }

  if (!rvc.begin(&Serial1)) {
    if (serialReady) {
      Serial.println("Could not find BNO08x!");
    }
    while (true) {
      delay(10);
    }
  }
  if (serialReady) {
    Serial.println("BNO08x found!");
  }

  if (!drv.begin()) {
    if (serialReady) {
      Serial.println("DRV2605 not found!");
    }
    while (true) {
      delay(10);
    }
  }
  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_INTTRIG);

  randomSeed(analogRead(A1));

  pixels.begin();
  pixels.setBrightness(64);
  pixels.clear();
  pixels.show();

  pickNewTarget();
}

void loop() {
  unsigned long now = millis();

  BNO08x_RVC_Data heading;
  if (rvc.read(&heading)) {
    const float avgDiff = updateAxisFeedback(heading);

    if (serialReady) {
      Serial.print("Current -> Yaw: ");
      Serial.print(heading.yaw, 1);
      Serial.print(" Pitch: ");
      Serial.print(heading.pitch, 1);
      Serial.print(" Roll: ");
      Serial.print(heading.roll, 1);
      Serial.print("  |  Target -> Yaw: ");
      Serial.print(targetYaw, 1);
      Serial.print(" Pitch: ");
      Serial.print(targetPitch, 1);
      Serial.print(" Roll: ");
      Serial.println(targetRoll, 1);
    }

    updateHaptic(avgDiff, now);

    if (targetReached(heading)) {
      celebrateTarget();
      pickNewTarget();
      now = millis();
    }
  }

  for (uint8_t i = 0; i < NUM_PIXELS; i++) {
    updateBlink(i, now);
  }
}
