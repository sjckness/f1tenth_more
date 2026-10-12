// camera_pan.ino - ZED 2i pan stepper controller for the F1Tenth car
//
// Board : Raspberry Pi Pico 2 W  (Arduino IDE + "arduino-pico" core by Earle Philhower,
//         board "Raspberry Pi Pico 2W"). Also builds unchanged for Uno/Nano.
// Driver: ULN2003 board (IN1..IN4)  +  28BYJ-48 geared stepper (5 wires)
// Lib   : AccelStepper (Library Manager -> "AccelStepper" by Mike McCauley)
//
// Serial protocol, 115200 baud (ignored on the Pico's native USB), ASCII,
// one command per line ("\n" or "\r\n"):
//   T <deg>   go to target angle (clamped to +/-LIMIT_DEG). Ignored if not homed.
//   H         run homing (needs HAS_HOME_SWITCH 1)
//   Z         declare the CURRENT position = 0 deg and mark homed
//             (bench testing without a switch: center the camera by hand, then Z)
//   E <0|1>   disable / enable the motor (disable = coils off, position becomes unknown)
//   W <0|1>   disable / enable the command watchdog (default enabled)
//   ?         print config
//
// Report, every REPORT_PERIOD_MS:
//   S <millis> <position_deg> <target_deg> <flags>
//   flags bits: 1=homed 2=moving 4=fault 8=watchdog_tripped 16=motor_enabled 32=coils_energized
//
// "position_deg" is the step counter. It is the real camera angle only if no
// steps were lost AND ignoring gearbox backlash (28BYJ-48: ~1-3 deg!). An AS5600
// encoder on the OUTPUT shaft is the fix for both and should come before the car.

#include <AccelStepper.h>

// ============================== CONFIG ==================================
// ULN2003 inputs. On the Pico these numbers are GP numbers.
const uint8_t PIN_IN1  = 2;
const uint8_t PIN_IN2  = 3;
const uint8_t PIN_IN3  = 4;
const uint8_t PIN_IN4  = 5;
const uint8_t PIN_HOME = 6;     // switch between pin and GND (INPUT_PULLUP)

#define HAS_HOME_SWITCH 0       // set to 1 once the switch is mounted
#define INVERT_DIR      0       // set to 1 if "T 15" turns the camera RIGHT (+ must be LEFT, ROS z-up)

// 28BYJ-48 in half-step mode: 64 half-steps/motor rev x 63.684 gearbox = ~4076 per output rev.
// (Often quoted as 4096 - the real gear ratio is 63.684:1, not 64:1.)
const float STEPS_PER_REV = 4075.7728;
const float GEAR_RATIO    = 1.0;    // extra belt/gear between motor shaft and camera
const float STEPS_PER_DEG = STEPS_PER_REV * GEAR_RATIO / 360.0;   // ~11.32

const float LIMIT_DEG       = 15.0;
const float MAX_SPEED_DEG_S = 60.0;   // 28BYJ-48 tops out around 80-90 deg/s; above that it stalls
const float ACCEL_DEG_S2    = 200.0;

// Coils off when idle: saves ~250 mA of heat. The 64:1 gearbox is hard to
// back-drive, so the camera holds position anyway. Set 0 to keep holding torque.
const uint32_t RELEASE_AFTER_MS = 500;

const float HOME_SWITCH_DEG  = -20.0; // camera angle where the switch triggers - MEASURE IT
const float HOME_SPEED_DEG_S = 10.0;
const float HOME_SEARCH_DEG  = 60.0;  // give up after this much travel

const uint32_t WATCHDOG_MS      = 200;
const uint32_t REPORT_PERIOD_MS = 20;  // 50 Hz
const long     BAUD             = 115200;
// ========================================================================

// HALF4WIRE wants the pins in the order IN1, IN3, IN2, IN4.
#if INVERT_DIR
AccelStepper stepper(AccelStepper::HALF4WIRE, PIN_IN4, PIN_IN2, PIN_IN3, PIN_IN1);
#else
AccelStepper stepper(AccelStepper::HALF4WIRE, PIN_IN1, PIN_IN3, PIN_IN2, PIN_IN4);
#endif

bool homed = false;
bool fault = false;
bool motorEnabled = true;     // user-level enable (E command)
bool coilsOn = false;         // actual coil state (idle release)
bool watchdogEnabled = true;
bool watchdogTripped = false;
uint32_t lastCmdMs = 0;
uint32_t lastReportMs = 0;
uint32_t lastMoveMs = 0;
float targetDeg = 0.0;

char rxBuf[32];
uint8_t rxLen = 0;

long degToSteps(float deg) { return lround(deg * STEPS_PER_DEG); }
float stepsToDeg(long steps) { return steps / STEPS_PER_DEG; }

void coils(bool on) {
  if (on && !coilsOn) stepper.enableOutputs();
  if (!on && coilsOn) stepper.disableOutputs();
  coilsOn = on;
}

bool homeSwitchActive() {
#if HAS_HOME_SWITCH
  // two reads ~50 us apart as a cheap debounce
  if (digitalRead(PIN_HOME) != LOW) return false;
  delayMicroseconds(50);
  return digitalRead(PIN_HOME) == LOW;
#else
  return false;
#endif
}

void setSpeedProfile(float speedDegS, float accelDegS2) {
  stepper.setMaxSpeed(speedDegS * STEPS_PER_DEG);
  stepper.setAcceleration(accelDegS2 * STEPS_PER_DEG);
}

void setTargetDeg(float deg) {
  targetDeg = constrain(deg, -LIMIT_DEG, LIMIT_DEG);
  long t = degToSteps(targetDeg);
  if (t != stepper.targetPosition() || stepper.distanceToGo() != 0) {
    coils(true);
    stepper.moveTo(t);
  }
}

void setFault(const char *why) {
  fault = true;
  homed = false;
  stepper.setCurrentPosition(stepper.currentPosition()); // stop immediately
  Serial.print(F("E fault: "));
  Serial.println(why);
}

void runBlocking() {
  while (stepper.distanceToGo() != 0) stepper.run();
}

// Blocking homing: fine, it only runs at boot or on request.
bool runHoming() {
#if HAS_HOME_SWITCH
  Serial.println(F("I homing"));
  fault = false;
  homed = false;
  motorEnabled = true;
  coils(true);
  setSpeedProfile(HOME_SPEED_DEG_S, ACCEL_DEG_S2);
  stepper.setCurrentPosition(0);

  // If we start ON the switch, back off first.
  if (homeSwitchActive()) {
    stepper.move(degToSteps(5.0));
    runBlocking();
  }

  // Search toward the switch (negative direction).
  stepper.move(-degToSteps(HOME_SEARCH_DEG));
  while (stepper.distanceToGo() != 0) {
    if (homeSwitchActive()) {
      stepper.setCurrentPosition(degToSteps(HOME_SWITCH_DEG));
      homed = true;
      break;
    }
    stepper.run();
  }
  setSpeedProfile(MAX_SPEED_DEG_S, ACCEL_DEG_S2);
  if (!homed) {
    setFault("home switch not found");
    return false;
  }
  setTargetDeg(0.0);
  runBlocking();
  lastCmdMs = lastMoveMs = millis();
  Serial.println(F("I homed"));
  return true;
#else
  Serial.println(F("E no home switch (HAS_HOME_SWITCH 0) - use Z"));
  return false;
#endif
}

void printConfig() {
  Serial.print(F("I steps_per_deg "));   Serial.println(STEPS_PER_DEG, 4);
  Serial.print(F("I limit_deg "));       Serial.println(LIMIT_DEG, 2);
  Serial.print(F("I max_speed_deg_s ")); Serial.println(MAX_SPEED_DEG_S, 1);
  Serial.print(F("I accel_deg_s2 "));    Serial.println(ACCEL_DEG_S2, 1);
  Serial.print(F("I home_switch "));     Serial.println(HAS_HOME_SWITCH);
  Serial.print(F("I watchdog_ms "));     Serial.println(WATCHDOG_MS);
  Serial.print(F("I release_ms "));      Serial.println(RELEASE_AFTER_MS);
}

void handleCommand(char *line) {
  char cmd = line[0];
  float arg = (line[1] != '\0') ? atof(line + 1) : 0.0;

  switch (cmd) {
    case 'T':
      if (!homed) { Serial.println(F("E not homed")); return; }
      if (!motorEnabled) { Serial.println(F("E motor disabled")); return; }
      setTargetDeg(arg);
      lastCmdMs = millis();
      watchdogTripped = false;
      break;
    case 'H':
      runHoming();
      break;
    case 'Z':
      stepper.setCurrentPosition(0);
      targetDeg = 0.0;
      homed = true;
      fault = false;
      motorEnabled = true;
      lastCmdMs = lastMoveMs = millis();
      Serial.println(F("I zero set"));
      break;
    case 'E':
      motorEnabled = (arg != 0.0);
      if (!motorEnabled) {
        stepper.setCurrentPosition(stepper.currentPosition());
        coils(false);
        homed = false;  // could be turned by hand now
      }
      break;
    case 'W':
      watchdogEnabled = (arg != 0.0);
      lastCmdMs = millis();
      break;
    case '?':
      printConfig();
      break;
    default:
      Serial.println(F("E unknown command"));
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) {
        rxBuf[rxLen] = '\0';
        handleCommand(rxBuf);
        rxLen = 0;
      }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;  // overflow: drop the line
    }
  }
}

void report() {
  uint32_t now = millis();
  if (now - lastReportMs < REPORT_PERIOD_MS) return;
  // Never block: a blocking print would stall step generation.
  if (Serial.availableForWrite() < 48) return;
  lastReportMs = now;

  uint8_t flags = 0;
  if (homed)                        flags |= 1;
  if (stepper.distanceToGo() != 0)  flags |= 2;
  if (fault)                        flags |= 4;
  if (watchdogTripped)              flags |= 8;
  if (motorEnabled)                 flags |= 16;
  if (coilsOn)                      flags |= 32;

  Serial.print('S'); Serial.print(' ');
  Serial.print(now); Serial.print(' ');
  Serial.print(stepsToDeg(stepper.currentPosition()), 3); Serial.print(' ');
  Serial.print(targetDeg, 3); Serial.print(' ');
  Serial.println(flags);
}

void setup() {
  Serial.begin(BAUD);
  pinMode(PIN_HOME, INPUT_PULLUP);
  setSpeedProfile(MAX_SPEED_DEG_S, ACCEL_DEG_S2);
  stepper.disableOutputs();   // all IN pins LOW = coils off until needed
  coilsOn = false;

  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}  // Pico USB: give the host a moment

  Serial.println(F("I camera_pan ready"));
  printConfig();

#if HAS_HOME_SWITCH
  delay(500);
  runHoming();
#else
  Serial.println(F("I no home switch: center camera by hand, then send Z"));
#endif
  lastCmdMs = lastMoveMs = millis();
}

void loop() {
  readSerial();

  if (watchdogEnabled && homed && !watchdogTripped &&
      millis() - lastCmdMs > WATCHDOG_MS) {
    watchdogTripped = true;
    setTargetDeg(0.0);
  }

  // Switch sits beyond the travel limit: touching it in normal use = lost steps.
  if (homed && homeSwitchActive()) {
    setFault("home switch hit during operation (lost steps)");
  }

  stepper.run();

  if (stepper.distanceToGo() != 0) {
    lastMoveMs = millis();
  } else if (RELEASE_AFTER_MS > 0 && coilsOn &&
             millis() - lastMoveMs > RELEASE_AFTER_MS) {
    coils(false);
  }

  report();
}
