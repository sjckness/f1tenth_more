// stepper_sweep_test.ino - minimal test: the 28BYJ-48 sweeps -15 / +15 deg by itself.
// No library, no serial commands needed. Pico 2 W + ULN2003 on GP2..GP5.
// If this moves and camera_pan.ino doesn't, the wiring is fine and the problem is the commands.

const uint8_t PINS[4] = {2, 3, 4, 5};   // IN1, IN2, IN3, IN4

// Half-step sequence (8 phases)
const uint8_t SEQ[8][4] = {
  {1,0,0,0}, {1,1,0,0}, {0,1,0,0}, {0,1,1,0},
  {0,0,1,0}, {0,0,1,1}, {0,0,0,1}, {1,0,0,1}
};

const float    STEPS_PER_DEG = 4075.7728 / 360.0;  // ~11.3 half-steps per degree
const uint32_t STEP_US       = 1500;               // time per half-step (~59 deg/s). Raise if it buzzes without turning.
const float    SWEEP_DEG     = 15.0;

int phase = 0;
long position = 0;   // in half-steps, 0 = where it was at power-on

void setCoils(int p) {
  for (int i = 0; i < 4; i++) digitalWrite(PINS[i], SEQ[p][i]);
}

void coilsOff() {
  for (int i = 0; i < 4; i++) digitalWrite(PINS[i], LOW);
}

void moveTo(float deg) {
  long target = lround(deg * STEPS_PER_DEG);
  while (position != target) {
    int dir = (target > position) ? 1 : -1;
    phase = (phase + dir + 8) % 8;
    setCoils(phase);
    position += dir;
    delayMicroseconds(STEP_US);
  }
  coilsOff();   // no heat while waiting; the gearbox holds the position
  Serial.print("at ");
  Serial.print(position / STEPS_PER_DEG, 2);
  Serial.println(" deg");
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) pinMode(PINS[i], OUTPUT);
  coilsOff();
  delay(2000);
  Serial.println("sweep test start");
}

void loop() {
  moveTo(+SWEEP_DEG);  delay(1000);
  moveTo(-SWEEP_DEG);  delay(1000);
  moveTo(0);           delay(1000);
}
