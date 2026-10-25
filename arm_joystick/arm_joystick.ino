3#include <ESP32Servo.h>
#include <math.h>

// ---------- Pins ----------
const int SERVO_PINS[4] = {13, 14, 27, 33};  // base (360), shoulder (180), elbow (180), wrist (180)
const int JOY_X   = 34;                      // ADC1 pin
const int JOY_Y   = 35;                      // ADC1 pin
const int BTN_UP   = 25;                     // to GND, internal pull-up
const int BTN_DOWN = 26;                     // to GND, internal pull-up
const int LED_PIN  = 2;                      // onboard LED: on = out of reach

// ---------- Base Servo Calibration (MG995 360) ----------
// Continuous servos treat 90 as stop. Adjust +/- if your specific motor creeps at 90.
const int BASE_STOP_VAL = 90;
const int BASE_DEADBAND = 1;                  // Command deadband around STOP
const float BASE_SPEED_SCALER = 25.0;        // Max speed offset (+/- from 90, e.g. 65..115)
const float BASE_DEG_PER_SEC_MAX = 50.0;     // Approx rotational speed at full stick (deg/sec)
const int   BASE_DIR = 1;                     // Set to 1 or -1 to match stick direction

// ---------- Arm dimensions in metres (matches URDF) ----------
const float Z_SHOULDER = 0.08;   // shoulder height above turntable
const float L1 = 0.16;           // shoulder -> elbow
const float L2 = 0.14;           // elbow -> wrist joint
const float L3 = 0.06;           // wrist -> tip (tool points straight down)

// ---------- Positional Servos (Shoulder, Elbow, Wrist) ----------
const int   SIGN[4]       = {1, 1, 1, 1};
const float MIN_ANGLE[4]  = {10, 10, 10, 10};
const float MAX_ANGLE[4]  = {170, 170, 170, 170};

// ---------- Motion settings ----------
const float HOME_R = 0.22;       // Radial reach from base center (metres)
const float HOME_Z = 0.12;       // Height (metres)
const float R_MIN  = 0.08, R_MAX = 0.32;
const float Z_MIN  = 0.03, Z_MAX = 0.20;

const float R_STEP  = 0.0020;    // radial step per 20ms loop
const float Z_STEP  = 0.0010;    // height step per 20ms loop while button held
const int   DEADZONE = 400;      // analog stick deadzone out of 4095
const bool  INVERT_X = false;
const bool  INVERT_Y = false;
const unsigned long LOOP_MS = 20;

// When the stick is pushed mostly forward/back, ignore the small sideways
// reading so the base does not turn. Base only turns if the sideways push is
// at least this fraction of the forward/back push (0 = off, 1 = strict).
const float AXIS_LOCK_RATIO = 0.7;

// Set to true to print raw stick values ("J,x,y,jx,jy") to the Serial Monitor.
// The ROS 2 bridge ignores these lines, but close the Monitor before using ROS 2.
const bool DEBUG_JOY = false;

// ---------- State ----------
Servo servos[4];
float targetR = HOME_R;
float targetZ = HOME_Z;
float baseEstimatedAngle = 90.0;             // Virtual tracked angle (degrees)
float angle[4] = {90.0, 90.0, 90.0, 90.0};   // Angles reported over Serial
int centerX = 2048, centerY = 2048;
unsigned long lastLoop = 0, lastPrint = 0;

// Average 8 ADC readings to suppress ESP32 noise
int sampleADC(int pin) {
  long sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(pin);
  }
  return sum / 8;
}

// Convert raw joystick reading to normalized range [-1.0 .. 1.0]
float readAxis(int pin, int center, bool invert) {
  int raw = sampleADC(pin);
  int offset = raw - center;
  if (abs(offset) < DEADZONE) return 0.0;
  float sign = (offset > 0) ? 1.0 : -1.0;
  float value = (abs(offset) - DEADZONE) / (float)(2047 - DEADZONE);
  value = constrain(value, 0.0, 1.0) * sign;
  return invert ? -value : value;
}

// 2D Planar IK for the vertical arm chain (shoulder, elbow, wrist)
bool solvePlanarIK(float r, float z, float &outShoulder, float &outElbow, float &outWrist) {
  float wristLo = radians(MIN_ANGLE[3] - 90.0);
  float wristHi = radians(MAX_ANGLE[3] - 90.0);
  float theta = PI; // tool points straight down

  float q1 = 0, q2 = 0, q3 = 0;
  for (int it = 0; it < 10; it++) {
    float wr = r - L3 * sin(theta);
    float wz = z - L3 * cos(theta) - Z_SHOULDER;
    float c  = (wr * wr + wz * wz - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    if (c < -1.0 || c > 1.0) return false;

    q2 = acos(c);
    float phi = atan2(wr, wz);
    q1 = phi - atan2(L2 * sin(q2), L1 + L2 * cos(q2));
    q3 = constrain(theta - q1 - q2, wristLo, wristHi);
    theta = q1 + q2 + q3;
  }

  outShoulder = 90.0 + degrees(q1);
  outElbow    = 90.0 + degrees(q2);
  outWrist    = 90.0 + degrees(q3);

  if (outShoulder < MIN_ANGLE[1] || outShoulder > MAX_ANGLE[1]) return false;
  if (outElbow    < MIN_ANGLE[2] || outElbow    > MAX_ANGLE[2]) return false;
  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  analogReadResolution(12);
  delay(200);

  // Calibrate joystick resting center
  long sx = 0, sy = 0;
  for (int i = 0; i < 100; i++) {
    sx += analogRead(JOY_X);
    sy += analogRead(JOY_Y);
    delay(5);
  }
  centerX = sx / 100;
  centerY = sy / 100;

  // Attach servos
  for (int i = 0; i < 4; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], 500, 2400);
  }

  // Ensure continuous base servo is stopped at start
  servos[0].write(BASE_STOP_VAL);

  // Solve home posture for arm joints
  float s, e, w;
  if (solvePlanarIK(HOME_R, HOME_Z, s, e, w)) {
    angle[1] = s;
    angle[2] = e;
    angle[3] = w;
    servos[1].write((int)(90 + SIGN[1] * (angle[1] - 90.0)));
    servos[2].write((int)(90 + SIGN[2] * (angle[2] - 90.0)));
    servos[3].write((int)(90 + SIGN[3] * (angle[3] - 90.0)));
  }
}

void loop() {
  unsigned long now = millis();

  if (now - lastLoop >= LOOP_MS) {
    float dt = (now - lastLoop) / 1000.0;
    lastLoop = now;

    bool up   = digitalRead(BTN_UP)   == LOW;
    bool down = digitalRead(BTN_DOWN) == LOW;

    if (up && down) {
      // Home posture reset
      targetR = HOME_R;
      targetZ = HOME_Z;
      servos[0].write(BASE_STOP_VAL);
      digitalWrite(LED_PIN, LOW);
    } else {
      float jx = readAxis(JOY_X, centerX, INVERT_X); // Base spin command
      float jy = readAxis(JOY_Y, centerY, INVERT_Y); // Reach in/out command

      // Axis lock: pushing forward/back must not spin the base
      if (fabs(jx) < AXIS_LOCK_RATIO * fabs(jy)) jx = 0.0;

      if (DEBUG_JOY) {
        Serial.printf("J,%d,%d,%.2f,%.2f\n", analogRead(JOY_X), analogRead(JOY_Y), jx, jy);
      }

      // --- 1. Base Continuous Motor Handling ---
      if (fabs(jx) < 0.05) {
        servos[0].write(BASE_STOP_VAL); // Complete stop
      } else {
        int speedCmd = BASE_STOP_VAL + (int)(jx * BASE_SPEED_SCALER * BASE_DIR);
        servos[0].write(speedCmd);

        // Update estimated base angle for simulation / bridge
        baseEstimatedAngle += jx * BASE_DEG_PER_SEC_MAX * dt * BASE_DIR;
        baseEstimatedAngle = constrain(baseEstimatedAngle, 10.0, 170.0);
      }
      angle[0] = baseEstimatedAngle;

      // --- 2. Shoulder, Elbow, Wrist Handling ---
      float nr = constrain(targetR + jy * R_STEP, R_MIN, R_MAX);
      float nz = targetZ;
      if (up)   nz += Z_STEP;
      if (down) nz -= Z_STEP;
      nz = constrain(nz, Z_MIN, Z_MAX);

      float s, e, w;
      if (solvePlanarIK(nr, nz, s, e, w)) {
        targetR = nr;
        targetZ = nz;
        angle[1] = s;
        angle[2] = e;
        angle[3] = w;

        // Drive standard 180 positional joints
        servos[1].write((int)(90 + SIGN[1] * (angle[1] - 90.0)));
        servos[2].write((int)(90 + SIGN[2] * (angle[2] - 90.0)));
        servos[3].write((int)(90 + SIGN[3] * (angle[3] - 90.0)));
        digitalWrite(LED_PIN, LOW);
      } else {
        digitalWrite(LED_PIN, HIGH); // unreachable position
      }
    }
  }

  // Publish joint angles over serial for ROS 2 bridge (A,base,shoulder,elbow,wrist)
  if (now - lastPrint >= 50) {
    lastPrint = now;
    Serial.printf("A,%.1f,%.1f,%.1f,%.1f\n", angle[0], angle[1], angle[2], angle[3]);
  }
}