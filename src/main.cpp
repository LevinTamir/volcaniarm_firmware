#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

// STEP1: GPIO16, DIR1: GPIO17
// STEP2: GPIO18, DIR2: GPIO19

const int STEP_PIN_1 = 16;
const int DIR_PIN_1  = 17;
const int STEP_PIN_2 = 18;
const int DIR_PIN_2 = 19;
const int LED_PIN = 2;  // Built-in LED on NodeMCU-32S

// Limit switch pins (NC switches between pin and GND, using internal pull-up)
// Normal state: LOW (closed), Triggered: HIGH (open/pressed)
const int RIGHT_LIMIT_PIN = 25;
const int LEFT_LIMIT_PIN  = 26;

// Motor configuration (51200 microsteps/rev * 3:1 gear = 153600 steps/output rev)
const float MAX_SPEED = 50000.0;        // steps per second
const float ACCELERATION = 50000.0;     // steps per second^2

// Homing configuration
const float HOMING_SPEED = 10000.0;         // steps/sec - slow for safety
const float HOMING_SPEED_PASSIVE = 7500.0;  // steps/sec - passive joint follows slower

// Known step positions of limit switches relative to home (0,0).
// Measure these: manually place arm at home, move to limit, read step count.
// Right limit is in the negative direction, left limit in the positive.
const long RIGHT_LIMIT_STEPS = -102400;  // TODO: measure actual value
const long LEFT_LIMIT_STEPS  = 102400;   // TODO: measure actual value

// Create stepper instances (DRIVER mode: STEP, DIR pins)
AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);

// MultiStepper for coordinated movement
MultiStepper steppers;

// Track if we're using coordinated movement
bool useCoordinatedMovement = false;

static const int INPUT_BUFFER_SIZE = 64;
char input_buffer[INPUT_BUFFER_SIZE];
int input_pos = 0;

// Position reporting interval (ms)
const unsigned long REPORT_INTERVAL_MS = 20;
unsigned long last_report_ms = 0;

void handleSerial();
void processCommand(const char * line);
void home();

void setup()
{
  // Configure stepper 1
  stepper1.setMaxSpeed(MAX_SPEED);
  stepper1.setAcceleration(ACCELERATION);
  stepper1.setCurrentPosition(0);

  // Configure stepper 2
  stepper2.setMaxSpeed(MAX_SPEED);
  stepper2.setAcceleration(ACCELERATION);
  stepper2.setCurrentPosition(0);

  // Add steppers to MultiStepper for coordinated movement
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);

  // Limit switches: NC to GND, use internal pull-up
  pinMode(RIGHT_LIMIT_PIN, INPUT_PULLUP);
  pinMode(LEFT_LIMIT_PIN, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  while (!Serial) {
    ; // needed only for some boards
  }

}

// Returns true if the given limit switch is triggered.
// NC switch with pull-up: LOW = closed (normal), HIGH = open (triggered).
bool limitTriggered(int pin)
{
  return digitalRead(pin) == HIGH;
}

// Move toward a limit switch. Both motors move together (the active one at
// homing speed, the passive one slightly slower).
// dir: +1 or -1 indicating step direction toward the limit.
void seekLimit(int limit_pin, AccelStepper &active, AccelStepper &passive, int dir)
{
  float active_speed  = dir * HOMING_SPEED;
  float passive_speed = dir * HOMING_SPEED_PASSIVE;

  unsigned long debounce_start = 0;
  bool debouncing = false;

  active.setSpeed(active_speed);
  passive.setSpeed(passive_speed);

  while (true) {
    active.runSpeed();
    passive.runSpeed();

    if (limitTriggered(limit_pin)) {
      if (!debouncing) {
        debounce_start = millis();
        debouncing = true;
      } else if (millis() - debounce_start >= 10) {
        break;  // confirmed
      }
    } else {
      debouncing = false;
    }
  }
}

void home()
{
  digitalWrite(LED_PIN, HIGH);

  Serial.println("HOMING_START");

  // Disable acceleration limits during homing -- use constant speed via runSpeed()
  stepper1.setMaxSpeed(HOMING_SPEED);
  stepper2.setMaxSpeed(HOMING_SPEED);
  stepper1.setAcceleration(ACCELERATION);
  stepper2.setAcceleration(ACCELERATION);

  // Phase 1: Seek right limit switch (negative direction for right motor)
  // stepper1 = right (active), stepper2 = left (passive)
  seekLimit(RIGHT_LIMIT_PIN, stepper1, stepper2, -1);

  // Now at the right limit -- set absolute position
  stepper1.setCurrentPosition(RIGHT_LIMIT_STEPS);
  // The passive motor moved the same direction; its position is unknown but
  // will be calibrated in phase 2.

  // Phase 2: Seek left limit switch (positive direction for left motor)
  // stepper2 = left (active), stepper1 = right (passive)
  seekLimit(LEFT_LIMIT_PIN, stepper2, stepper1, +1);

  // Now at the left limit -- set absolute position
  stepper2.setCurrentPosition(LEFT_LIMIT_STEPS);

  // Phase 3: Move to home position (0, 0) using coordinated movement
  stepper1.setMaxSpeed(MAX_SPEED);
  stepper2.setMaxSpeed(MAX_SPEED);

  long home_positions[2] = {0, 0};
  steppers.moveTo(home_positions);
  while (steppers.run()) {
    // blocking until both reach home
  }

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);

  digitalWrite(LED_PIN, LOW);

  Serial.println("H 0 0");
}

void loop()
{
  handleSerial();

  // Use appropriate run method based on movement mode
  if (useCoordinatedMovement) {
    // MultiStepper handles coordination to reach targets simultaneously
    steppers.run();
  } else {
    // Individual control - only run if motors need to move
    if (stepper1.distanceToGo() != 0) {
      stepper1.run();
    }
    if (stepper2.distanceToGo() != 0) {
      stepper2.run();
    }
  }

  // Stream current positions at fixed rate
  unsigned long now = millis();
  if (now - last_report_ms >= REPORT_INTERVAL_MS) {
    last_report_ms = now;
    Serial.print("S ");
    Serial.print(stepper1.currentPosition());
    Serial.print(" ");
    Serial.println(stepper2.currentPosition());
  }
}

void handleSerial()
{
  while (Serial.available() > 0)
  {
    char c = static_cast<char>(Serial.read());

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      input_buffer[input_pos] = '\0';
      processCommand(input_buffer);
      input_pos = 0;
    } else {
      if (input_pos < INPUT_BUFFER_SIZE - 1) {
        input_buffer[input_pos++] = c;
      } else {
        input_pos = 0;
      }
    }
  }
}

void processCommand(const char * line)
{
  // Parse format: "P1 steps1 P2 steps2" - coordinated movement
  long steps1 = 0, steps2 = 0;
  int matched = sscanf(line, "P1 %ld P2 %ld", &steps1, &steps2);
  
  if (matched == 2) {
    // Use MultiStepper for coordinated movement
    // Both motors will reach their targets at the same time
    long positions[2];
    positions[0] = steps1;
    positions[1] = steps2;
    steppers.moveTo(positions);
    useCoordinatedMovement = true;
    return;
  }
  
  // Legacy format: P1steps or P2steps - individual movement
  if (line[0] == 'P' || line[0] == 'p') {
    useCoordinatedMovement = false;  // Switch to individual control
    if (line[1] == '1') {
      long steps = 0;
      int matched = sscanf(line + 2, "%ld", &steps);
      if (matched == 1) {
        stepper1.moveTo(steps);
      }
    } else if (line[1] == '2') {
      long steps = 0;
      int matched = sscanf(line + 2, "%ld", &steps);
      if (matched == 1) {
        stepper2.moveTo(steps);
      }
    }
  } else if (line[0] == 'Z' || line[0] == 'z' ||
             line[0] == 'H' || line[0] == 'h') {
    // Home/zero: reset position and report
    home();
  }
}
