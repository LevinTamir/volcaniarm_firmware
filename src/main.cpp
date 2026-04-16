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

// Motor configuration
const float MAX_SPEED = 30000.0;        // steps per second
const float ACCELERATION = 30000.0;      // steps per second^2

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

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  while (!Serial) {
    ; // needed only for some boards
  }

  home();
}

// Homing placeholder: assumes current position is 0,0.
// TODO: replace with limit switch homing sequence.
void home()
{
  // Blink LED 3 times to indicate homing
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
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
