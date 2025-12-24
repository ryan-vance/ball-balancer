#include <TouchScreen.h> // Adafruit TouchScreen library
#include <AccelStepper.h> //Stepper driver library
#include <math.h> // Math library


// Define touchscreen pins and create TouchScreen object
TouchScreen ts(38, 41, 40, 39, 0); //  (XGND, YGND, X5V, Y5V)


// Define motor configuration
#define STEPS_PER_REV 1600  // Motor steps per revolution
const int en_pin = 10;
const int MOTOR_PINS[][2] = {
  {8, 9},    // Motor 3: Step, Dir
  {6, 7},   // Motor 2: Step, Dir
  {4, 5}  // Motor 1: Step, Dir
};


long missCount = 0;


// Create stepper objects
AccelStepper steppers[] = {
  AccelStepper(AccelStepper::DRIVER, MOTOR_PINS[0][0], MOTOR_PINS[0][1]),  // First pin is step, second is DIR
  AccelStepper(AccelStepper::DRIVER, MOTOR_PINS[1][0], MOTOR_PINS[1][1]),
  AccelStepper(AccelStepper::DRIVER, MOTOR_PINS[2][0], MOTOR_PINS[2][1])
};




// Control dimensions (x, y, z for delta robot)
const int dimensions = 3;


// Variables for PID calculations
float target[dimensions] = {0, 0, 4.25}; // Target position (x, y, h) - center with height 4.25
float position[dimensions] = {0, 0, 4.25}; // Current position (x, y, h)
float error[dimensions] = {0, 0, 0}; // Current error
float lastError[dimensions] = {0, 0, 0}; // Previous error
float integral[dimensions] = {0, 0, 0}; // Integral sum
float derivative[dimensions] = {0, 0, 0}; // Rate of change
float output[dimensions] = {0, 0, 0}; // Control outputs (x, y, h adjustments)


// Constants for PID
float kp = 0.15; // Proportional gain
float ki = 0.02; // Integral gain
float kd = 0.075; // Derivative gain


// Delta robot parameters
const double Rb = 2; // Base radius
const double Rp = 3.125; // Platform radius
const double Llow = 1.75; // Lower arm length
const double Lup = 3.669291339; // Upper arm length
const double defaultHeight = 4.25; // Default platform height


// Variables for timing
unsigned long previousTime = 0;
const float maxIntegral = 175; // Max buildup of integral


// Variables for motor control
long motorPositions[3] = {0, 0, 0};
float motorSpeed[3] = {1000, 1000, 1000}; // Default speeds
float prevMotorSpeed[3] = {1000, 1000, 1000};
float speedKs = 25; // Speed amplifying constant
double angOrig = 206.662752199; // Original starting angle
double angToStep = STEPS_PER_REV / 360.0; // Angle to step conversion


// Ball detection flag
bool ballDetected = false;


// Function to offset x- and y-coordinates of point
TSPoint offsetPoint(TSPoint pt, int16_t off_x, int16_t off_y) {
   pt.x = pt.x - off_x;
   pt.y = pt.y - off_y;
   return pt;
}


// Function to deoffset x- and y-coordinates of point
TSPoint deoffsetPoint(TSPoint pt, int16_t off_x, int16_t off_y) {
   pt.x = pt.x + off_x;
   pt.y = pt.y + off_y;
   return pt;
}


// Inverse kinematics function
double MtrAng(int mtrNum, double h, double n_x, double n_y) {
  double theta = 0;    // Initialize theta to be 0 (radians)
  double x, y, z, len; // Declare intermediate values


  // Find the normal vector
  double nlen = sqrt(1 + n_y * n_y + n_x * n_x);
  double n_z = pow(nlen, -1);
  n_y /= nlen;
  n_x /= nlen;


  // Find the corresponding motor angle based on inverse kinematics calculations
  if (mtrNum == 0) {
    y = Rb + (Rp / 2) * (1 - (n_x * n_x + 3 * n_z * n_z + 3 * n_z) / (n_z + 1 - n_x * n_x + (pow(n_x, 4) - 3 * n_x * n_x * n_y * n_y) / ((n_z + 1) * (n_z + 1 - n_x * n_x))));
    z = Rp * n_y + h;
    len = sqrt(z * z + y * y);
    theta = acos((len * len + Llow * Llow - Lup * Lup) / (2 * len * Llow)) + acos(y / len);
  }
  else if (mtrNum == 1) {
    x = (sqrt(3) / 2) * (Rp * (1 - (n_x * n_x + sqrt(3) * n_x * n_y) / (n_z + 1)) - Rb);
    y = x / sqrt(3);
    z = h - (Rp / 2) * (sqrt(3) * n_x + n_y);
    len = sqrt(z * z + y * y + x * x);
    theta = acos((len * len + Llow * Llow - Lup * Lup) / (2 * len * Llow)) + acos((sqrt(3) * x + y) / (-2 * len));
  }
  else if (mtrNum == 2) {
    x = (sqrt(3) / 2) * (Rb - Rp * (1 - (n_x * n_x - sqrt(3) * n_x * n_y) / (n_z + 1)));
    y = -x / sqrt(3);
    z = h + (Rp / 2) * (sqrt(3) * n_x - n_y);
    len = sqrt(z * z + y * y + x * x);
    theta = acos((len * len + Llow * Llow - Lup * Lup) / (2 * len * Llow)) + acos((sqrt(3) * x - y) / (2 * len));
  }


  return theta * 180 / PI; // Return angle in degrees
}


// Function to calculate and set motor positions based on platform position
void moveToPosition(float h, float nx, float ny) {
  // Calculate motor angles using inverse kinematics
  for (int i = 0; i < 3; i++) {
    double angle = MtrAng(i, h, nx, ny);
   
    // Convert angle to steps (map from degrees to steps)
    motorPositions[i] = round((angOrig - angle) * angToStep);
   
    // Constrain to safe motor range
    motorPositions[i] = constrain(motorPositions[i], 15, 550);
  }
 
  if (ballDetected) {
    // Set calculated speed and acceleration for dynamic control
    for (int i = 0; i < 3; i++) {
      prevMotorSpeed[i] = motorSpeed[i];
      // Calculate speed based on distance to target
      float distance = abs(steppers[i].currentPosition() - motorPositions[i]);
      motorSpeed[i] = distance * speedKs;
     
      /*TSPoint point = ts.getPoint();
        offsetPoint(point, 500, 500);
        float distance = sqrt(sq(point.x) + sq(point.y));
        motorSpeed[i] = distance * speedKs; */
     
      // Filter speed changes to avoid jerky movements
      motorSpeed[i] = constrain(motorSpeed[i], prevMotorSpeed[i] - 200, prevMotorSpeed[i] + 200);
      motorSpeed[i] = constrain(motorSpeed[i], 0, 2000);
     
      // Set speed and acceleration
      steppers[i].setMaxSpeed(motorSpeed[i]);
      steppers[i].setAcceleration(motorSpeed[i] * 30);
     
      // Set target position
      steppers[i].moveTo(motorPositions[i]);
    }
  }
  else {
    // If no ball detected, use consistent speed for all motors
    for (int i = 0; i < 3; i++) {
      steppers[i].setMaxSpeed(800);
      steppers[i].setAcceleration(4000);
      steppers[i].moveTo(motorPositions[i]);
    }
  }
  // Non-blocking motor movement - one step at a time
  steppers[0].run();
  steppers[1].run();
  steppers[2].run();
}


void setup() {
  Serial.begin(9600);
 
  // Configure motors
  for (int i = 0; i < 3; i++) {
    pinMode(MOTOR_PINS[i][0], OUTPUT); // Enable, step, dir for each motor
    pinMode(MOTOR_PINS[i][1], OUTPUT);
   
 
    // Configure each stepper
    steppers[i].setPinsInverted(true, false, false); // Set direction as inverted
    steppers[i].setMaxSpeed(6000); // Set max speed
    steppers[i].setAcceleration(6000); // Set max acceleration
    steppers[i].setCurrentPosition(0); // Set current position for the motors
  }


  pinMode(en_pin, OUTPUT);
  digitalWrite(en_pin, HIGH);
   


  delay(500);


  digitalWrite(en_pin, LOW);
 
  // Set initial target position
  target[0] = 0;    // x = 0 (center)
  target[1] = 0;    // y = 0 (center)
  target[2] = 4.25; // h = 4.25 (default height)
 
  // Move to home position
  moveToPosition(defaultHeight, 0, 0);
 
  // Run motors until they reach home position
  bool running = true;
  while (running) {
    running = false;
    for (int i = 0; i < 3; i++) {
      if (steppers[i].distanceToGo() != 0) {
        steppers[i].run();
        running = true;
      }
    }
  }
 
  // Initialize timing
  previousTime = millis();
}


void loop() {
  moveToTarget();
}
//============================================================================================================================


// Attempt to move ball to target for one iteration.
void moveToTarget() {
  // Run PID control at a consistent rate
  unsigned long currentTime = millis();
  unsigned long startTime = currentTime;
 
  // Read touchscreen
  TSPoint point = ts.getPoint();
 
  // Check if ball is detected (touchscreen returns non-zero value)
  if (point.x != 0) {
    if (missCount > 3) {
      Serial.print("Ball lost for ");
      Serial.print(missCount);
      Serial.println(" consecutive readings");
    }
    missCount = 0; // Reset counter if ball detected
    ballDetected = true;
   
    // Center touchscreen coordinates (offset from center)
    point = offsetPoint(point, 500, 500);
   
    // Map touchscreen coordinates to movement space
    position[0] = map(point.x, -500, 500, -100, 100) / 100.0; // x coordinate
    position[1] = map(point.y, -500, 500, -100, 100) / 100.0; // y coordinate
   
    // Debug output
    Serial.print("Touch: (");
    Serial.print(point.x);
    Serial.print(",");
    Serial.print(point.y);
    Serial.print(") Mapped: (");
    Serial.print(position[0]);
    Serial.print(",");
    Serial.print(position[1]);
    Serial.println(")");
   
    // Calculate PID for X and Y dimensions
    float deltaTime = (currentTime - previousTime) / 1000.0; // Converts time to seconds
    previousTime = currentTime;
   
    for (int i = 0; i < 2; i++) { // Only for x and y (0 and 1)
      // Calculate error
      lastError[i] = error[i];
      error[i] = target[i] - position[i];
     
      // Integral term with anti-windup
      integral[i] = constrain(integral[i] + error[i] * deltaTime, -maxIntegral, maxIntegral);
     
      // Derivative term
      derivative[i] = (error[i] - lastError[i]) / (deltaTime > 0 ? deltaTime : 0.001);
     
      // Check for NaN or Inf in derivative
      if (isnan(derivative[i]) || isinf(derivative[i])) {
        derivative[i] = 0;
      }
     
      // PID output
      output[i] = kp * error[i] + ki * integral[i] + kd * derivative[i];
     
      // Constrain output to reasonable values
      output[i] = constrain(output[i], -0.25, 0.25);
    }
   
    // Keep z/height constant
    output[2] = 0;
   
    // Debug PID outputs
    /*
    Serial.print("PID Outputs: (");
    Serial.print(output[0]);
    Serial.print(",");
    Serial.print(output[1]);
    Serial.println(")");
    */
  }
  else {
    // Double-check that there is no ball with a small delay
    point = ts.getPoint();
    if (point.x == 0) {
      missCount = missCount++;
      if (missCount == 4) {
        Serial.println("Ball lost, resetting PID");
        // Reset PID values when ball is lost
        for (int i = 0; i < 2; i++) {
          integral[i] = 0, derivative[i] = 0, output[i] = 0;
        }
        ballDetected = false;
      }
    }
  }
 
  // Move platform based on PID outputs
  moveToPosition(defaultHeight, -output[0], -output[1]);
 
  // Ensure consistent loop timing (20ms cycle)
  while (millis() - startTime < 20) {
    // Keep running steppers while waiting
    moveToPosition(defaultHeight, -output[0], -output[1]);
  }
}


// Make ball go in a line pattern. Go to (x1, y1), then go to (x2, y2). "wait" is the time ball has to reach destination in
// milliseconds. "rep" is the number of times the pattern is repeated.
void patternLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t wait, unsigned int rep) {
  uint32_t timeFlag;  // Declare flag to measure duration and determine if wait time was exceeded
  for (unsigned int i = 0; i < rep; i++) {
    // Move to (x1, y1)
    target[0] = x1;
    target[1] = y1;
    target[2] = defaultHeight;
    timeFlag = millis();
    while (millis() - timeFlag < wait) {
      moveToTarget();
    }
    // Move to (x2, y2)
    target[0] = x2;
    target[1] = y2;
    target[2] = defaultHeight;
    timeFlag = millis();
    while (millis() - timeFlag < wait) {
      moveToTarget();
    }
  }
}


// Triangle pattern. Arguments are similar to patternLine
void patternTriangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3, uint32_t wait, unsigned int rep) {
  uint32_t timeFlag;  // Declare flag to measure duration and determine if wait time was exceeded
  for (unsigned int i = 0; i < rep; i++) {
    // Move to (x1, y1)
    target[0] = x1;
    target[1] = y1;
    target[2] = defaultHeight;
    timeFlag = millis();
    while (millis() - timeFlag < wait) {
      moveToTarget();
    }
    // Move to (x2, y2)
    target[0] = x2;
    target[1] = y2;
    target[2] = defaultHeight;
    timeFlag = millis();
    while (millis() - timeFlag < wait) {
      moveToTarget();
    }
    // Move to (x3, y3)
    target[0] = x3;
    target[1] = y3;
    target[2] = defaultHeight;
    timeFlag = millis();
    while (millis() - timeFlag < wait) {
      moveToTarget();
    }
  }
}
void patternPolygon(int16_t coords[][2], uint32_t wait, unsigned int rep) {
  uint32_t timeFlag;  // Declare flag to measure duration and determine if wait time was exceeded
  for (unsigned int i = 0; i < rep; i++) {
    for (unsigned int j = 0; j < sizeof(coords) / sizeof(coords[0]); j++) {
      // Move to (coords[j][0], coords[j][1])
      target[0] = coords[j][0];
      target[1] = coords[j][1];
      target[2] = defaultHeight;
      timeFlag = millis();
      while (millis() - timeFlag < wait) {
        moveToTarget();
      }
    }
  }
}
