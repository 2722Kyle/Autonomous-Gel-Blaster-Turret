// ***** Sensor Libraries *****
#include <Pixy2.h>
#include <RPLidar.h>

#define RPLIDAR_MOTOR 8  // PWM pin for RPLIDAR motor control

// Create instances
Pixy2 pixy;
RPLidar lidar;

// Define pin assignments
const int MOTOR_CW_PIN = 10;  //In1 Pin on Driver (CW)
const int MOTOR_CCW_PIN = 9;  //In2 Pin on Driver (CCW)
const int ACTUATOR_UP_PIN = 4;
const int ACTUATOR_DOWN_PIN = 5;
const int TRIGGER_PIN_POS = 2;
const int TRIGGER_PIN_NEG = 3;

int validatecounter = 0;

// Variables
const int GUN_BARREL_OFFSET_Y = 18;  // Barrel offset from camera center

unsigned long alignmentStartTime = 0;
const unsigned long alignmentHoldTime = 3000;  // time in ms to hold steady before shooting


const int PIXY_CENTER_X = 158;
const int PIXY_CENTER_Y = 116;
const int X_THRESHOLD = 25;  // Keep equal to or below 25. Default 25
const int Y_THRESHOLD = 1;   //
int panPos = 485;
int tiltPos = 425;
const float PAN_SWEEP_DEG = 180.0f;
const float TILT_SWEEP_DEG = 90.0f;

// State machine
enum State {
  SCAN,
  MOVE,
  VALIDATE,
  SHOOT,
  RESET
};

State currentState = SCAN;

// Function prototypes
void LIDAR_printData(float angle, float distance);

void setup() {
  Serial.begin(115200);
  delay(1000);
  
    Serial1.begin(115200);  // Use Serial1 for RPLidar
  lidar.begin(Serial1);
  pinMode(RPLIDAR_MOTOR, OUTPUT);

  pixy.init();
  pixy.setServos(panPos, tiltPos);
  pinMode(TRIGGER_PIN_POS, OUTPUT);
  pinMode(TRIGGER_PIN_NEG, OUTPUT);

  Serial.println("System initialized. Entering scan mode...");
}

float minDistance = 800;
float angleAtMin = 0;
float currentdist = 0;
float currentangle = 0;
//float distance = 0;
//float angle = 0;
void loop() {
  switch (currentState) {
    case SCAN:
      {
        if (IS_OK(lidar.waitPoint())) {

          float distance = lidar.getCurrentPoint().distance;
          float angle = lidar.getCurrentPoint().angle;

          if (lidar.getCurrentPoint().startBit) {

            if (angleAtMin >= 51 && angleAtMin <= 179) {
              analogWrite(MOTOR_CW_PIN, 110);  // Turn right at full speed
              analogWrite(MOTOR_CCW_PIN, 0);   // Make sure left motor is off
              Serial.println("Turning Right");
              validatecounter = 0;
            } else if (angleAtMin >= 10 && angleAtMin <= 50) {
              analogWrite(MOTOR_CW_PIN, 80);  // Turn right at full speed
              analogWrite(MOTOR_CCW_PIN, 0);  // Make sure left motor is off
              Serial.println("Turning Right");
              validatecounter = 0;
              //delay(10);
            } else if (angleAtMin >= 181 && angleAtMin <= 299) {
              analogWrite(MOTOR_CCW_PIN, 110);  // Turn left at full speed
              analogWrite(MOTOR_CW_PIN, 0);     // Make sure right motor is off
              Serial.println("Turning Left");
              validatecounter = 0;
              //delay(10);
            } else if (angleAtMin >= 300 && angleAtMin <= 350) {
              analogWrite(MOTOR_CCW_PIN, 110);  // Turn left at full speed
              analogWrite(MOTOR_CW_PIN, 0);     // Make sure right motor is off
              Serial.println("Turning Left");
              validatecounter = 0;
              //delay(10);
            }

            else {
              // If angle is exactly 180 or out of bounds, stop both
              analogWrite(MOTOR_CW_PIN, 0);
              analogWrite(MOTOR_CCW_PIN, 0);
              Serial.println("No Turn");
              validatecounter++;
              Serial.println(validatecounter);
            }

            minDistance = 1000;
            angleAtMin = 0;

            if (validatecounter >= 20) {
              currentState = VALIDATE;
              Serial.println("reset validate counter");
            }

          } else {
            if (distance > 0 && distance < minDistance) {

              if (angleAtMin == 0) {
                minDistance = distance;
                angleAtMin = angle;
              } else if (abs(angle - angleAtMin) <= 40) {
                minDistance = distance;
                angleAtMin = angle;
              } else {
                Serial.println("Angle out of range. Ignoring.");
              }
              Serial.print("angle is: ");
              Serial.print(angleAtMin);
              Serial.print(" distance is: ");
              Serial.println(minDistance);
            }
          }
        } else {
          analogWrite(RPLIDAR_MOTOR, 0);  //stop the rplidar motor

          // try to detect RPLIDAR...
          rplidar_response_device_info_t info;
          if (IS_OK(lidar.getDeviceInfo(info, 100))) {
            lidar.startScan();
            analogWrite(RPLIDAR_MOTOR, 255);
            delay(1000);
          }
        }
        break;
      }

    case VALIDATE:
      {

        Serial.println("enter validate");
        pixy.ccc.getBlocks();
        if (pixy.ccc.numBlocks > 0) {
          auto &b = pixy.ccc.blocks[0];
          int errorX = b.m_x - PIXY_CENTER_X;
          int desiredY = PIXY_CENTER_Y + GUN_BARREL_OFFSET_Y;
          int errorY = b.m_y - desiredY;
          errorX = -errorX;

          Serial.print("Error in X: ");
          Serial.print(errorX);
          Serial.print(", Error in Y: ");
          Serial.println(errorY);

          /// motor deviation ///
          if (abs(errorX) > X_THRESHOLD) {
            if (errorX < -50) {
              analogWrite(MOTOR_CW_PIN, 110);
              analogWrite(MOTOR_CCW_PIN, 0);
            }

            else if (errorX > 50) {
              analogWrite(MOTOR_CW_PIN, 0);
              analogWrite(MOTOR_CCW_PIN, 110);
            } else {
              analogWrite(MOTOR_CW_PIN, 0);
              analogWrite(MOTOR_CCW_PIN, 0);
            }
          }
          /// actuator deviation ///
          if (abs(errorY) > Y_THRESHOLD) {
            if (errorY < -8) {
              analogWrite(ACTUATOR_UP_PIN, 0);
              analogWrite(ACTUATOR_DOWN_PIN, 255);
            } else if (errorY > 8) {
              analogWrite(ACTUATOR_UP_PIN, 255);
              analogWrite(ACTUATOR_DOWN_PIN, 0);
            } else {
              analogWrite(ACTUATOR_UP_PIN, 0);
              analogWrite(ACTUATOR_DOWN_PIN, 0);
            }
          }
          // if no motor deviation and no actuator deviation. Shoot
          if (abs(errorX) <= 35 && abs(errorY) <= 20) {
            if (alignmentStartTime == 0) {
              alignmentStartTime = millis();  // start timing
            } else if (millis() - alignmentStartTime >= alignmentHoldTime) {
              currentState = SHOOT;
              alignmentStartTime = 0;  // reset allignmentStartTime for next loop
            }
          } else {
            // Reset if target goes out of bounds
            alignmentStartTime = 0;
          }
        } else {
          Serial.println("Target lost. Restarting Scan...");
          currentState = SCAN;
          analogWrite(MOTOR_CW_PIN, 0);
          analogWrite(MOTOR_CCW_PIN, 0);
          analogWrite(ACTUATOR_UP_PIN, 0);
          analogWrite(ACTUATOR_DOWN_PIN, 0);
        }
        break;
      }

    case SHOOT:
      {
        Serial.println("State = SHOOT");
        digitalWrite(TRIGGER_PIN_POS, LOW);
        digitalWrite(TRIGGER_PIN_NEG, HIGH);
        Serial.println("FIRING...");
        delay(250);
        digitalWrite(TRIGGER_PIN_POS, LOW);
        digitalWrite(TRIGGER_PIN_NEG, LOW);
        delay(250);
        currentState = SCAN;
        break;
      }


    default:
      {
        Serial.println("Invalid state. Resetting.");
        delay(1000);
        currentState = SCAN;
        break;
      }
  }
}