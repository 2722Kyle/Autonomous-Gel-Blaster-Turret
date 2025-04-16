// ***** Sensor Libraries *****
#include <Pixy2.h>
#include <RPLidar.h>
#define RPLIDAR_MOTOR 8  // PWM pin for RPLIDAR motor control

// Create instances
Pixy2 pixy;
RPLidar lidar;

// Define pin assignments
const int MOTOR_CW_PIN = 9;    //In1 Pin on Driver (CW)
const int MOTOR_CCW_PIN = 10;  //In2 Pin on Driver (CCW)
const int ACTUATOR_UP_PIN = 4;
const int ACTUATOR_DOWN_PIN = 5;
const int TRIGGER_PIN_HIGH = 11;
const int TRIGGER_PIN_LOW = 12;

// User Defined Lidar Constants
const int MIN_D = 50;        // User defined distance minimum
const int MAX_D = 1000;      // User defined distance maximum
const int MIN_A = 0;         // User defined angle minimum
const int MAX_A = 360;       // User defined angle maximum
const int MAX_POINTS = 500;  // User defined Max Points Scanned

// Lidar Variables
float avgDistance = 0;
float avgAngle = 0;
float maxDistance = 10000;
float angleAtMinDist = MIN_A;

// Target information
float targetAngle = 0;
float targetDistance = 0;

// Variables
float currentTurretAngle = 0;
int PWM = 0; // Motor PWM



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
void moveTurret();
// void moveTilt();


void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial1.begin(115200);  // Use Serial1 for RPLidar
  lidar.begin(Serial1);
  pinMode(RPLIDAR_MOTOR, OUTPUT);

  pixy.init();

  pinMode(TRIGGER_PIN_HIGH, OUTPUT);
  digitalWrite(TRIGGER_PIN_HIGH, LOW);

  Serial.println("System initialized. Entering scan mode...");
}

void loop() {
  switch (currentState) {
    case SCAN:
      {
        Serial.println("State = SCAN");
        float runningSumDistance = 0.0;
        float runningSumAngle = 0.0;
        int validPointCount = 0;

        while (validPointCount < MAX_POINTS) {
          if (IS_OK(lidar.waitPoint())) {
            float distance = lidar.getCurrentPoint().distance;
            float angle = lidar.getCurrentPoint().angle;

            // Only consider valid scan points (exclude startBit markers and out-of-range data)
            if (!lidar.getCurrentPoint().startBit && distance >= MIN_D && distance <= MAX_D) {

              LIDAR_printData(angle, distance);
              Serial.println(validPointCount);
              runningSumDistance += distance;
              runningSumAngle += angle;
              validPointCount++;
            }
          } else {
            // Recovery logic for failed communication
            analogWrite(RPLIDAR_MOTOR, 0);

            rplidar_response_device_info_t info;
            if (IS_OK(lidar.getDeviceInfo(info, 100))) {
              lidar.startScan();
              analogWrite(RPLIDAR_MOTOR, 255);
              delay(1000);
            }
          }
        }

        // --- After collecting data, compute averages ---
        avgDistance = runningSumDistance / validPointCount;
        avgAngle = runningSumAngle / validPointCount;

        Serial.print("Average Distance (mm): ");
        Serial.println(avgDistance);
        Serial.print("Average Angle (deg): ");
        Serial.println(avgAngle);


        delay(5000);
        currentState = MOVE;
        break;
      }

    case MOVE:
      {
        Serial.println("State = MOVE");

        //gradually adjust toward target positions
        moveTurret(avgAngle);
        Serial.print("Moving. Angle:");
        Serial.println(avgAngle);
        //When near target, move to validation
        // if (abs(currentTurretAngle - avgAngle) < 5) {
        currentState = VALIDATE;
        // }



        break;
      }

    case VALIDATE:
      {
        Serial.println("State = VALIDATE");

        // Pixy validation logic goes here (currently bypassed)
        delay(1000);
        currentState = SHOOT;
        break;
      }

    case SHOOT:
      {
        Serial.println("State = SHOOT");

        // digitalWrite(TRIGGER_PIN, HIGH);
        // delay(100);
        // digitalWrite(TRIGGER_PIN, LOW);

        delay(1000);
        currentState = RESET;
        break;
      }

    case RESET:
      {
        Serial.println("Resetting. Returning to SCAN.");
        delay(1000);
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

void LIDAR_printData(float angle, float distance) {  // Print lidar data for debugging
  Serial.print("dist: ");
  Serial.print(distance);
  Serial.print("    angle: ");
  Serial.println(angle);
}

void moveTurret(float targetTurretAngle) {  // Move Turret About Z (Yaw)
  float target = (targetTurretAngle <= 180.0f ? targetTurretAngle : targetTurretAngle - 360.0f);
  float error = target - currentTurretAngle;

  if (fabs(error) < 3) {
    analogWrite(MOTOR_CW_PIN, 0);
    analogWrite(MOTOR_CCW_PIN, 0);
    currentTurretAngle = targetTurretAngle;
    return;
  }
  // int PWM = constrain(abs(error) * 3, 50, 255);
  PWM = 90;
  if (error > 0 && targetTurretAngle > 0 && targetTurretAngle < 180) {
    analogWrite(MOTOR_CW_PIN, PWM);
    analogWrite(MOTOR_CCW_PIN, 0);
    Serial.println("Turning Clockwise...");
    delay(1000);
  } else {
    analogWrite(MOTOR_CW_PIN, 0);
    analogWrite(MOTOR_CCW_PIN, PWM);
    Serial.println("Turning Counter Clockwise...");
    delay(1000);
  }
  currentTurretAngle += 15;
  delay(100);
}

// void moveTilt(int direction, float angle){ // Move Turret About X (Pitch)
//   int error = targetTiltAngle - currentTiltAngle;
//   if (abs(error) < 3){
//     analogWrite(tilt_up_pin, 0);
//     analogWrite(tilt_down_pin, 0);
//     return;
//   }
//   int PWM = constrain(abs(error)*3, 50, 255);
//   if (error > 0){
//     analogWrite(tilt_down_pin, PWM);
//     analogWrite(tilt_up_pin, 0);
//   }
//   else{
//     analogWrite(tilt_down_pin, 0);
//     analogWrite(tilt_up_pin, PWM);
//   }
//   currentTiltAngle += error / 10;
//   delay(100);
// }
