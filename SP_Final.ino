// ***** Sensor Libraries *****
#include <Pixy2.h>
#include <RPLidar.h>

#define RPLIDAR_MOTOR 8  // PWM pin for RPLIDAR motor control

// Create instances
Pixy2 pixy;
RPLidar lidar;

// Define pin assignments
const int MOTOR_CW_PIN = 10;    //In1 Pin on Driver (CW)
const int MOTOR_CCW_PIN = 9;  //In2 Pin on Driver (CCW)
const int ACTUATOR_UP_PIN = 4;
const int ACTUATOR_DOWN_PIN = 5;
const int TRIGGER_PIN_HIGH = 11;
const int TRIGGER_PIN_LOW = 12;


const float TOLERANCE = 2.0;
const float MS_PER_DEGREE = 5;
const int MOTOR_SPEED = 150;
const int MOTOR_CCW_SPEED = 175;

// User Defined Lidar Constants
const int MIN_D = 50;        // User defined distance minimum
const int MAX_D = 1000;      // User defined distance maximum
const int MIN_A = 0;         // User defined angle minimum
const int MAX_A = 360;       // User defined angle maximum
const int MAX_POINTS = 250;  // User defined Max Points Scanned

// Lidar Variables
float avgDistance = 0;
float avgAngle = 0;
float maxDistance = 10000;
float angleAtMinDist = MIN_A;

// Target information
float targetAngle = 0;
float targetDistance = 0;

// Variables
float currentTurretAngle = 0.0;



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
        if (IS_OK(lidar.waitPoint())) {
          //perform data processing here... 
          float distance = lidar.getCurrentPoint().distance;
          float angle = lidar.getCurrentPoint().angle;
    
          if (lidar.getCurrentPoint().startBit) {
            // a new scan, display the previous data...
             move360motor(angleAtMinDist, minDistance);
       
             minDistance = 1000;
             angleAtMinDist = 0;

             Serial.print("angle is: ");
             Serial.print(angleAtMinDist);
             Serial.print(" distance is: ");
             Serial.print(minDistance);


          } else {
               if ( distance > 0 &&  distance < minDistance) {
                  minDistance = distance;
                  angleAtMinDist = angle;
                  Serial.print("angle is: ");
                  Serial.print(angleAtMinDist);
                  Serial.print(" distance is: ");
                  Serial.print(minDistance);
                 }
            }
          } else {
            analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    
            // try to detect RPLIDAR... 
            rplidar_response_device_info_t info;
            if (IS_OK(lidar.getDeviceInfo(info, 100))) {
               //detected...
               lidar.startScan();
               analogWrite(RPLIDAR_MOTOR, 255);
               delay(1000);
            }
  }
        // float runningSumDistance = 0.0;
        // float runningSumAngle = 0.0;
        // int validPointCount = 0;

        // while (validPointCount < MAX_POINTS) {
        //   if (IS_OK(lidar.waitPoint())) {
        //     float distance = lidar.getCurrentPoint().distance;
        //     float angle = lidar.getCurrentPoint().angle;

        //     // Only consider valid scan points (exclude startBit markers and out-of-range data)
        //     if (!lidar.getCurrentPoint().startBit && distance >= MIN_D && distance <= MAX_D) {

        //       LIDAR_printData(angle, distance);
        //       Serial.println(validPointCount);
        //       runningSumDistance += distance;
        //       runningSumAngle += angle;
        //       validPointCount++;
        //     }
        //   } else {
        //     // Recovery logic for failed communication
        //     analogWrite(RPLIDAR_MOTOR, 0);

        //     rplidar_response_device_info_t info;
        //     if (IS_OK(lidar.getDeviceInfo(info, 100))) {
        //       lidar.startScan();
        //       analogWrite(RPLIDAR_MOTOR, 255);
        //       delay(1000);
        //     }
        //   }
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

void move360motor(float angle, float distance)
{
      if (angle >= 10 && angle <= 179) {
    analogWrite(motorRight, 90);  // Turn right at full speed
    analogWrite(motorLeft, 0);     // Make sure left motor is off
    Serial.println("Turning Right");
   } 
    else if (angle >= 181 && angle <= 350) {
    analogWrite(motorLeft, 90);   // Turn left at full speed
    analogWrite(motorRight, 0);    // Make sure right motor is off
    Serial.println("Turning Left");
   } 
    else {
    // If angle is exactly 180 or out of bounds, stop both
    analogWrite(motorRight, 0);
    analogWrite(motorLeft, 0);
    Serial.println("No Turn");
   }

void moveTurret(float targetTurretAngle) {  // Move Turret About Z (Yaw)
  float target = (targetTurretAngle <= 180.0f ? targetTurretAngle : targetTurretAngle - 360.0f);
  float error = target - currentTurretAngle;

  if (fabs(error) < TOLERANCE) {
    analogWrite(MOTOR_CW_PIN, 0);
    analogWrite(MOTOR_CCW_PIN, 0);
    return;
  }
  int runTime = (int)(fabs(error) * MS_PER_DEGREE);
  if (error > 0) {
    analogWrite(MOTOR_CW_PIN, MOTOR_SPEED);
    analogWrite(MOTOR_CCW_PIN, 0);
    Serial.println("Turning Clockwise...");
  } else {
    analogWrite(MOTOR_CW_PIN, 0);
    analogWrite(MOTOR_CCW_PIN, MOTOR_CCW_SPEED);
    Serial.println("Turning Counter Clockwise...");
  }
  Serial.print(runTime);
  Serial.println("ms");
  delay(runTime);
  analogWrite(MOTOR_CW_PIN, 0);
  analogWrite(MOTOR_CCW_PIN, 0);
  currentTurretAngle = target;
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
