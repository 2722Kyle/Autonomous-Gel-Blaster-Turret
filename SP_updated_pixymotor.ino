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
const int TRIGGER_PIN_POS = 11;
const int TRIGGER_PIN_NEG = 12;


const float TOLERANCE = 2.0;
const float MS_PER_DEGREE = 5;
const int MOTOR_SPEED = 175;
const int MOTOR_CCW_SPEED = 175;

// User Defined Lidar Constants
const int MIN_D = 50;        // User defined distance minimum
const int MAX_D = 1000;      // User defined distance maximum
const int MIN_A = 0;         // User defined angle minimum
const int MAX_A = 360;       // User defined angle maximum
const int MAX_POINTS = 100;  // User defined Max Points Scanned

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
float currentTiltAngle = 0.0;

const int PIXY_CENTER_X = 158;
const int PIXY_CENTER_Y = 116;
const int X_THRESHOLD = 25;
const int Y_THRESHOLD = 10; 
int panPos = 485;
int tiltPos = 425;
const float PAN_SWEEP_DEG = 180.0f;
const float TILT_SWEEP_DEG = 90.0f;

int lowest_Xerror = X_THRESHOLD;
int new_Xerror = X_THRESHOLD;
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
  pixy.setServos(panPos, tiltPos);
  pinMode(TRIGGER_PIN_POS, LOW);
  digitalWrite(TRIGGER_PIN_NEG, LOW);

  Serial.println("System initialized. Entering scan mode...");
}

void loop() {
  switch (currentState) {
    case SCAN:
      {
        // Serial.println("State = SCAN");
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
        // }

        // // --- After collecting data, compute averages ---
        // avgDistance = runningSumDistance / validPointCount;
        // avgAngle = runningSumAngle / validPointCount;

        // Serial.print("Average Distance (mm): ");
        // Serial.println(avgDistance);
        // Serial.print("Average Angle (deg): ");
        // Serial.println(avgAngle);


        delay(1000);
        currentState = MOVE;
        break;
      }

    case MOVE:
      {
        // Serial.println("State = MOVE");

        // //gradually adjust toward target positions
        // moveTurret(avgAngle);
        // Serial.print("Moving. Angle: ");
        // Serial.println(avgAngle);
        // //When near target, move to validation
        currentState = VALIDATE;
        break;
      }

    case VALIDATE:
      {
        // Serial.println("State = VALIDATE");
        // //Code to move motors as a function of pixy pan/tilt (polling)
        // int count = pixy.ccc.getBlocks();
        // if (count == 0){
        //   return;
        // }
        // auto &b = pixy.ccc.blocks[0];
        // int errorX = -(b.m_x - PIXY_CENTER_X);
        // int errorY = b.m_y - PIXY_CENTER_Y;

        // int oldPan = panPos;
        // int oldTilt = tiltPos;

        // if (abs(errorX) > X_THRESHOLD){
        //   panPos = constrain(panPos + errorX * 4, 0, 1000);
        // }
        // if (abs(errorY) > Y_THRESHOLD){
        //   tiltPos = constrain(tiltPos + errorY * 4, 0, 1000);
        // }
        // pixy.setServos(panPos, tiltPos);
        // int dPan = panPos - oldPan;
        // int dTilt = tiltPos - oldTilt;

        // float degPan = (dPan / 1000.0f) * PAN_SWEEP_DEG;
        // float degTilt = (dTilt / 1000.0f) * TILT_SWEEP_DEG;

        // if (degPan > 0){
        //   analogWrite(MOTOR_CW_PIN, 100);
        //   analogWrite(MOTOR_CCW_PIN, 0);
        // }
        // else{
        //   analogWrite(MOTOR_CW_PIN, 0);
        //   analogWrite(MOTOR_CCW_PIN, 100);
        // }
        // if (degTilt > 0){
        //   analogWrite(ACTUATOR_UP_PIN, 100);
        //   analogWrite(ACTUATOR_DOWN_PIN, 0);
        // }
        // else{
        //   analogWrite(ACTUATOR_UP_PIN, 0);
        //   analogWrite(ACTUATOR_DOWN_PIN, 100);
        // }
        // Somewhat working code
        pixy.ccc.getBlocks();
        if (pixy.ccc.numBlocks > 0) {
          auto &b = pixy.ccc.blocks[0];
          int errorX = b.m_x - PIXY_CENTER_X;
          int errorY = b.m_y - PIXY_CENTER_Y;
          errorX = -errorX;
          

          Serial.print("Error in X: ");
          Serial.println(errorX);
          Serial.print("Error in Y: ");
          Serial.println(errorY);

          if (abs(errorX) > X_THRESHOLD) {
            // panPos += errorX * 0.75;
            // pixy.setServos(panPos, tiltPos);
            new_Xerror = (abs(errorX));

            Serial.print ("current error is = ");
            Serial.println (new_Xerror);
            if(lowest_Xerror > (abs(new_Xerror))){
              lowest_Xerror = new_Xerror;
            }
            Serial.print("lowest error is");
            Serial.println(lowest_Xerror);

            if (errorX < -40){
              analogWrite(MOTOR_CW_PIN, 0);
              analogWrite(MOTOR_CCW_PIN, 110);
            }
            
            else if (errorX > 40){
              analogWrite(MOTOR_CW_PIN, 110);
              analogWrite(MOTOR_CCW_PIN, 0);
            }
            else {
              analogWrite(MOTOR_CW_PIN, 0);
              analogWrite(MOTOR_CCW_PIN, 0);
            }

            Serial.println("Adjusting Rotation...");
            
          }
          if (abs(errorY) > Y_THRESHOLD){
            tiltPos += errorY * 0.75;
            pixy.setServos(panPos, tiltPos);
            if (errorY < 0){
              analogWrite(ACTUATOR_UP_PIN, 0);
              analogWrite(ACTUATOR_DOWN_PIN, 255);
            }
            if (errorY > 0){
              analogWrite(ACTUATOR_UP_PIN, 255);
              analogWrite(ACTUATOR_DOWN_PIN, 0);
            }
            Serial.println("Adjusting Tilt...");
          }
          if (abs(errorX) <= X_THRESHOLD && abs(errorY) <= Y_THRESHOLD){
            Serial.println("Target aligned...");
            delay(10);
            Serial.println("Beginning to Shoot...");
            currentState = VALIDATE;
          }
        }
        else {
          Serial.println("Target lost. Restarting Scan...");
          currentState = VALIDATE;
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
        digitalWrite(TRIGGER_PIN_POS, HIGH);
        digitalWrite(TRIGGER_PIN_NEG, LOW);
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
