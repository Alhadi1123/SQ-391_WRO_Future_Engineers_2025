#include <ArduinoQueue.h> // Library for creating and managing queues
#include <NewPing.h> // Library for controlling ultrasonic sensors
#include <Servo.h> // Library for controlling servo motors
#include <math.h> // Math library for mathematical functions
#include <MPU6050_light.h> // Library for MPU6050 accelerometer and gyroscope

// Define pin numbers for ultrasonic sensors
#define TRIGGER_PIN_LEFT1  40
#define ECHO_PIN_LEFT1  41
#define TRIGGER_PIN_LEFT2  29
#define ECHO_PIN_LEFT2  28
#define TRIGGER_PIN_RIGHT1  32
#define ECHO_PIN_RIGHT1  33
#define TRIGGER_PIN_RIGHT2  48
#define ECHO_PIN_RIGHT2  49
#define TRIGGER_PIN_FRONT 44
#define ECHO_PIN_FRONT  45
#define TRIGGER_PIN_DIAG_RIGHT 52
#define ECHO_PIN_DIAG_RIGHT 53
#define TRIGGER_PIN_DIAG_LEFT 30
#define ECHO_PIN_DIAG_LEFT 31
#define TRIGGER_PIN_BACK 24
#define ECHO_PIN_BACK 25
#define TRIGGER_PIN_BACKLEFT 3
#define ECHO_PIN_BACKLEFT 4
#define TRIGGER_PIN_BACKRIGHT 27
#define ECHO_PIN_BACKRIGHT 26

// Define constants
#define MAX_DISTANCE 300 // Maximum distance to ping (in centimeters)
#define RIGHT_EN 35 // Enable pin for right motor
#define LEFT_EN 34 // Enable pin for left motor
#define RIGHT_PWM 3 // PWM pin for right motor speed control
#define LEFT_PWM 2 // PWM pin for left motor speed control

// Initialize MPU6050, Servos, and NewPing objects
MPU6050 mpu(Wire); // Create MPU6050 object for IMU
Servo leftServo, rightServo; // Create Servo objects for left and right steering
NewPing sonarLeft1(TRIGGER_PIN_LEFT1, ECHO_PIN_LEFT1, MAX_DISTANCE); // Initialize ultrasonic sensor for left1
NewPing sonarLeft2(TRIGGER_PIN_LEFT2, ECHO_PIN_LEFT2, MAX_DISTANCE); // Initialize ultrasonic sensor for left2
NewPing sonarRight1(TRIGGER_PIN_RIGHT1, ECHO_PIN_RIGHT1, MAX_DISTANCE); // Initialize ultrasonic sensor for right1
NewPing sonarRight2(TRIGGER_PIN_RIGHT2, ECHO_PIN_RIGHT2, MAX_DISTANCE); // Initialize ultrasonic sensor for right2
NewPing sonarFront(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE); // Initialize ultrasonic sensor for front
NewPing sonarDiagRight(TRIGGER_PIN_DIAG_RIGHT, ECHO_PIN_DIAG_RIGHT, MAX_DISTANCE); // Initialize ultrasonic sensor for diagonal right
NewPing sonarDiagLeft(TRIGGER_PIN_DIAG_LEFT, ECHO_PIN_DIAG_LEFT, MAX_DISTANCE); // Initialize ultrasonic sensor for diagonal left
NewPing sonarBack(TRIGGER_PIN_BACK, ECHO_PIN_BACK, MAX_DISTANCE); // Initialize ultrasonic sensor for back
NewPing sonarBackLeft(TRIGGER_PIN_BACKLEFT, ECHO_PIN_BACKLEFT, MAX_DISTANCE); // Initialize ultrasonic sensor for back left
NewPing sonarBackRight(TRIGGER_PIN_BACKRIGHT, ECHO_PIN_BACKRIGHT, MAX_DISTANCE); // Initialize ultrasonic sensor for back right

// Variables for storing sensor readings
float distanceLeft1, distanceLeft2, distanceRight1, distanceRight2;
float distanceFront, distanceDiagRight, distanceDiagLeft;
float distanceBack, distanceBackLeft, distanceBackRight;

// Setup function to initialize sensors and servos
void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud rate
  
  // Initialize servos
  leftServo.attach(9); // Attach left servo to pin 9
  rightServo.attach(10); // Attach right servo to pin 10

  // Initialize MPU6050
  Wire.begin(); // Begin I2C communication
  mpu.begin(); // Initialize MPU6050
  mpu.calcGyroOffsets(true); // Calibrate gyro offsets and wait for it to be done

  // Set motor control pins as outputs
  pinMode(RIGHT_EN, OUTPUT);
  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
}

// Function to read distances from all ultrasonic sensors
void readDistances() {
  distanceLeft1 = sonarLeft1.ping_cm(); // Read distance from left1 sensor
  distanceLeft2 = sonarLeft2.ping_cm(); // Read distance from left2 sensor
  distanceRight1 = sonarRight1.ping_cm(); // Read distance from right1 sensor
  distanceRight2 = sonarRight2.ping_cm(); // Read distance from right2 sensor
  distanceFront = sonarFront.ping_cm(); // Read distance from front sensor
  distanceDiagRight = sonarDiagRight.ping_cm(); // Read distance from diagonal right sensor
  distanceDiagLeft = sonarDiagLeft.ping_cm(); // Read distance from diagonal left sensor
  distanceBack = sonarBack.ping_cm(); // Read distance from back sensor
  distanceBackLeft = sonarBackLeft.ping_cm(); // Read distance from back left sensor
  distanceBackRight = sonarBackRight.ping_cm(); // Read distance from back right sensor
}

// Function to set motor speeds
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  analogWrite(LEFT_PWM, leftSpeed); // Set speed for left motor
  analogWrite(RIGHT_PWM, rightSpeed); // Set speed for right motor
}
void loop() {
    // Main loop where sensor readings and control logic are processed
    sensorRead(); // Update sensor readings from all sensors

    if (servoSet) {
        rightServo.write(servoPosRight); // Set the right servo position
        leftServo.write(servoPosLeft); // Set the left servo position
        servoSet = false; // Reset the servoSet flag
        mpu.update(); // Update MPU6050 sensor data
        gyroRefMpu = mpu.getAngleZ(); // Reference the Z-axis angle from the MPU6050
    }

    // Main navigation and wall avoidance logic
    if (distanceFront > 90) {  // If front distance is greater than 90 cm
        if (realDistanceLeft < setPoint) {
            runMotor(speedBeforeTurn); // Move forward at speed before turn
            controlDistanceLeft(realDistanceLeft); // Adjust left distance
        }

        if (realDistanceRight < setPoint) {
            runMotor(speedBeforeTurn); // Move forward at speed before turn
            controlDistanceRight(realDistanceRight); // Adjust right distance
        }

        if (realDistanceLeft > setPoint && realDistanceRight > setPoint && angleCounterLeft == 1) {
            runMotor(speedCurrent); // Move forward at current speed
            if (angleCounterLeft > 0) {
                angle += angleCalc(distanceLeft1, distanceLeft2) * 180 / acos(-1); // Calculate and accumulate angle
                angleCounterLeft--; // Decrement angle counter
            }
            if (angleCounterLeft == 0) {
                gyroRefMpu += angle / 5; // Adjust gyro reference
                angleCounterLeft = -1; // Reset angle counter
                angle = 0; // Reset angle
            }
            mpu.update(); // Update MPU6050 sensor data
            pidAngle(mpu.getAngleZ()); // PID control based on current angle
        }

        if (realDistanceLeft > setPoint && realDistanceRight > setPoint && angleCounterRight == 1) {
            runMotor(speedCurrent); // Move forward at current speed
            if (angleCounterRight > 0) {
                angle += angleCalc(distanceRight1, distanceRight2) * 180 / acos(-1); // Calculate and accumulate angle
                angleCounterRight--; // Decrement angle counter
            }
            if (angleCounterRight == 0) {
                gyroRefMpu -= angle / 5; // Adjust gyro reference
                angleCounterRight = -1; // Reset angle counter
                angle = 0; // Reset angle
            }
            mpu.update(); // Update MPU6050 sensor data
            pidAngle(mpu.getAngleZ()); // PID control based on current angle
        }

        if (realDistanceLeft > setPoint && realDistanceRight > setPoint && angleCounterLeft == 0 && angleCounterRight == 0) {
            runMotor(speedBeforeTurn); // Move forward at speed before turn
            turnLeft(distanceLeft1, distanceLeft2); // Execute left turn
        }

        if (distanceBack < 100 && (distanceDiagRight > 100 || distanceDiagLeft > 100)) {
            runMotor(120); // Move forward at speed 120
            while (distanceBack < 100 && (distanceDiagRight > 100 || distanceDiagLeft > 100)) {
                sensorRead(); // Continuously read sensors
                mpu.update(); // Update MPU6050 sensor data
                pidAngle(mpu.getAngleZ()); // PID control based on current angle
            }
        }

        if (distanceBack < 100 && (distanceDiagLeft > 100 || distanceDiagRight > 100)) {
            runMotor(120); // Move forward at speed 120
            while (distanceBack < 100 && (distanceDiagLeft > 100 || distanceDiagRight > 100)) {
                sensorRead(); // Continuously read sensors
                mpu.update(); // Update MPU6050 sensor data
                pidAngle(mpu.getAngleZ()); // PID control based on current angle
            }
        }
    }

    // Logic for when front distance is between 0 and 90 cm
    if (distanceFront < 90 && distanceFront > 0) {
        runMotor(speedBeforeTurn); // Move forward at speed before turn

        if (realDistanceLeft < setPoint) {
            controlDistanceLeft(realDistanceLeft); // Adjust left distance
        }

        if (realDistanceRight < setPoint) {
            controlDistanceRight(realDistanceRight); // Adjust right distance
        }

        if (realDistanceLeft > setPoint && realDistanceRight > setPoint && angleCounterLeft == 1) {
            mpu.update(); // Update MPU6050 sensor data
            pidAngle(mpu.getAngleZ()); // PID control based on current angle
        }

        if (realDistanceLeft > setPoint && realDistanceRight > setPoint && angleCounterRight == 1) {
            mpu.update(); // Update MPU6050 sensor data
            pidAngle(mpu.getAngleZ()); // PID control based on current angle
        }

        if (realDistanceLeft > setPoint && realDistanceRight > setPoint && angleCounterLeft == 0 && angleCounterRight == 0) {
            mpu.update(); // Update MPU6050 sensor data
            pidAngle(mpu.getAngleZ()); // PID control based on current angle
        }

        if (angleCounterRight == 1) {
            if ((distanceDiagLeft > 100 || distanceDiagRight > 100) || (distanceFront < 50 && distanceFront > 0 && distanceBack > 210)) {
                runMotor(speedTurn); // Move forward at turn speed
                turnLeftGyro(); // Execute left turn using gyro
            }
        }

        if (angleCounterLeft == 1) {
            if (distanceDiagRight > 100 || distanceDiagLeft > 100 || (distanceFront < 50 && distanceFront > 0 && distanceBack > 210)) {
                runMotor(speedTurn); // Move forward at turn speed
                turnRightGyro(); // Execute right turn using gyro
            }
        }

        if (angleCounterLeft == 0 && angleCounterRight == 0) {
            if (distanceDiagLeft > 100 || distanceDiagRight > 100) {
                turnLeftGyro(); // Execute left turn using gyro
            }
            if (distanceDiagRight > 100 || distanceDiagLeft > 100) {
                turnRightGyro(); // Execute right turn using gyro
            }
        }
    }

    // Logic for specific sensor conditions
    if (sensorCounter >= 12) {
        sensorRead(); // Read sensors
        while (distanceFront > 200 || distanceBack < 110) {
            if (angleCounterLeft == 1) {
                if (realDistanceLeft < setPoint) {
                    controlDistanceLeft(realDistanceLeft); // Adjust left distance
                } else if (realDistanceLeft > setPoint) {
                    mpu.update(); // Update MPU6050 sensor data
                    pidAngle(mpu.getAngleZ()); // PID control based on current angle
                }
            } else {
                if (realDistanceRight < setPoint) {
                    controlDistanceRight(realDistanceRight); // Adjust right distance
                } else if (realDistanceRight > setPoint) {
                    mpu.update(); // Update MPU6050 sensor data
                    pidAngle(mpu.getAngleZ()); // PID control based on current angle
                }
            }
            sensorRead(); // Continuously read sensors
        }
        stopMotor(0); // Stop the vehicle
        delay(100000000000); // Delay for a very long time (essentially stops further execution)
    }
}


// Functions definition
void sensorRead() {
    // Reads sensor data from all ultrasonic sensors
    // This function reads the ping duration from each ultrasonic sensor and converts it to distance in centimeters.
    
    durationFront = sonarFront.ping(); // Get ping duration from front sensor
    distanceFront = durationFront / 2 / 29.1; // Convert duration to distance (cm) for front sensor

    durationLeft1 = sonarLeft1.ping(); // Get ping duration from left1 sensor
    distanceLeft1 = durationLeft1 / 2 / 29.1; // Convert duration to distance (cm) for left1 sensor

    durationLeft2 = sonarLeft2.ping(); // Get ping duration from left2 sensor
    distanceLeft2 = durationLeft2 / 2 / 29.1; // Convert duration to distance (cm) for left2 sensor

    durationRight1 = sonarRight1.ping(); // Get ping duration from right1 sensor
    distanceRight1 = durationRight1 / 2 / 29.1; // Convert duration to distance (cm) for right1 sensor

    durationRight2 = sonarRight2.ping(); // Get ping duration from right2 sensor
    distanceRight2 = durationRight2 / 2 / 29.1; // Convert duration to distance (cm) for right2 sensor

    durationBack = sonarBack.ping(); // Get ping duration from back sensor
    distanceBack = durationBack / 2 / 29.1; // Convert duration to distance (cm) for back sensor

    durationDiagLeft = sonarDiagLeft.ping(); // Get ping duration from diagonal left sensor
    distanceDiagLeft = durationDiagLeft / 2 / 29.1; // Convert duration to distance (cm) for diagonal left sensor

    durationDiagRight = sonarDiagRight.ping(); // Get ping duration from diagonal right sensor
    distanceDiagRight = durationDiagRight / 2 / 29.1; // Convert duration to distance (cm) for diagonal right sensor

    durationBackLeft = sonarBackLeft.ping(); // Get ping duration from back left sensor
    distanceBackLeft = durationBackLeft / 2 / 29.1; // Convert duration to distance (cm) for back left sensor

    durationBackRight = sonarBackRight.ping(); // Get ping duration from back right sensor
    distanceBackRight = durationBackRight / 2 / 29.1; // Convert duration to distance (cm) for back right sensor
}

void runMotor(int speed) {
    // Set motor speed
    // This function sets the speed for both the left and right motors to move the vehicle forward.
    analogWrite(LEFT_EN, speed); // Set left motor speed
    analogWrite(RIGHT_EN, speed); // Set right motor speed
}

void stopMotor(int speed) {
    // Stop the motors
    // This function stops the motors by setting their speed to the specified value (typically 0 to stop).
    analogWrite(LEFT_EN, speed); // Stop left motor
    analogWrite(RIGHT_EN, speed); // Stop right motor
}

void controlDistanceLeft(double distance) {
    // Adjusts the left servo to maintain a set distance on the left side
    // This function uses a proportional control to adjust the left servo based on the error between the set point and the actual distance.
    double error = setPoint - distance; // Calculate error from the set point
    double adjustment = kpLeft * error; // Proportional adjustment based on the error
    leftServo.write(servoPosLeft + adjustment); // Adjust the left servo position
}

void controlDistanceRight(double distance) {
    // Adjusts the right servo to maintain a set distance on the right side
    // This function uses a proportional control to adjust the right servo based on the error between the set point and the actual distance.
    double error = setPoint - distance; // Calculate error from the set point
    double adjustment = kpRight * error; // Proportional adjustment based on the error
    rightServo.write(servoPosRight + adjustment); // Adjust the right servo position
}

void turnLeft(double dist1, double dist2) {
    // Turn left function
    // Logic for turning left based on distance values
    // Example: slow down the left motor, speed up the right motor
    analogWrite(LEFT_EN, speedCurrent / 2); // Reduce left motor speed to half
    analogWrite(RIGHT_EN, speedCurrent); // Maintain right motor speed
    delay(delayTurnLeft); // Delay for the duration needed to complete the turn
    analogWrite(LEFT_EN, speedCurrent); // Restore left motor speed
    analogWrite(RIGHT_EN, speedCurrent); // Maintain right motor speed
}

void turnLeftGyro() {
    // Turn left using gyroscope data
    // This function rotates the robot by adjusting the servos based on the gyroscope angle to achieve a 90-degree left turn.
    double currentAngle = mpu.getAngleZ(); // Get the current Z-axis angle from the MPU6050
    while (currentAngle < gyroRefMpu + 90) { // Continue turning until a 90-degree turn is achieved
        mpu.update(); // Update MPU6050 sensor data
        currentAngle = mpu.getAngleZ(); // Update current angle
        leftServo.write(servoPosLeft - 10); // Adjust left servo to turn left
        rightServo.write(servoPosRight + 10); // Adjust right servo to turn left
    }
    leftServo.write(servoPosLeft); // Restore left servo position
    rightServo.write(servoPosRight); // Restore right servo position
}

void turnRightGyro() {
    // Turn right using gyroscope data
    // This function rotates the robot by adjusting the servos based on the gyroscope angle to achieve a 90-degree right turn.
    double currentAngle = mpu.getAngleZ(); // Get the current Z-axis angle from the MPU6050
    while (currentAngle > gyroRefMpu - 90) { // Continue turning until a 90-degree turn is achieved
        mpu.update(); // Update MPU6050 sensor data
        currentAngle = mpu.getAngleZ(); // Update current angle
        leftServo.write(servoPosLeft + 10); // Adjust left servo to turn right
        rightServo.write(servoPosRight - 10); // Adjust right servo to turn right
    }
    leftServo.write(servoPosLeft); // Restore left servo position
    rightServo.write(servoPosRight); // Restore right servo position
}
double angleCalc(double dist1, double dist2) {
    // Calculate angle based on distance
    // This function calculates the angle between two distances using the arctangent function.
    // The formula used is atan((dist1 - dist2) / narrowDistFront), where dist1 and dist2 are distances
    // from two sensors, and narrowDistFront is the distance between those two sensors.
    return atan((dist1 - dist2) / narrowDistFront); // Return the calculated angle in radians
}

void pidAngle(double currentAngle) {
    // PID control to maintain the angle
    // This function implements a proportional control (P control) to maintain the desired angle.
    // It calculates the error between the desired angle (gyroRefMpu) and the current angle (currentAngle),
    // then adjusts the servo positions to correct the error.
    double error = gyroRefMpu - currentAngle; // Calculate the difference between desired and current angle
    double adjustment = constKp * error; // Calculate adjustment using proportional gain (constKp)
    leftServo.write(servoPosLeft + adjustment); // Adjust left servo position to correct the angle
    rightServo.write(servoPosRight - adjustment); // Adjust right servo position to correct the angle
}

