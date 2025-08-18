#include <ArduinoQueue.h>
#include <NewPing.h>
#include <Servo.h>
#include <math.h>
#include <MPU6050_light.h>


double First_reference = 0;                    // Initial reference angle
double gyro_ref = 0;                           // Gyroscope reference angle
double Pre_Time = 0, Cur_Time = 0;             // Time variables for calculations
double theta = 0, theta_mpu = 0, ref_mpu = 0;  // Angle variables
String data;                                   // Data received from sensors
bool Set_Servo = 1, Switch = 50;               // Servo settings and switch state
int start = 0;                                 // Start flag

int pos_R = 51, pos_R_back = 53, pos_L = 57, pos_L_Back = 49, turn_pos = 3, turn_pos2 = 3;  // Motor positions and turn settings

int Delay_Turn_Left = 1000, Delay_Turn_Right = 1150;                                             // Delays for turning
int delay_towide = 1200, delay_tonarrow = 1150;                                                  // Delays for path adjustments
int counter_angle_Right = 0, counter_angle_Left = 0, counter = 0;                                // Counters for angles and general purposes
int dis_narrow_F = 100;                                                                          // Distance for narrow path detection
double Position_L = pos_R - turn_pos, Position_R = pos_R + turn_pos2;                            // Calculated positions
double position_angle = 0;                                                                       // Angle for position calculation
double distance = 0, real_distance = 0, Real_dis_L = 0, Real_dis_R = 0, dis_L1 = 0, dis_L2 = 0;  // Distance variables for sensors
// Distance variables for each sensor
double dis_R1 = 0, dis_R2 = 0, dis_F = 0, dis_B = 0, dis_D2R = 0, dis_BL = 0, dis_D2L = 0, dis_BR = 0;

// Angle calculations and constants for PID control
double angle = 0, Angle = 0, Aangle = 0;
double conskpL = 0.32, conskpR = 0.95, kp = 1, kpl = 1, kpr = 1;  // PID constants
double conskp_NotstartR = 4, conskp_NotstartL = 2.1;              // PID constants when not starting
double Setpoint = 13, Setpoint_min = 7, Setpoint_max = 15;        // PID setpoints for various thresholds
double Setpoint_min_p = 21, Setpoint_max_p = 24;                  // PID setpoints for precise control
double highest_angle = 0.27;                                      // Highest angle threshold for control logic

// Miscellaneous variables
double pos_distance;                                                                                                                                                              // Positional distance variable for calculations
double jbes = 0;                                                                                                                                                                  // Placeholder variable (purpose to be defined)
double duration_F = 0, duration_L1 = 0, duration_L2 = 0, duration_R1 = 0, duration_R2 = 0, duration_B = 0, duration_BR = 0, duration_BL = 0, duration_D2R = 0, duration_D2L = 0;  // Duration of echo responses

// Speed settings for various maneuvers
int spd_now = 50;               // Current speed
int spd_turn_now = 50;          // Speed during turning
int spd_before_turn = 50;       // Speed before turning
int spd_before_turn_back = 50;  // Speed before turning back
int spd_Ob = 50;                // Speed during obstacle navigation
int spd_turn_Back_ob = 50;      // Speed during obstacle navigation turn back

// Angle range definitions for turning calculations
double max_ang = max(pos_R, pos_L) + 30;
double min_ang = min(pos_R, pos_L) - 30;
double min, max;  // Variables for storing minimum and maximum values

// PID controller variables for angle control
double currentTime_angle = 0, dtc = 0, lastTime_angle = 0;                  // Time variables for PID
double current_angle = 0, error_c = 0, output_c = 0, last_angle_error = 0;  // Current angle, error, and output for PID
double kpc = 1, kdc = 1;                                                    // PID coefficients
double reference = 0, reference_1 = 0;                                      // Reference angles for PID
int reference_counter = 5;                                                  // Counter for reference updates
double angle_mean = 0;                                                      // Average angle calculation

// Array to store conditions for cloros at each section
String Conditions[4] = { "", "", "", "" };
// Array to store conditions for parking lot posibilities
bool Pink[4] = { 0, 0, 0, 0 };
// Servo  initializations
Servo myservo;

//define for ultrasonic sensors pins

#define R_EN 35  // Right motor enable pin
#define L_EN 34  // Left motor enable pin
#define R_PWM 3  // Right motor PWM pin
#define L_PWM 2  // Left motor PWM pin


#define TRIGGER_PIN_F 38  // Front sensor trigger pin
#define ECHO_PIN_F 39     // Front sensor echo pin

#define D2R_trigger 50  // Diagonal right trigger pin
#define D2R_echo 51     // Diagonal right echo pin

#define D2L_trigger 41  // Diagonal left trigger pin
#define D2L_echo 40     // Diagonal left echo pin

#define B_trigger 48  // Back sensor trigger pin
#define B_echo 49     // Back sensor echo pin

#define BL_trigger 22  // Back-left sensor trigger pin
#define BL_echo 23     // Back-left sensor echo pin

#define BR_trigger 46  // Back-right sensor trigger pin
#define BR_echo 47     // Back-right sensor echo pin


#define TRIGGER_PIN_L1 44  // Left sensor 1 trigger pin
#define ECHO_PIN_L1 45     // Left sensor 1 echo pin

#define TRIGGER_PIN_L2 24  // Left sensor 2 trigger pin
#define ECHO_PIN_L2 25     // Left sensor 2 echo pin

#define TRIGGER_PIN_R1 32  // Right sensor 1 trigger pin
#define ECHO_PIN_R1 33     // Right sensor 1 echo pin

#define TRIGGER_PIN_R2 29  // Right sensor 2 trigger pin
#define ECHO_PIN_R2 28     // Right sensor 2 echo pin


#define MAX_DISTANCE 300  // Maximum distance for ultrasonic sensors


#define swi 30  // Switch pin

MPU6050 mpu(Wire);  // MPU6050 sensor for gyroscope data


// Sensor objects for ultrasonic measurements on all sides of the vehicle
NewPing sonar_L1(TRIGGER_PIN_L1, ECHO_PIN_L1, MAX_DISTANCE);
NewPing sonar_L2(TRIGGER_PIN_L2, ECHO_PIN_L2, MAX_DISTANCE);
NewPing sonar_R1(TRIGGER_PIN_R1, ECHO_PIN_R1, MAX_DISTANCE);
NewPing sonar_R2(TRIGGER_PIN_R2, ECHO_PIN_R2, MAX_DISTANCE);
NewPing sonar_F(TRIGGER_PIN_F, ECHO_PIN_F, MAX_DISTANCE);  // Front sensor
NewPing sonar_B(B_trigger, B_echo, MAX_DISTANCE);          // Back sensor
NewPing sonar_BL(BL_trigger, BL_echo, MAX_DISTANCE);       // Back-left sensor
NewPing sonar_BR(BR_trigger, BR_echo, MAX_DISTANCE);       // Back-right sensor
NewPing sonar_D2R(D2R_trigger, D2R_echo, MAX_DISTANCE);    // Diagonal-right sensor
NewPing sonar_D2L(D2L_trigger, D2L_echo, MAX_DISTANCE);    // Diagonal-left sensor

// Variables for holding the calculated distances
double f, fl, bl, fr, br, b;

// Function to get the distance from the front sensor
double get_distance_F() {
  duration_F = sonar_F.ping();    // Ping the sensor and get the echo response duration
  f = (duration_F / 2) * 0.0343;  // Calculate the distance based on sound speed
  return f;                       // Return the calculated distance
}

// Function to get the distance from the back sensor
double get_distance_B() {
  duration_B = sonar_B.ping();    // Ping the sensor and get the echo response duration
  b = (duration_B / 2) * 0.0343;  // Calculate the distance based on sound speed
  return b;                       // Return the calculated distance
}

// Function to get the distance from the back-left sensor
double get_distance_BL() {
  duration_BL = sonar_BL.ping();
  return (duration_BL / 2) * 0.0343;
}
  // Function to get the distance from the back-right sensor
  double get_distance_BR() {
    duration_BR = sonar_BR.ping();
    return (duration_BR / 2) * 0.0343;
  }

  // Function to get the distance from the left sensor 1
  double get_distance_L1() {
    duration_L1 = sonar_L1.ping();
    fl = (duration_L1 / 2) * 0.0343;
    return fl;
  }
    // Function to get the distance from the left sensor 2
    double get_distance_L2() {
      duration_L2 = sonar_L2.ping(101);
      bl = ((duration_L2 / 2) * 0.0343);
      return bl;
    }
    // Function to get the distance from the right sensor 1
    double get_distance_R1() {
      duration_R1 = sonar_R1.ping(101);
      fr = ((duration_R1 / 2) * 0.0343);

      return fr;
    }
    // Function to get the distance from the right sensor 2
    double get_distance_R2() {
      duration_R2 = sonar_R2.ping(101);
      br = ((duration_R2 / 2) * 0.0343);

      return br;
    }
    // Function to get the distance from the front right radical sensor

    double get_distance_D2R() {
      duration_D2R = sonar_D2R.ping();
      if (((duration_D2R / 2) * 0.0343) == 0)
        return 100;
      return ((duration_D2R / 2) * 0.0343);
    }
    // Function to get the distance from the front left radical sensor
    double get_distance_D2L() {
      duration_D2L = sonar_D2L.ping();
      if (((duration_D2L / 2) * 0.0343) == 0)
        return 100;
      return ((duration_D2L / 2) * 0.0343) - 3;
    }


    /*  NOTE: all functions mentioned in this code and start with "turn" are responsible for turning the robot in
            clockwise or counterclockwise dirction , the safety conditions are different in each one.(There are a few samples below:*/


    //a function to control servo motor movements with a specific constrains
    void myservo_write(double ang) {
      if (ang >= max_ang) ang = max_ang;
      if (ang <= min_ang) ang = min_ang;
      myservo.write(ang);
    }
    // a function to rotate robot in a specific angle in the clockwise dirction
    void turn_right_gyro(double v = 52.5, double ang = 75) {  // v is a setpoint for front sensor distance to make a turn in a safe place
      sensor();                                               // and ang is the angle of turn
      if (f < v) {
        Stop_Sumo(0);
        delay(50);
        while (f < v) {

          angle_Left_Back(bl, fl);  //@
          Back_Sumo(spd_before_turn_back);
          sensor();
        }
        Stop_Sumo(0);
        delay(200);


        Run_Sumo(spd_turn_now);
      }
      theta = ang - angle_cal(fl, bl) * 180 / acos(-1);
      mpu.update();


      theta_mpu = mpu.getAngleZ();
      ref_mpu = theta_mpu;
      theta = ref_mpu - theta;
      get_distance_F();
      delay(10);
      while (true)  //abs(theta_mpu - theta) >= 2 || f<150
      {
        if ((theta_mpu - theta) <= 0) break;  //&& f>150
        get_distance_F();
        mpu.update();
        theta_mpu = mpu.getAngleZ();
        delay(30);
        myservo_write(pos_R + 40);
      }
      myservo_write(pos_R);
      counter += 1;
      counter_angle_Left = 1;
    }
    // a function to rotate robot in a specific angle in the counterclockwise dirction

    void turn_left_GG_CCW_3(double ang = 75, double x = 60) {  // x is a setpoint for front sensor distance to make a turn in a safe place
      sensor();                                                //and ang is the angle of turn

      if ((f < x)) {
        Stop_Sumo(0);
        delay(50);
        while (f < x) {
          angle_Right(br, fr);  //@
          Back_Sumo(spd_before_turn_back);
          sensor();
        }
        Stop_Sumo(0);
        delay(50);

        Run_Sumo(spd_turn_now);
      }

      theta = ang;
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      ref_mpu = theta_mpu;
      theta = ref_mpu + theta;
      get_distance_F();
      delay(10);
      dis_D2R = get_distance_D2R();
      delay(10);

      while (true) {
        if (((theta_mpu - theta) >= 0) || ((dis_D2R < 3) && (dis_D2R > 0))) break;  // && f>150
        mpu.update();
        theta_mpu = mpu.getAngleZ();

        myservo_write(pos_L - 50);
        get_distance_F();
        delay(5);
        dis_D2R = get_distance_D2R();
        delay(10);
      }
      myservo_write(pos_L);
      counter_angle_Right = 1;

      myservo_write(pos_L);
    }

    // a function to rotate robot in a specific angle in the counterclockwise dirction
    void turn_left_gyro(double ang = 75, double x = 60) {  // x is a setpoint for front sensor distance to make a turn in a safe place
      sensor();                                            //and ang is the angle of turn

      if (f < x) {
        Stop_Sumo(0);
        delay(50);
        while (f < x) {
          angle_Right(br, fr);  //@
          Back_Sumo(spd_before_turn_back);
          sensor();
        }
        Stop_Sumo(0);
        delay(50);

        Run_Sumo(spd_turn_now);
      }

      theta = ang - angle_cal(fr, br) * 180 / acos(-1);
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      ref_mpu = theta_mpu;
      theta = ref_mpu + theta;
      get_distance_F();
      delay(10);
      while (true) {
        if ((theta_mpu - theta) >= 0) break;  // && f>150
        mpu.update();
        theta_mpu = mpu.getAngleZ();
        delay(30);
        myservo_write(pos_L - 50);
        get_distance_F();
      }
      myservo_write(pos_L);
      counter += 1;
      counter_angle_Right = 1;

      myservo_write(pos_L);
    }

    void turn_back_CW(double ang) {

      theta = ang;  //- angle_cal(fl, bl) * 180 / acos(-1);

      mpu.update();
      theta_mpu = mpu.getAngleZ();

      theta = mpu.getAngleZ() - theta;

      Back_Sumo(spd_turn_Back_ob);
      while (true) {
        if (((theta_mpu - theta) <= 0)) {
          break;
        }

        mpu.update();
        theta_mpu = mpu.getAngleZ();

        myservo_write(pos_R - 40);
        delay(30);

        Back_Sumo(spd_turn_Back_ob);
      }
      myservo_write(pos_R);
    }


    void turn_back_CCW(double ang) {

      theta = ang - angle_cal(fr, br) * 180 / acos(-1);

      mpu.update();
      theta_mpu = mpu.getAngleZ();

      theta = mpu.getAngleZ() + theta;

      Back_Sumo(spd_turn_Back_ob);
      while (true) {
        if (((theta_mpu - theta) >= 0)) {
          break;
        }

        mpu.update();
        theta_mpu = mpu.getAngleZ();

        myservo_write(pos_L + 50);
        delay(30);

        Back_Sumo(spd_turn_Back_ob);
      }
      myservo_write(pos_L);
    }


    void turn_back_parking(double ang) {
      sensor();
      theta = ang - angle_cal(fr, br) * 180 / acos(-1);

      mpu.update();
      theta_mpu = mpu.getAngleZ();

      theta = mpu.getAngleZ() + theta;
      dis_BL = get_distance_BL();

      Back_Sumo(spd_turn_Back_ob);
      while (true) {
        if (((theta_mpu - theta) >= 0) || (dis_BL < 3 && dis_BL > 0) || (b < 6.5 && b > 0)) {
          Stop_Sumo(0);

          break;
        }
        dis_BL = get_distance_BL();

        sensor();
        mpu.update();
        theta_mpu = mpu.getAngleZ();

        myservo_write(pos_L + 50);
        delay(30);

        Back_Sumo(spd_turn_Back_ob);
      }
      myservo_write(pos_L);
    }


    void turn_left_CW_parking(double ang) {
      sensor();
      dis_D2L = get_distance_D2L();

      theta = ang - angle_cal(fl, bl) * 180 / acos(-1);
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      ref_mpu = theta_mpu;
      theta = ref_mpu + theta;
      Run_Sumo(spd_Ob);
      while (true) {
        sensor();
        dis_D2L = get_distance_D2L();

        if (((theta_mpu - theta) >= 0) || (dis_D2L < 2 && dis_BL > 0) || (f < 5 && f > 0)) {
          Stop_Sumo(0);

          break;
        }  // && f>150
        mpu.update();
        theta_mpu = mpu.getAngleZ();
        Serial.println(mpu.getAngleZ());
        delay(30);
        myservo_write(pos_R - 40);
      }
      myservo_write(pos_R);
    }


    void turn_left_gyro_G_CW(double ang, int th = 1) {
      sensor();
      theta = ang - th * (angle_cal(fl, bl) * 180 / acos(-1));
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      ref_mpu = theta_mpu;
      theta = ref_mpu + theta;
      Run_Sumo(spd_Ob);
      while (true) {
        if ((theta_mpu - theta) >= 0) break;  // && f>150
        mpu.update();
        theta_mpu = mpu.getAngleZ();
        Serial.println(mpu.getAngleZ());
        delay(30);
        myservo_write(pos_R - 40);
      }
      myservo_write(pos_R);
    }


    void turn_right_gyro_G_CW(double ang) {
      sensor();

      mpu.update();
      theta_mpu = mpu.getAngleZ();
      theta = ang - theta_mpu;  //- angle_cal(fl, bl) * 180 / acos(-1);
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      ref_mpu = theta_mpu;
      theta = ref_mpu + theta;
      Run_Sumo(spd_Ob);
      while (true) {
        if ((theta_mpu - gyro_ref) <= 0) break;  // && f>150
        mpu.update();
        theta_mpu = mpu.getAngleZ();
        delay(30);
        myservo_write(pos_R + 40);
      }
      myservo_write(pos_R);
    }



    void turn_right_gyro_R_CW(double ang, int th = 1) {

      sensor();
      theta = ang - th * angle_cal(fl, bl) * 180 / acos(-1);
      mpu.update();


      theta_mpu = mpu.getAngleZ();
      ref_mpu = theta_mpu;
      theta = ref_mpu - theta;

      while (true)  //abs(theta_mpu - theta) >= 2 || f<150
      {
        if ((theta_mpu - theta) <= 0) break;  //&& f>150

        mpu.update();
        theta_mpu = mpu.getAngleZ();
        delay(30);
        myservo_write(pos_R + 40);
      }
      myservo_write(pos_R);
    }

    void turn_left_gyro_R_CW(double ang) {
      sensor();
      mpu.update();
      theta_mpu = mpu.getAngleZ();
      theta = ang + theta_mpu;  // - angle_cal(fr, br) * 180 / acos(-1);

      // ref_mpu = theta_mpu;
      // theta = ref_mpu + theta;

      while (true) {
        if ((theta_mpu - gyro_ref) >= 0) break;  // && f>150
        mpu.update();
        theta_mpu = mpu.getAngleZ();
        delay(30);
        myservo_write(pos_L - 50);
      }

      myservo_write(pos_L);
    }



    // Sensor() is a function for collect readings from ultrasonic sensors and to Real_Distance mentioned in the engineering journal
    void sensor() {

      dis_L1 = get_distance_L1();
      dis_R2 = get_distance_R2();
      delay(15);

      dis_L2 = get_distance_L2();
      dis_R1 = get_distance_R1();
      delay(15);


      dis_B = get_distance_B();
      delay(5);
      dis_F = get_distance_F();
      delay(5);

      Real_dis_L = Distance(fl, bl);
      Real_dis_R = Distance(fr, br);
    }

    //to calculate the angle of robot according to wall using ultrasonic sensors reading
    double angle_cal(double dis_1, double dis_2) {
      Angle = atan((dis_1 - dis_2) / 10.5);
      if (Angle > highest_angle) {
        Angle = highest_angle;
      }
      if (Angle < ((-1) * highest_angle)) {
        Angle = (-1) * highest_angle;
      }
      return Angle;
      //Radian
    }


    // P_control for the calculated angle to make the robot parallel to the left wall
    // while moving forward
    double angle_Left(double L1, double L2) {

      angle = angle_cal(L1, L2);

      Angle = (angle * 180) / 3.14;

      position_angle = (pos_L - Angle) * kpl;

      myservo_write(position_angle);
    }
    // P_control for the calculated angle to make the robot parallel to the left wall
    // while moving backward
    double angle_Left_Back(double L1, double L2) {

      angle = angle_cal(L1, L2);

      Angle = (angle * 180) / 3.14;

      position_angle = pos_L_Back - Angle;

      myservo.write(position_angle);
    }
    // P_control for the calculated angle to make the robot parallel to the right wall
    // while moving forward
    double angle_Right(double R1, double R2) {

      angle = angle_cal(R1, R2);

      Angle = (angle * 180) / 3.14;

      position_angle = (pos_R + Angle) * kpr;

      myservo_write(position_angle);
    }



    // P_control for the calculated angle to make the robot parallel to the right wall
    // while moving backward

    double angle_Right_Back(double R1, double R2) {

      angle = angle_cal(R1, R2);

      Angle = (angle * 180) / 3.14;

      position_angle = (pos_R_back + Angle) * kpr;

      myservo_write(position_angle);
    }


    //to calculate the Real_Distance (the perpendicular distnace from the wall which mention in the Engineering Journal)
    double Distance(double L1, double L2) {
      distance = (L1 + L2) / 2;
      angle = angle_cal(L1, L2);
      real_distance = distance * cos(angle);
      return real_distance;
    }

    // P_control for the distance form wall to make the robot with sepcific setpoint from wall

    double P_distance_R_ob(double real_distance, double setpoint) {
      pos_distance = conskp_NotstartR * (real_distance - setpoint);
      jbes = pos_R + pos_distance;
      myservo_write(jbes);
    }


    // P_control for the distance form wall to make the robot with sepcific setpoint from wall

    double P_distance_L_ob(double real_distance, double setpoint) {

      pos_distance = conskp_NotstartL * (real_distance - setpoint);
      jbes = pos_L - pos_distance;
      myservo_write(jbes);
    }


    // P_control for the distance form wall to make the robot with sepcific setpoint from wall

    double P_distanceL(double real_distance) {
      if (counter != 0) {
        conskpL = conskp_NotstartL;
      }

      pos_distance = conskpL * (real_distance - Setpoint);
      jbes = pos_L - pos_distance;
      myservo_write(jbes);
    }



    // P_control for the distance form wall to make the robot with sepcific setpoint from wall

    double P_distanceR(double real_distance) {
      if (counter != 0) {
        conskpR = conskp_NotstartR;
      }
      pos_distance = conskpR * (real_distance - Setpoint);
      jbes = pos_R + pos_distance;
      myservo_write(jbes);
    }


    // to move robot forward
    void Run_Sumo(int pwm) {
      digitalWrite(R_EN, HIGH);
      digitalWrite(L_EN, HIGH);
      analogWrite(R_PWM, pwm);
      analogWrite(L_PWM, 0);
    }


    // to move robot backward
    void Back_Sumo(int pwm) {
      digitalWrite(R_EN, HIGH);
      digitalWrite(L_EN, HIGH);
      analogWrite(R_PWM, 0);
      analogWrite(L_PWM, pwm);
    }

    // to stop robot
    void Stop_Sumo(int pwm) {
      digitalWrite(R_EN, HIGH);
      digitalWrite(L_EN, HIGH);
      analogWrite(R_PWM, 0);
      analogWrite(L_PWM, 0);
    }


    // PID_control for gyro angle to move on a reference angle forwad
    void PID_angle(double current_angle) {
      currentTime_angle = (micros()) * (1e-3);
      dtc = currentTime_angle - lastTime_angle;

      lastTime_angle = currentTime_angle;
      error_c = current_angle - reference;

      output_c = kpc * error_c + ((error_c - last_angle_error) / dtc) * kdc;
      myservo_write(output_c + pos_R);
      last_angle_error = error_c;
    }


    // PID_control for gyro angle to move on a reference angle backward
    void Back_PID_angle(double current_angle) {
      currentTime_angle = (micros()) * (1e-3);
      dtc = currentTime_angle - lastTime_angle;
      lastTime_angle = currentTime_angle;
      error_c = current_angle - reference;
      output_c = kpc * error_c + ((error_c - last_angle_error) / dtc) * kdc;
      myservo_write(pos_R - output_c);
      last_angle_error = error_c;
    }

    /* All functions starts with "PID" are functions for entering or exit the straight forward section and the navigation inside it
   the diffence in the cases which can be determined by the name of function*/

    void PID_G(double dis = 30) {
      reference = First_reference;
      while (true) {
        sensor();
        if ((fl > dis || fl == -4) || (bl > dis || bl == -4)) {  //(fr < 50) ||
          counter_angle_Right = 1;

          break;
        }
        if (f < 35) {  //(fr < 50) ||


          counter_angle_Left = 1;
          break;
        }
        if (Real_dis_L < Setpoint_min) {
          Run_Sumo(spd_Ob);
          P_distance_L_ob(Real_dis_L, Setpoint_min);
        }

        else if (Real_dis_L > Setpoint_max) {
          Run_Sumo(spd_Ob);
          P_distance_L_ob(Real_dis_L, Setpoint_max);
        }

        else {
          Run_Sumo(spd_Ob);
          angle_Left(dis_L1, dis_L2);
        }
        mpu.update();
        PID_angle(mpu.getAngleZ());
      }
    }


    void PID_R(double dis = 75) {
      Run_Sumo(spd_Ob);
      while (true) {  //|| ((millis() - Pre_Time) < 1000)) {
        sensor();
        if ((fr > 40) || (br > 40) || (fr == -4) || (br == -4)) {  //(fr < 50) ||
          counter_angle_Left = 1;
          // Stop_Sumo(0);
          // delay(2000);
          Run_Sumo(spd_Ob);
          break;
        }
        if (f < 40) {
          Stop_Sumo(0);

          if ((fl > dis) || (bl > dis) || (fl == -4) || (bl == -4)) {  //(fr < 50) ||
            counter_angle_Right = 1;

            Run_Sumo(spd_Ob);
            break;
          }
        }


        if (Real_dis_R < Setpoint_min) {

          P_distance_R_ob(Real_dis_R, Setpoint_min);
        }

        else if (Real_dis_R > Setpoint_max) {

          P_distance_R_ob(Real_dis_R, Setpoint_max);
        }

        else {

          mpu.update();
          PID_angle(mpu.getAngleZ());
        }
      }
      //Stop_Sumo(0);
      //delay(1000);
    }

    bool Break_While = false;
    void PID_parking() {

      while (true) {
        sensor();

        if (bl < 11) {
          Pre_Time = millis();
          while (true) {
            sensor();


            if ((bl < 11) && ((millis() - Pre_Time) > 400)) {  //(fr < 50) ||
              Break_While = true;
              break;
            }
            if (Real_dis_L < Setpoint_min) {
              Run_Sumo(spd_Ob);
              P_distance_L_ob(Real_dis_L, Setpoint_min);
            }

            else if (Real_dis_L > Setpoint_max) {
              Run_Sumo(spd_Ob);
              P_distance_L_ob(Real_dis_L, Setpoint_max);
            }

            else {
              Run_Sumo(spd_Ob);
              angle_Left(dis_L1, dis_L2);
            }
          }  //(fr < 50) ||
        }
        if (Break_While) break;
        if (Real_dis_L < Setpoint_min) {
          Run_Sumo(spd_Ob);
          P_distance_L_ob(Real_dis_L, Setpoint_min);
        }

        else if (Real_dis_L > Setpoint_max) {
          Run_Sumo(spd_Ob);
          P_distance_L_ob(Real_dis_L, Setpoint_max);
        }

        else {
          Run_Sumo(spd_Ob);
          angle_Left(dis_L1, dis_L2);
        }
      }
    }

    void PID_L_Range() {

      while ((fr > 100)) {
        sensor();
        if (Real_dis_L < Setpoint_min) {
          Run_Sumo(spd_Ob);
          P_distance_L_ob(Real_dis_L, Setpoint_min);
        }

        else if (Real_dis_L > Setpoint_max) {
          Run_Sumo(spd_Ob);
          P_distance_L_ob(Real_dis_L, Setpoint_max);
        }

        else {
          Run_Sumo(spd_Ob);

          angle_Left(dis_L1, dis_L2);
        }
      }
      while (true) {
        sensor();

        if (f < 40) {  //(fr < 50) ||
          break;
        }
        if (Real_dis_L < Setpoint_min) {
          Run_Sumo(spd_Ob);
          P_distance_L_ob(Real_dis_L, Setpoint_min);
        }

        else if (Real_dis_L > Setpoint_max) {
          Run_Sumo(spd_Ob);
          P_distance_L_ob(Real_dis_L, Setpoint_max);
        }

        else {
          Run_Sumo(spd_Ob);
          angle_Left(dis_L1, dis_L2);
        }
      }
    }



    void PID_R_Range() {

      while ((br > 100) || ((br > 100) && (fr > 100)) || (b < 90)) {
        sensor();
        Run_Sumo(spd_Ob);
        //  Stop_Sumo(0);

        // delay(100);
        // mpu.update();
        // PID_angle(mpu.getAngleZ());
        angle_Left(dis_L1, dis_L2);
        // }
      }

      // Stop_Sumo(0);

      // delay(1000);
      Pre_Time = millis();
      sensor();
      while ((fr < 30) && (fr > 0)) {  //|| ((millis() - Pre_Time) < 1000)) {
        sensor();

        if (Real_dis_R < Setpoint_min - 4) {
          Run_Sumo(spd_Ob);
          P_distance_R_ob(Real_dis_R, Setpoint_min - 4);
        }

        else if (Real_dis_R > Setpoint_max - 5) {
          Run_Sumo(spd_Ob);
          P_distance_R_ob(Real_dis_R, Setpoint_max - 5);
        }

        else {
          Run_Sumo(spd_Ob);
          angle_Right(dis_R1, dis_R2);
        }
      }
      //Stop_Sumo(0);
      //delay(1000);
    }


    void PID_L_GR() {
      while (true) {  //62
        sensor();
        if ((br < 87) && (b > 85)) {  //(br < 80) ||
          break;
        }
        if (Real_dis_L < Setpoint_min) {
          Run_Sumo(spd_Ob);
          P_distance_L_ob(Real_dis_L, Setpoint_min);
        }

        if (Real_dis_L > Setpoint_max) {
          Run_Sumo(spd_Ob);
          P_distance_L_ob(Real_dis_L, Setpoint_max);
        }

        else {
          Run_Sumo(spd_Ob);
          angle_Left(dis_L1, dis_L2);
        }
      }
    }


    void PID_L_RG() {

      while (true) {
        sensor();

        if (fr > 90 || f < 70) {  //(fr < 50) ||
          break;
        }
        if (Real_dis_L < Setpoint_min) {
          Run_Sumo(spd_Ob);
          P_distance_L_ob(Real_dis_L, Setpoint_min);
        }

        else if (Real_dis_L > Setpoint_max) {
          Run_Sumo(spd_Ob);
          P_distance_L_ob(Real_dis_L, Setpoint_max);
        }

        else {
          Run_Sumo(spd_Ob);
          angle_Left(dis_L1, dis_L2);
        }
      }
    }

    void PID_R_GR() {

      while (true) {
        sensor();
        if (fl > 30) {
          break;
        }
        if (Real_dis_R < Setpoint_min - 2) {
          Run_Sumo(spd_Ob);
          P_distance_R_ob(Real_dis_R, Setpoint_min - 2);
        }

        else if (Real_dis_R > Setpoint_max - 2) {
          Run_Sumo(spd_Ob);
          P_distance_R_ob(Real_dis_R, Setpoint_max - 2);
        }

        else {
          Run_Sumo(spd_Ob);
          angle_Right(dis_R1, dis_R2);
        }
      }
      //Stop_Sumo(0);
      //delay(1000);
    }

    void PID_R_RG() {
      while (true) {  //|| (b < 90)
        if (((br < 50)) && (b > 80)) {
          break;
        }

        sensor();
        Run_Sumo(spd_Ob);
        //  Stop_Sumo(0);

        delay(100);
        // mpu.update();
        // PID_angle(mpu.getAngleZ());
        angle_Left(dis_L1, dis_L2);
        // }
      }
    }

    void PID_RG_CCW_1() {
      while (true) {  //|| (b < 90)
        if (((bl < 80)) && (b > 80)) {
          break;
        }

        sensor();
        Run_Sumo(spd_Ob);
        //  Stop_Sumo(0);

        delay(100);
        // mpu.update();
        // PID_angle(mpu.getAngleZ());
        angle_Right(dis_R1, dis_R2);
        // }
      }
    }


    void PID_GG_CCW() {
      // double xr = 5.5;
      // pos_R += xr;

      Run_Sumo(spd_Ob);

      while (true) {  //|| ((fl > 100))
        sensor();
        if ((bl < 40) && (b > 80) && (bl > 0)) break;

        //  Stop_Sumo(0);

        //delay(10);
        //mpu.update();
        //  PID_angle(mpu.getAngleZ());
        angle_Right(dis_R1, dis_R2);
        // }
      }

      //Stop_Sumo(0);
      //delay(1000);
      // pos_R -= xr;

      Pre_Time = millis();


      // double xl = 4;
      // pos_L += xl;
      while (true) {
        sensor();

        if ((fl > 40) || (fl < 1) || (bl > 40) || (bl < 0)) {
          break;
        }


        if (Real_dis_L < Setpoint_min - 2) {
          Run_Sumo(spd_Ob);
          P_distance_L_ob(Real_dis_L, Setpoint_min);
        }

        else if (Real_dis_L > Setpoint_max - 6) {
          Run_Sumo(spd_Ob);
          P_distance_L_ob(Real_dis_L, Setpoint_max - 6);
        }

        else {
          Run_Sumo(spd_Ob);
          angle_Left(dis_L1, dis_L2);
        }
      }
      //Stop_Sumo(0);
      //delay(1000);
      // pos_L -= xl;
    }


    void PID_GR_CCW_1() {

      Run_Sumo(spd_Ob);

      while (true) {  //|| ((fl > 100))
        sensor();
        if ((bl < 40) && (b > 80) && (bl > 0)) break;

        //  Stop_Sumo(0);

        //delay(10);
        //mpu.update();
        //  PID_angle(mpu.getAngleZ());
        angle_Right(dis_R1, dis_R2);
        // }
      }

      //Stop_Sumo(0);
      //delay(1000);
      // pos_R -= xr;
    }




    void PID_RR_CCW() {
      Run_Sumo(spd_Ob);
      Setpoint_max -= 1;
      while (true) {  //|| ((millis() - Pre_Time) < 1000)) {
        sensor();

        if (((f < 40) && (fl < 0)) || ((f < 40) && (fl > 85))) {
          break;
        }



        if (Real_dis_R < Setpoint_min) {

          P_distance_R_ob(Real_dis_R, Setpoint_min);
        }

        else if (Real_dis_R > Setpoint_max) {

          P_distance_R_ob(Real_dis_R, Setpoint_max);
        }

        else {

          angle_Right(fr, br);
        }
      }
      Setpoint_max += 1;
      //Stop_Sumo(0);
      //delay(1000);
    }




    void PID_RG_CCW_2() {
      while (true) {
        sensor();

        if ((fl > 40) || (fl < 1) || (bl > 40) || (bl < 0)) {
          break;
        }


        if (Real_dis_L < Setpoint_min) {
          Run_Sumo(spd_Ob);
          P_distance_L_ob(Real_dis_L, Setpoint_min);
        }

        else if (Real_dis_L > Setpoint_max - 4) {
          Run_Sumo(spd_Ob);
          P_distance_L_ob(Real_dis_L, Setpoint_max - 4);
        }

        else {
          Run_Sumo(spd_Ob);
          angle_Left(dis_L1, dis_L2);
        }
      }
    }


    // Dealing with No-Pillars case (in the start section only)
    void Nan() {
      mpu.update();
      reference = mpu.getAngleZ();
      Run_Sumo(spd_Ob);

      while (true) {
        sensor();
        if (f < 50) {
          Stop_Sumo(0);
          //       Serial.print("dis_L1 ");
          //  Serial.println(dis_L1);
          //  Serial.print("dis_L2 ");
          //  Serial.println(dis_L2);

          if ((fl > 50) || (bl > 50) || (bl <= 0) || (fl <= 0)) {  //(fr < 50) ||
            counter_angle_Right = 1;

            Run_Sumo(spd_Ob);
            break;
          }
          if ((fr > 50) || (br > 50) || (br <= 0) || (fr <= 0)) {  //(fr < 50) ||
            counter_angle_Left = 1;
            // Stop_Sumo(0);
            // delay(2000);
            Run_Sumo(spd_Ob);
            break;
          }
        }
        mpu.update();
        PID_angle(mpu.getAngleZ());
      }


      if (counter_angle_Left == 1) {
        Run_Sumo(spd_Ob);
        turn_right_gyro_R_CW(40);
        // Stop_Sumo(0);
        // delay(2000);




        Back_Sumo(spd_Ob);
        turn_back_CW(50);
        //

        // Stop_Sumo(0);
        // delay(1000);



        while (b > 15) {
          angle_Left_Back(bl, fl);  //@
          Back_Sumo(spd_before_turn_back);
          sensor();
        }
      }

      if (counter_angle_Right == 1) {
        mpu.update();
        Run_Sumo(spd_Ob);
        turn_left_gyro_G_CW(30);
        Back_Sumo(spd_Ob);
        // turn_right_gyro_G_CW(40);
        // Back_Sumo(spd_Ob); //reference = reference_1;

        sensor();



        //Stop_Sumo(0);
        //delay(1000);
        turn_back_CCW(67);
        while (b > 7) {
          angle_Right_Back(br, fr);
          Back_Sumo(spd_before_turn_back);
          sensor();
        }
      }
    }




    // Dealing with Green Pillar case (in the start section only)
    void G() {
      Pre_Time = millis();
      mpu.update();
      gyro_ref = mpu.getAngleZ();
      Run_Sumo(spd_Ob);
      turn_left_gyro_G_CW(30);


      //Stop_Sumo(0);
      //delay(1000);
      mpu.update();
      reference = mpu.getAngleZ();
      dis_D2L = get_distance_D2L();

      while (dis_D2L < 13) {
        // angle_Left(dis_L1, dis_L2);
        dis_D2L = get_distance_D2L();
        Back_Sumo(spd_Ob);
        mpu.update();
        Back_PID_angle(mpu.getAngleZ());
      }
      Run_Sumo(spd_Ob);

      while (dis_D2L > 13) {
        // angle_Left(dis_L1, dis_L2);
        dis_D2L = get_distance_D2L();
        delay(5);

        mpu.update();
        PID_angle(mpu.getAngleZ());
      }
      //Stop_Sumo(0);
      //delay(1000);
      Run_Sumo(spd_Ob);

      turn_right_gyro_G_CW(30);

      // Stop_Sumo(0);
      // delay(1000);
      Stop_Sumo(0);
      delay(2000);
      Run_Sumo(spd_Ob);
      PID_G();
      // Stop_Sumo(0);
      // delay(1000);
      Run_Sumo(spd_Ob);

      if (counter_angle_Left == 1) {
        turn_right_gyro(70);

        // Stop_Sumo(0);
        // delay(1000);



        while (b > 15) {
          angle_Left_Back(bl, fl);  //@
          Back_Sumo(spd_before_turn_back);
          sensor();
        }
      }

      if (counter_angle_Right == 1) {

        while (f > 7) {
          // mpu.update();
          sensor();
          // PID_angle(mpu.getAngleZ());  //@
          angle_Right(fr, br);

          Run_Sumo(spd_Ob);
        }

        sensor();



        //Stop_Sumo(0);
        //delay(1000);
        turn_back_CCW(76);
        while (b > 7) {
          angle_Right_Back(br, fr);
          Back_Sumo(spd_before_turn_back);
          sensor();
        }
      }
    }


    // Dealing with Green-Green or Green Pillars case (all sections) with parking lots
    void GGP(double dis_d2l = 33) {
      Pre_Time = millis();
      mpu.update();
      gyro_ref = mpu.getAngleZ();
      Run_Sumo(spd_Ob);

      turn_left_gyro_G_CW(25);

      Stop_Sumo(0);
      delay(1000);
      mpu.update();
      reference = mpu.getAngleZ();
      dis_D2L = get_distance_D2L();

      while (dis_D2L < dis_d2l) {
         
        dis_D2L = get_distance_D2L();
        Back_Sumo(spd_Ob);
        mpu.update();
        Back_PID_angle(mpu.getAngleZ());
      }
      Run_Sumo(spd_Ob);

      while (dis_D2L > dis_d2l) {
 
        dis_D2L = get_distance_D2L();
        delay(5);

        mpu.update();
        PID_angle(mpu.getAngleZ());
      }
  
      Run_Sumo(spd_Ob);

      turn_right_gyro_G_CW(20);

 
      PID_L_Range();
     

      Run_Sumo(spd_Ob);
      turn_right_gyro_R_CW(40);




      Back_Sumo(spd_Ob);
      turn_back_CW(50);



      while (b > 15) {

        angle_Left_Back(bl, fl);  
        Back_Sumo(spd_before_turn_back);
        sensor();
      }
    }


    // Dealing with Green-Green or Green Pillars case (all sections)
    void GG(double dis_d2l = 13) {
      Pre_Time = millis();
      mpu.update();
      gyro_ref = mpu.getAngleZ();
      Run_Sumo(spd_Ob);

      turn_left_gyro_G_CW(30);

      mpu.update();
      reference = mpu.getAngleZ();
      dis_D2L = get_distance_D2L();

      while (dis_D2L < dis_d2l) {

        dis_D2L = get_distance_D2L();
        Back_Sumo(spd_Ob);
        mpu.update();
        Back_PID_angle(mpu.getAngleZ());
      }
      Run_Sumo(spd_Ob);

      while (dis_D2L > dis_d2l) {
  
        dis_D2L = get_distance_D2L();
        delay(5);

        mpu.update();
        PID_angle(mpu.getAngleZ());
      }

      Run_Sumo(spd_Ob);

      turn_right_gyro_G_CW(20);



      PID_L_Range();
    
      Run_Sumo(spd_Ob);

      turn_right_gyro(60);

    



      while (b > 15) {

        angle_Left_Back(bl, fl);  //@
        Back_Sumo(spd_before_turn_back);
        sensor();
      }
    }

    // Dealing with Red Pillar case (start section only)
    void R() {

      Pre_Time = millis();
      mpu.update();
      gyro_ref = mpu.getAngleZ();



      Run_Sumo(spd_Ob);

      turn_right_gyro_R_CW(45);


      mpu.update();
      reference = mpu.getAngleZ();

      dis_D2R = get_distance_D2R();
      delay(5);

      while (dis_D2R > 13) {

        dis_D2R = get_distance_D2R();
        delay(5);
        mpu.update();
        PID_angle(mpu.getAngleZ());
        Run_Sumo(spd_Ob);
      }


      while (dis_D2R < 13) {

        dis_D2R = get_distance_D2R();
        delay(5);
        mpu.update();
        Back_PID_angle(mpu.getAngleZ());
        Back_Sumo(spd_Ob);
      }


      Run_Sumo(spd_Ob);

      turn_left_gyro_R_CW(40);



      PID_R();

      Stop_Sumo(0);
      delay(100);
      Run_Sumo(spd_Ob);



      if (counter_angle_Left == 1) {


        sensor();
        reference = gyro_ref;
        while (f > 7) {

          mpu.update();
          PID_angle(mpu.getAngleZ());  
          Run_Sumo(spd_Ob);
          sensor();
        }
        sensor();


        turn_back_CW(80);

        while (b > 15) {
          angle_Left_Back(bl, fl);
          Back_Sumo(spd_before_turn_back);
          sensor();
        }
      }
      if (counter_angle_Right == 1) {

        turn_left_gyro(80);



        while (b > 15) {
          angle_Right_Back(bl, fl); 
          Back_Sumo(spd_before_turn_back);
          sensor();
        }
      }
    }


    // Dealing with Red-Red or Red Pillars case (all sections) with parking lots

    void RR() {

      Pre_Time = millis();
      mpu.update();
      gyro_ref = mpu.getAngleZ();


      Run_Sumo(spd_Ob);

      turn_right_gyro_R_CW(35);

      mpu.update();
      reference = mpu.getAngleZ();

      dis_BL = get_distance_BL();
      delay(5);

      while (dis_BL > 60) {

        dis_BL = get_distance_BL();
        Back_Sumo(spd_Ob);
        mpu.update();
        Back_PID_angle(mpu.getAngleZ());
      }
      Run_Sumo(spd_Ob);

      while (dis_BL < 60) {

        dis_BL = get_distance_BL();
        mpu.update();
        PID_angle(mpu.getAngleZ());
      }



      Run_Sumo(spd_Ob);

      turn_left_gyro_R_CW(40);




      PID_R_Range();
      myservo_write(pos_L);
      mpu.update();
      reference = mpu.getAngleZ();
      Pre_Time = millis();
      while ((millis() - Pre_Time) < 100) {
        mpu.update();
        PID_angle(mpu.getAngleZ());
        Run_Sumo(spd_Ob);
      }
      sensor();
      spd_Ob = 45;
      Run_Sumo(spd_Ob);
      turn_left_gyro_G_CW(20, 0);
      Run_Sumo(spd_Ob);

      turn_right_gyro_R_CW(10);


      while (f > 12) {
        sensor();
        angle_Left(fl, bl);  //@
        Run_Sumo(spd_Ob);
      }
      turn_right_gyro(12, 20);



      turn_back_CW(60);

      sensor();
      spd_Ob = 50;

      while (b > 10) {
        sensor();
        angle_Left_Back(bl, fl);
        Back_Sumo(spd_before_turn_back);
      }
    }


    // Dealing with Green-Red Pillars case (all sections)
    void GR(double dis = 16) {
      Pre_Time = millis();
      mpu.update();
      gyro_ref = mpu.getAngleZ();
      Run_Sumo(spd_Ob);

      turn_left_gyro_G_CW(35);  

      mpu.update();
      reference = mpu.getAngleZ();
      dis_D2L = get_distance_D2L();

      while (dis_D2L < dis) {

        dis_D2L = get_distance_D2L();
        Back_Sumo(spd_Ob);
        mpu.update();
        Back_PID_angle(mpu.getAngleZ());
      }
      Run_Sumo(spd_Ob);

      while (dis_D2L > dis) {

        dis_D2L = get_distance_D2L();
        delay(5);

        mpu.update();
        PID_angle(mpu.getAngleZ());
      }

      Run_Sumo(spd_Ob);

      turn_right_gyro_G_CW(25);



      PID_L_GR();

      sensor();
      Pre_Time = millis();
      mpu.update();



      Run_Sumo(spd_Ob);

      turn_right_gyro_R_CW(50);


      mpu.update();
      reference = mpu.getAngleZ();

      dis_BL = get_distance_BL();
      delay(5);

      while (dis_D2R < 19) {
      
        dis_D2R = get_distance_D2R();
        Back_Sumo(spd_Ob);
        mpu.update();
        Back_PID_angle(mpu.getAngleZ());
      }
      Run_Sumo(spd_Ob);

      while (dis_D2R > 19) {
      
        dis_D2R = get_distance_D2R();
        mpu.update();
        PID_angle(mpu.getAngleZ());
      }
 

      Run_Sumo(spd_Ob);

      turn_left_gyro_R_CW(50);




      PID_R_GR();

   
      Run_Sumo(spd_Ob);

   

      mpu.update();
      reference = mpu.getAngleZ();
      Pre_Time = millis();
      while ((millis() - Pre_Time) < 100) {
        mpu.update();
        PID_angle(mpu.getAngleZ());
        Run_Sumo(spd_Ob);
      }
      sensor();
      spd_Ob = 45;
      Run_Sumo(spd_Ob);
      turn_left_gyro_G_CW(30, 0);
      Run_Sumo(spd_Ob);

      turn_right_gyro_R_CW(15);

      // Stop_Sumo(0);
      // delay(1000);
      while (f > 7) {
        sensor();
        angle_Left(fl, bl);  //@
        Run_Sumo(spd_Ob);
      }
      turn_back_CW(45);  //55
      Run_Sumo(spd_Ob);

      turn_right_gyro_R_CW(25, 0);  //15
      // Stop_Sumo(0);
      // delay(1000);

      Back_Sumo(spd_Ob);

      turn_back_CW(0);
      // Stop_Sumo(0);
      // delay(1000);
      sensor();
      spd_Ob = 50;

      while (b > 10) {
        angle_Left_Back(bl, fl);
        Back_Sumo(spd_before_turn_back);
        sensor();
      }
    }


    // Dealing with Red-Green Pillars case (all sections)
    void RG(double dis = 18) {
      Pre_Time = millis();
      mpu.update();
      gyro_ref = mpu.getAngleZ();


      Run_Sumo(spd_Ob);

      turn_right_gyro_R_CW(40);

      // Stop_Sumo(0);
      // delay(1000);
      mpu.update();
      reference = mpu.getAngleZ();

      dis_BL = get_distance_BL();
      delay(5);

      while (dis_BL > 60) {
        // angle_Left(dis_L1, dis_L2);
        dis_BL = get_distance_BL();
        Back_Sumo(spd_Ob);
        mpu.update();
        Back_PID_angle(mpu.getAngleZ());
      }
      Run_Sumo(spd_Ob);

      while (dis_BL < 60) {
        // angle_Left(dis_L1, dis_L2);
        dis_BL = get_distance_BL();
        mpu.update();
        PID_angle(mpu.getAngleZ());
      }


      //Stop_Sumo(0);
      //delay(1000);
      Run_Sumo(spd_Ob);

      turn_left_gyro_R_CW(40);


      //Stop_Sumo(0);
      //delay(1000);


      PID_R_RG();

      //Stop_Sumo(0);
      //delay(1000);

      turn_left_gyro_G_CW(40, 0);  //pre 45


      // Stop_Sumo(0);
      // delay(1000);
      mpu.update();
      reference = mpu.getAngleZ();
      dis_D2L = get_distance_D2L();

      while (dis_D2L < dis) {
        // angle_Left(dis_L1, dis_L2);
        dis_D2L = get_distance_D2L();
        Back_Sumo(spd_Ob);
        mpu.update();
        Back_PID_angle(mpu.getAngleZ());
      }
      Run_Sumo(spd_Ob);
      // Stop_Sumo(0);
      // delay(1000);
      while (dis_D2L > dis) {
        // angle_Left(dis_L1, dis_L2);
        dis_D2L = get_distance_D2L();
        delay(5);

        mpu.update();
        PID_angle(mpu.getAngleZ());
      }
      //Stop_Sumo(0);
      //delay(1000);
      Run_Sumo(spd_Ob);

      turn_right_gyro_G_CW(10);  //pre 15

      //Stop_Sumo(0);
      //delay(1000);

      PID_L_RG();
      //Stop_Sumo(0);
      //delay(1000);
      Run_Sumo(spd_Ob);

      turn_right_gyro();

      //Stop_Sumo(0);
      //delay(1000);



      while (b > 7) {
        angle_Left_Back(bl, fl);  //@
        Back_Sumo(spd_before_turn_back);
        sensor();
      }
    }

    // Dealing with Red-Green Pillars case (all sections) with pink lots

    void RGP(double dis = 33) {
      Pre_Time = millis();
      mpu.update();
      gyro_ref = mpu.getAngleZ();


      Run_Sumo(spd_Ob);

      turn_right_gyro_R_CW(40);

      // Stop_Sumo(0);
      // delay(1000);
      mpu.update();
      reference = mpu.getAngleZ();

      dis_BL = get_distance_BL();
      delay(5);

      while (dis_BL > 60) {
        // angle_Left(dis_L1, dis_L2);
        dis_BL = get_distance_BL();
        Back_Sumo(spd_Ob);
        mpu.update();
        Back_PID_angle(mpu.getAngleZ());
      }
      Run_Sumo(spd_Ob);

      while (dis_BL < 60) {
        // angle_Left(dis_L1, dis_L2);
        dis_BL = get_distance_BL();
        mpu.update();
        PID_angle(mpu.getAngleZ());
      }


      //Stop_Sumo(0);
      //delay(1000);
      Run_Sumo(spd_Ob);

      turn_left_gyro_R_CW(40);


      //Stop_Sumo(0);
      //delay(1000);


      PID_R_RG();

      //Stop_Sumo(0);
      //delay(1000);

      turn_left_gyro_G_CW(47);  //pre 45


      // Stop_Sumo(0);
      // delay(1000);
      mpu.update();
      reference = mpu.getAngleZ();
      dis_BR = get_distance_BR();

      while (dis_BR > dis) {
        // angle_Left(dis_L1, dis_L2);
        dis_BR = get_distance_BR();
        Back_Sumo(spd_Ob);
        mpu.update();
        Back_PID_angle(mpu.getAngleZ());
      }
      Run_Sumo(spd_Ob);
      // Stop_Sumo(0);
      // delay(1000);
      while (dis_BR < dis) {
        // angle_Left(dis_L1, dis_L2);
        dis_BR = get_distance_BR();
        delay(5);

        mpu.update();
        PID_angle(mpu.getAngleZ());
      }
      //Stop_Sumo(0);
      //delay(1000);
      Run_Sumo(spd_Ob);

      turn_right_gyro_G_CW(10);  //pre 15

      //Stop_Sumo(0);
      //delay(1000);

      PID_L_RG();
      //Stop_Sumo(0);
      //delay(1000);
      while (true) {
        sensor();
        if (f < 50) {
          break;
        }
        angle_Left(fl, bl);
      }
      Run_Sumo(spd_Ob);
      turn_right_gyro_R_CW(40);
      // Stop_Sumo(0);
      // delay(2000);




      Back_Sumo(spd_Ob);
      turn_back_CW(50);

      //Stop_Sumo(0);
      //delay(1000);



      while (b > 15) {
        angle_Left_Back(bl, fl);  //@
        Back_Sumo(spd_before_turn_back);
        sensor();
      }
    }

    // Dealing with Green-Green Pillars case (all sections in the counterclockwise direction )
    void GG_CCW() {
      Pre_Time = millis();
      sensor();
      mpu.update();
      gyro_ref = mpu.getAngleZ() - angle_cal(fr, br) * 180 / acos(-1);
      Run_Sumo(spd_Ob);

      turn_left_gyro_G_CW(40);

      //Stop_Sumo(0);
      //delay(1000);
      mpu.update();
      reference = mpu.getAngleZ();
      dis_BR = get_distance_BR();

      while (dis_BR > 58) {
        // angle_Left(dis_L1, dis_L2);
        dis_BR = get_distance_BR();
        Back_Sumo(spd_Ob);
        mpu.update();
        Back_PID_angle(mpu.getAngleZ());
      }
      Run_Sumo(spd_Ob);

      while (dis_BR < 58) {
        // angle_Left(dis_L1, dis_L2);
        dis_BR = get_distance_BR();
        delay(5);

        mpu.update();
        PID_angle(mpu.getAngleZ());
      }
      //Stop_Sumo(0);
      //delay(1000);
      Run_Sumo(spd_Ob);

      turn_right_gyro_G_CW(20);


      PID_GG_CCW();

      // while (f > 7) {
      //   // mpu.update();
      //   // PID_angle(mpu.getAngleZ());  //@
      //   angle_Right(fr, br);

      //   Run_Sumo(spd_Ob);

      //   sensor();
      // }

      // turn_back_CCW(76);


      sensor();
      spd_Ob = 45;
      turn_right_gyro_R_CW(20, 0);

      Run_Sumo(spd_Ob);
      Run_Sumo(spd_Ob);

      turn_left_gyro_G_CW(10, 0);

      // Stop_Sumo(0);
      // delay(1000);
      while (f > 17) {
        sensor();
        angle_Right(fr, br);  //@
        Run_Sumo(spd_Ob);
      }
      turn_left_gyro(20, 17);



      turn_back_CCW(60);


      sensor();
      spd_Ob = 50;

      while (b > 7) {
        angle_Right_Back(br, fr);
        Back_Sumo(spd_before_turn_back);
        sensor();
      }
    }


    // Dealing with Red-Red and Red Pillars case (all sections in the counterclockwise direction )
    void RR_CCW(double dis = 17) {

      Pre_Time = millis();
      mpu.update();
      gyro_ref = mpu.getAngleZ();


      Run_Sumo(spd_Ob);

      turn_right_gyro_R_CW(35);


      mpu.update();
      reference = mpu.getAngleZ();

      dis_D2R = get_distance_D2R();
      delay(5);

      while (dis_D2R < dis) {

        dis_D2R = get_distance_D2R();
        Back_Sumo(spd_Ob);
        mpu.update();
        Back_PID_angle(mpu.getAngleZ());
      }
      Run_Sumo(spd_Ob);


      dis_D2R = get_distance_D2R();
      mpu.update();
      PID_angle(mpu.getAngleZ());
    Run_Sumo(spd_Ob);
    turn_left_gyro_R_CW(40);

    PID_RR_CCW();

    mpu.update();

    turn_left_gyro(80, 63);

    // Stop_Sumo(0);
    // delay(1000);

    while (b > 7) {
      angle_Right_Back(br, fr);
      Back_Sumo(spd_before_turn_back);
      sensor();
    }
    }
  



  // Dealing with Red-Red Pillars case (all sections in the counterclockwise direction ) with ink lots

  void RRP_CCW(double dis = 33) {

    Pre_Time = millis();
    mpu.update();
    gyro_ref = mpu.getAngleZ();


    Run_Sumo(spd_Ob);

    turn_right_gyro_R_CW(35);

    mpu.update();
    reference = mpu.getAngleZ();

    dis_D2R = get_distance_D2R();
    delay(5);

    while (dis_D2R < dis) {

      dis_D2R = get_distance_D2R();
      Back_Sumo(spd_Ob);
      mpu.update();
      Back_PID_angle(mpu.getAngleZ());
    }
    Run_Sumo(spd_Ob);

    while (dis_D2R > dis) {

      dis_D2R = get_distance_D2R();
      mpu.update();
      PID_angle(mpu.getAngleZ());
    }
    Run_Sumo(spd_Ob);
    turn_left_gyro_R_CW(40);

    PID_RR_CCW();


    mpu.update();
    Run_Sumo(spd_Ob);


    if (f < 39) {
      Stop_Sumo(0);
      delay(50);
      while (f < 39) {
        angle_Right(br, fr);  //@
        Back_Sumo(spd_before_turn_back);
        sensor();
      }
      Stop_Sumo(0);
      delay(50);

      Run_Sumo(spd_turn_now);
    }
    turn_left_gyro_G_CW(30);
    Back_Sumo(spd_Ob);


    sensor();



    turn_back_CCW(67);


    while (b > 15) {
      angle_Right_Back(bl, fl);  //@
      Back_Sumo(spd_before_turn_back);
      sensor();
    }
  }

  void RG_CCW(double dis = 15) {
    Pre_Time = millis();
    mpu.update();
    gyro_ref = mpu.getAngleZ();


    Run_Sumo(spd_Ob);

    turn_right_gyro_R_CW(35);


    mpu.update();
    reference = mpu.getAngleZ();

    dis_D2R = get_distance_D2R();
    delay(5);

    while (dis_D2R < dis) {

      dis_D2R = get_distance_D2R();
      Back_Sumo(spd_Ob);
      mpu.update();
      Back_PID_angle(mpu.getAngleZ());
    }
    Run_Sumo(spd_Ob);

    while (dis_D2R > dis) {

      dis_D2R = get_distance_D2R();
      mpu.update();
      PID_angle(mpu.getAngleZ());
    }
    Run_Sumo(spd_Ob);
    turn_left_gyro_R_CW(40);

    PID_RG_CCW_1();





    turn_left_gyro_G_CW(40, 0);



    mpu.update();
    reference = mpu.getAngleZ();
    dis_D2L = get_distance_D2L();

    while (dis_D2L < 18) {

      dis_D2L = get_distance_D2L();
      Back_Sumo(spd_Ob);
      mpu.update();
      Back_PID_angle(mpu.getAngleZ());
    }
    Run_Sumo(spd_Ob);

    while (dis_D2L > 18) {

      dis_D2L = get_distance_D2L();
      delay(5);

      mpu.update();
      PID_angle(mpu.getAngleZ());
    }

    Run_Sumo(spd_Ob);

    turn_right_gyro_G_CW(10);


    PID_RG_CCW_2();




    sensor();
    spd_Ob = 45;
    turn_right_gyro_R_CW(20, 0);

    Run_Sumo(spd_Ob);

    turn_left_gyro_G_CW(10, 0);


    while (f > 17) {
      sensor();
      angle_Right(fr, br);  //@
      Run_Sumo(spd_Ob);
    }
    turn_left_GG_CCW_3(20, 17);



    turn_back_CCW(60);

    sensor();
    spd_Ob = 50;


    while (b > 7) {
      angle_Right_Back(br, fr);
      Back_Sumo(spd_before_turn_back);
      sensor();
    }
  }



  // Dealing with Green-Red Pillars case (all sections in the counterclockwise direction ) with parking lots
  void GRP_CCW(double dis = 42) {
    Pre_Time = millis();
    mpu.update();
    gyro_ref = mpu.getAngleZ();
    Run_Sumo(spd_Ob);

    turn_left_gyro_G_CW(40);

    mpu.update();
    reference = mpu.getAngleZ();
    dis_BR = get_distance_BR();

    while (dis_BR > 56) {
      // angle_Left(dis_L1, dis_L2);
      dis_BR = get_distance_BR();
      Back_Sumo(spd_Ob);
      mpu.update();
      Back_PID_angle(mpu.getAngleZ());
    }
    Run_Sumo(spd_Ob);

    while (dis_BR < 56) {
      // angle_Left(dis_L1, dis_L2);
      dis_BR = get_distance_BR();
      delay(5);

      mpu.update();
      PID_angle(mpu.getAngleZ());
    }

    Run_Sumo(spd_Ob);

    turn_right_gyro_G_CW(20);


    PID_GR_CCW_1();



    Pre_Time = millis();
    mpu.update();
    gyro_ref = mpu.getAngleZ();


    Run_Sumo(spd_Ob);

    turn_right_gyro_R_CW(39);

    mpu.update();
    reference = mpu.getAngleZ();

    dis_BL = get_distance_BL();
    delay(5);

    while (dis_BL > dis) {
      // angle_Left(dis_L1, dis_L2);
      dis_BL = get_distance_BL();
      Back_Sumo(spd_Ob);
      mpu.update();
      Back_PID_angle(mpu.getAngleZ());
    }
    Run_Sumo(spd_Ob);

    while (dis_BL < dis) {
      // angle_Left(dis_L1, dis_L2);
      dis_BL = get_distance_BL();
      mpu.update();
      PID_angle(mpu.getAngleZ());
    }
    Run_Sumo(spd_Ob);
    turn_left_gyro_R_CW(41);

    PID_RR_CCW();

    if (f < 39) {
      Stop_Sumo(0);
      delay(50);
      while (f < 39) {
        angle_Right(br, fr);  //@
        Back_Sumo(spd_before_turn_back);
        sensor();
      }
      Stop_Sumo(0);
      delay(50);

      Run_Sumo(spd_turn_now);
    }
    turn_left_gyro_G_CW(30);
    Back_Sumo(spd_Ob);



    sensor();




    turn_back_CCW(67);


    while (b > 7) {
      angle_Right_Back(bl, fl);  //@
      Back_Sumo(spd_before_turn_back);
      sensor();
    }
  }



  // Dealing with Green-Red Pillars case (all sections in the counterclockwise direction )
  void GR_CCW(double dis = 17) {
    Pre_Time = millis();
    mpu.update();
    gyro_ref = mpu.getAngleZ();
    Run_Sumo(spd_Ob);

    turn_left_gyro_G_CW(40);


    mpu.update();
    reference = mpu.getAngleZ();
    dis_BR = get_distance_BR();

    while (dis_BR > 56) {
      // angle_Left(dis_L1, dis_L2);
      dis_BR = get_distance_BR();
      Back_Sumo(spd_Ob);
      mpu.update();
      Back_PID_angle(mpu.getAngleZ());
    }
    Run_Sumo(spd_Ob);

    while (dis_BR < 56) {
      // angle_Left(dis_L1, dis_L2);
      dis_BR = get_distance_BR();
      delay(5);

      mpu.update();
      PID_angle(mpu.getAngleZ());
    }

    Run_Sumo(spd_Ob);

    turn_right_gyro_G_CW(20);


    PID_GR_CCW_1();



    Pre_Time = millis();
    mpu.update();
    gyro_ref = mpu.getAngleZ();


    Run_Sumo(spd_Ob);

    turn_right_gyro_R_CW(39, 0);


    mpu.update();
    reference = mpu.getAngleZ();

    dis_D2R = get_distance_D2R();
    delay(5);

    while (dis_D2R < dis) {
      // angle_Left(dis_L1, dis_L2);
      dis_D2R = get_distance_D2R();
      Back_Sumo(spd_Ob);
      mpu.update();
      Back_PID_angle(mpu.getAngleZ());
    }
    Run_Sumo(spd_Ob);

    while (dis_D2R > dis) {
      // angle_Left(dis_L1, dis_L2);
      dis_D2R = get_distance_D2R();
      mpu.update();
      PID_angle(mpu.getAngleZ());
    }
    Run_Sumo(spd_Ob);
    turn_left_gyro_R_CW(41);

    PID_RR_CCW();

    mpu.update();

    turn_left_gyro(80, 63);



    while (b > 7) {
      angle_Right_Back(bl, fl);  //@
      Back_Sumo(spd_before_turn_back);
      sensor();
    }
  }


  // Dealing with Green Pillar case (first section only) with parking lots
  void GP() {


    Pre_Time = millis();
    mpu.update();
    gyro_ref = mpu.getAngleZ();
    Run_Sumo(spd_Ob);

    turn_left_gyro_G_CW(30);


    mpu.update();
    reference = mpu.getAngleZ();
    dis_BR = get_distance_BR();

    while (dis_BR > 45) {
      // angle_Left(dis_L1, dis_L2);
      dis_BR = get_distance_BR();
      Back_Sumo(spd_Ob);
      mpu.update();
      Back_PID_angle(mpu.getAngleZ());
    }
    Run_Sumo(spd_Ob);

    while (dis_BR < 45) {
      // angle_Left(dis_L1, dis_L2);
      dis_BR = get_distance_BR();
      delay(5);

      mpu.update();
      PID_angle(mpu.getAngleZ());
    }

    Run_Sumo(spd_Ob);

    turn_right_gyro_G_CW(30);

    Run_Sumo(spd_Ob);
    PID_G(50);

    Run_Sumo(spd_Ob);

    if (counter_angle_Left == 1) {


      while (f < 39) {
        angle_Right(br, fr);  //@
        Back_Sumo(spd_before_turn_back);
        sensor();
      }
      Run_Sumo(spd_Ob);
      turn_right_gyro_R_CW(40);



      Back_Sumo(spd_Ob);
      turn_back_CW(50);


      while (b > 7) {
        angle_Left_Back(bl, fl);  //@
        Back_Sumo(spd_before_turn_back);
        sensor();
      }
    }


    if (counter_angle_Right == 1) {

      while (f > 7) {
        // mpu.update();
        sensor();
        // PID_angle(mpu.getAngleZ());  //@
        angle_Right(fr, br);

        Run_Sumo(spd_Ob);
      }

      sensor();



      turn_back_CCW(76);
      while (b > 7) {
        angle_Right_Back(br, fr);
        Back_Sumo(spd_before_turn_back);
        sensor();
      }
    }
  }


  // Dealing with Red Pillar case (first section only) with parking lots
  void RP() {

    Pre_Time = millis();
    mpu.update();
    gyro_ref = mpu.getAngleZ();


    Run_Sumo(spd_Ob);

    turn_right_gyro_R_CW(45);


    mpu.update();
    reference = mpu.getAngleZ();

    dis_BL = get_distance_BL();
    delay(5);

    while (dis_BL < 45) {

      dis_BL = get_distance_BL();
      delay(5);
      mpu.update();
      PID_angle(mpu.getAngleZ());
      Run_Sumo(spd_Ob);
    }


    while (dis_BL > 45) {

      dis_BL = get_distance_BL();
      delay(5);
      mpu.update();
      Back_PID_angle(mpu.getAngleZ());
      Back_Sumo(spd_Ob);
    }


    Run_Sumo(spd_Ob);

    turn_left_gyro_R_CW(40);





    PID_R(55);


    Run_Sumo(spd_Ob);


    mpu.update();

    if (counter_angle_Left == 1) {


      while (f > 7) {
        angle_Left(fl, bl);
        Run_Sumo(spd_Ob);
        sensor();
      }
      sensor();


      turn_back_CW(80);

      while (b > 15) {
        angle_Left_Back(bl, fl);
        Back_Sumo(spd_before_turn_back);
        sensor();
      }
    }
    if (counter_angle_Right == 1) {

      mpu.update();
      Run_Sumo(spd_Ob);


      if (f < 39) {
        Stop_Sumo(0);
        delay(50);
        while (f < 39) {
          angle_Right(br, fr);
          Back_Sumo(spd_before_turn_back);
          sensor();
        }
        Stop_Sumo(0);
        delay(200);

        Run_Sumo(spd_turn_now);
      }
      turn_left_gyro_G_CW(30);
      Back_Sumo(spd_Ob);


      sensor();




      turn_back_CCW(67);




      while (b > 7) {
        angle_Right_Back(bl, fl);  //@
        Back_Sumo(spd_before_turn_back);
        sensor();
      }
    }
  }


  // dealing with parking lots to confirm a parallel parking in both directions CCW and CW
  void parking(double dis = 30) {
    if (counter_angle_Left == 1) {

      Pre_Time = millis();
      mpu.update();
      gyro_ref = mpu.getAngleZ();
      Run_Sumo(spd_Ob);

      turn_left_gyro_G_CW(25);

      mpu.update();
      reference = mpu.getAngleZ();
      dis_D2L = get_distance_D2L();

      while (dis_D2L < dis) {

        dis_D2L = get_distance_D2L();
        Back_Sumo(spd_Ob);
        mpu.update();
        Back_PID_angle(mpu.getAngleZ());
      }
      Run_Sumo(spd_Ob);

      while (dis_D2L > dis) {

        dis_D2L = get_distance_D2L();
        delay(5);

        mpu.update();
        PID_angle(mpu.getAngleZ());
      }

      Run_Sumo(spd_Ob);

      turn_right_gyro_G_CW(20);


      PID_parking();

      Stop_Sumo(0);
      Pre_Time = millis();
      while (true) {
        Run_Sumo(spd_Ob);
        if ((millis() - Pre_Time) > 900) break;
        sensor();
        angle_Left(fl, bl);
      }
      Stop_Sumo(0);
      if (Real_dis_L < 22.5 && Real_dis_L > 19) {
        Pre_Time = millis();
        while (true) {
          Back_Sumo(spd_Ob);
          if ((millis() - Pre_Time) > 130) break;
          sensor();
          angle_Left_Back(bl, fl);
        }
      }
      sensor();


      Stop_Sumo(0);
      Back_Sumo(spd_Ob);
      turn_back_CW(45);





      mpu.update();
      reference = mpu.getAngleZ();
      dis_BL = get_distance_BL();

      while (dis_BL < 20) {

        dis_BL = get_distance_BL();
        delay(5);
        mpu.update();
        PID_angle(mpu.getAngleZ());
        Run_Sumo(spd_Ob);
      }


      while (dis_BL > 20) {
        dis_BL = get_distance_BL();
        delay(5);
        mpu.update();
        Back_PID_angle(mpu.getAngleZ());
        Back_Sumo(spd_Ob);
      }

      Back_Sumo(spd_turn_Back_ob);

      mpu.update();
      gyro_ref = mpu.getAngleZ();

      turn_back_parking(30);

      Run_Sumo(spd_Ob);
      turn_left_CW_parking(30);
      sensor();

      while (b > 7) {
        sensor();

        myservo_write(pos_R);
        Back_Sumo(spd_turn_Back_ob - 5);
      }
      Stop_Sumo(0);
      sensor();
      while (f > 5) {


        myservo_write(pos_R - 15);
        Run_Sumo(spd_turn_Back_ob - 5);
        sensor();
      }


      Stop_Sumo(0);
    }
  }


  // initials all pins , sensors and Serial
  void setup() {
    Serial.begin(115200);
    pinMode(Switch, CHANGE);
    pinMode(TRIGGER_PIN_L1, OUTPUT);
    pinMode(ECHO_PIN_L1, INPUT);
    pinMode(TRIGGER_PIN_L2, OUTPUT);
    pinMode(ECHO_PIN_L2, INPUT);
    pinMode(TRIGGER_PIN_R1, OUTPUT);
    pinMode(ECHO_PIN_R1, INPUT);
    pinMode(TRIGGER_PIN_R2, OUTPUT);
    pinMode(ECHO_PIN_R2, INPUT);
    pinMode(TRIGGER_PIN_F, OUTPUT);
    pinMode(ECHO_PIN_F, INPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);
    pinMode(R_PWM, OUTPUT);
    pinMode(L_PWM, OUTPUT);
    pinMode(swi, INPUT_PULLUP);



    myservo.attach(8);

    Wire.begin();
    byte status = mpu.begin();
    while (status != 0) {
      Serial.println("Satus!=0");
      status = mpu.begin();
    }

    mpu.calcOffsets();
    mpu.update();
    delay(10);
    //  while(!digitalRead(swi))
    //  {

    //    }
  }



  //Our main algorithm
  void loop() {

    if (counter == 0) {  //in the start section the counter = 0, we have 3 cases R (Red pillar), G (Green pillar) and N (no pillars) and if GG, RR, GR and RG send that mean the second pillar is in another section
      spd_Ob = 40;
      mpu.update();
      First_reference = mpu.getAngleZ();
      if (Serial.available()) {
        data = Serial.readStringUntil('#');
      }
      if (data == "R" || data == "RR" || data == "RG") {

        R();
        Stop_Sumo(0);
        delay(200);
        if (counter_angle_Left == 1) {

          Serial.println("R");
        } else if (counter_angle_Right == 1) {

          Serial.println("L");
        }
      }

      else if (data == "G" || data == "GG" || data == "GR") {

        G();
        Stop_Sumo(0);
        delay(200);

        if (counter_angle_Left == 1) {

          Serial.println("R");
        } else if (counter_angle_Right == 1) {

          Serial.println("L");
        }
      } else if (data == "N") {
        Nan();
        Stop_Sumo(0);
        delay(200);
        if (counter_angle_Left == 1) {

          Serial.println("R");
        } else if (counter_angle_Right == 1) {

          Serial.println("L");
        }
      }
      counter++;

      spd_Ob = 45;
    }
    Serial.begin(115200);



    if (counter != 0, counter < 5) {  //in the first loop after the start section (counter != 0, counter < 5) the arduino tack output from the raspberry pi (RR,RG,GR,GG) and save it in array (condition), and check if there are pink parking lot
      if (Serial.available()) {
        data = Serial.readStringUntil('#');

        Conditions[counter % 4] = data[counter % 4];
      }

      if (counter_angle_Left == 1) {        // the dirction of movment ClockWise
        if (data == "R" || data == "RR") {  //red pillar or pillars was ditected
       
            RR();
      
          Stop_Sumo(0);
          Serial.println("R");

          delay(200);
        } else if (data == "G" || data == "GG") {  //Green pillar or pillars was ditected
          Run_Sumo(spd_Ob);
          if (Pink[counter % 4])  //if there are parking lot
          {
            GGP();
          } else {
            GG();
          }
          Stop_Sumo(0);
          Serial.println("R");

          delay(200);
        } else if (data == "RG") {  //red and Green pillars was ditected
          if (Pink[counter % 4])    //if there are parking lot
          {
            RGP();
          } else {
            RG();
          }
          Stop_Sumo(0);
          Serial.println("R");

          delay(200);
        } else if (data == "GR") {  //Green and red pillars was ditected
          if (Pink[counter % 4])    //if there are parking lot
          {
            GR(42);
          } else {
            GR();
          }
          Stop_Sumo(0);
          Serial.println("R");

          delay(200);
        }
      }

      ///////////////////////////////////////////////////////////////////////////////////
      else if (counter_angle_Right == 1) {  // the dirction of movment CounterClockWise
        if (data == "R" || data == "RR") {
          if (Pink[counter % 4]) {
            RRP_CCW();
          } else {
            RR_CCW();
          }
          Stop_Sumo(0);
          Serial.println("L");

          delay(200);
        } else if (data == "G" || data == "GG") {
          Run_Sumo(spd_Ob);
        
            GG_CCW();
          
          Stop_Sumo(0);
          Serial.println("L");

          delay(200);
        } else if (data == "RG") {
          if (Pink[counter % 4]) {
            RG_CCW(33);
          } else {
            RG_CCW();
          }
          Stop_Sumo(0);
          Serial.println("L");

          delay(200);
        } else if (data == "GR") {
          if (Pink[counter % 4]) {
            GRP_CCW();
          } else {
            GR_CCW();
          }
          Stop_Sumo(0);
          Serial.println("L");

          delay(200);
        }
      }
      counter++;
    }


    if (counter >= 5, counter <= 12) {  //after the first loop the arduino check the Conditions and Pink array, which it saved them before without taking any images
                                        //the navigation of the robot does not differe from the previous If-Condtion


      if (counter_angle_Left == 1) {
        if (Conditions[counter % 4] == "R" || Conditions[counter % 4] == "RR") {

            RR();
          

        } else if (Conditions[counter % 4] == "G" || Conditions[counter % 4] == "GG") {
          Run_Sumo(spd_Ob);
          if (Pink[counter % 4]) {
            GGP();
          } else {
            GG();
          }

        } else if (Conditions[counter % 4] == "RG") {
          RG();

        } else if (Conditions[counter % 4] == "GR") {
          if (Pink[counter % 4]) {
            GR(42);
          } else {
            GR();
          }
        }
      }

      ///////////////////////////////////////////////////////////////////////////////////
      else if (counter_angle_Right == 1) {
        if (Conditions[counter % 4] == "R" || Conditions[counter % 4] == "RR") {
          if (Pink[counter % 4]) {
            RRP_CCW();
          } else {
            RR_CCW();
          }

        } else if (Conditions[counter % 4] == "G" || Conditions[counter % 4] == "GG") {
          Run_Sumo(spd_Ob);

            GG_CCW();
  


        } else if (Conditions[counter % 4] == "RG") {
          if (Pink[counter % 4]) {
            RG_CCW(33);
          } else {
            RG_CCW();
          }



        } else if (Conditions[counter % 4] == "GR") {
          if (Pink[counter % 4]) {
            GRP_CCW();
          } else {
            GR_CCW();
          }
        }
      }
      counter++;
    }



    if (counter > 12) {  // after three laps
    for   (int i = 0; i < 3;i++)
    {
      if (Pink[counter % 4])  //if the vehicl is in the same section of the parking lot
      {
        parking();
        Stop_Sumo(0);
        delay(10000000);

      } else  //if the vehicl is not in the same section of the parking lot
      {
        if (counter_angle_Left == 1)  //ClockWise moving direction
        {
          GG();
        } else  //CounterClockWise moving direction
        {
          RR();
        }
      }
    }
    }
  }