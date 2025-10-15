#include <ArduinoQueue.h>
#include <NewPing.h>
#include <Servo.h>


#define TRIGGER_PIN_L1 44
#define ECHO_PIN_L1 45

#define TRIGGER_PIN_L2 24
#define ECHO_PIN_L2 25

#define TRIGGER_PIN_R1 32
#define ECHO_PIN_R1 33

#define TRIGGER_PIN_R2 29
#define ECHO_PIN_R2 28

#define TRIGGER_PIN_F 38
#define ECHO_PIN_F 39

#define D2R_trigger 50
#define D2R_echo 51

#define D2L_trigger 41
#define D2L_echo 40

#define B_trigger 48
#define B_echo 49

#define BL_trigger 22
#define BL_echo 23

#define BR_trigger 46
#define BR_echo 47
#define MAX_DISTANCE 300




double distance = 0, real_distance = 0, Real_dis_L = 0, Real_dis_R = 0, dis_L1 = 0, dis_L2 = 0, dis_R1 = 0, dis_R2 = 0, dis_F = 0, dis_D2L = 0, dis_D2R = 0, dis_B = 0, dis_BL = 0, dis_BR = 0;

double duration_F = 0, duration_L1 = 0, duration_L2 = 0, duration_R1 = 0, duration_R2 = 0, duration_D2L = 0, duration_D2R = 0, duration_B = 0, duration_BL = 0, duration_BR = 0;


////////////////PID__C

NewPing sonar_L1(TRIGGER_PIN_L1, ECHO_PIN_L1, MAX_DISTANCE);
NewPing sonar_L2(TRIGGER_PIN_L2, ECHO_PIN_L2, MAX_DISTANCE);
NewPing sonar_R1(TRIGGER_PIN_R1, ECHO_PIN_R1, MAX_DISTANCE);
NewPing sonar_R2(TRIGGER_PIN_R2, ECHO_PIN_R2, MAX_DISTANCE);
NewPing sonar_F(TRIGGER_PIN_F, ECHO_PIN_F, MAX_DISTANCE);
NewPing sonar_D2R(D2R_trigger, D2R_echo, MAX_DISTANCE);
NewPing sonar_D2L(D2L_trigger, D2L_echo, MAX_DISTANCE);
NewPing sonar_B(B_trigger, B_echo, MAX_DISTANCE);
NewPing sonar_BL(BL_trigger, BL_echo, MAX_DISTANCE);
NewPing sonar_BR(BR_trigger, BR_echo, MAX_DISTANCE);



double f, fl, bl, fr, br, b;

/////////////////////////////
double get_distance_F() {
  duration_F = sonar_F.ping();
  f = (duration_F / 2) * 0.0343;
  //mean_f = Filter(f, mean_f, Means_F);
  return f;
}

double get_distance_B() {
  duration_B = sonar_B.ping();
  b = ((duration_B / 2) * 0.0343);
  //  mean_b = Filter(b, mean_b, Means_B);
  return b;
}

double get_distance_BL() {
  duration_BL = sonar_BL.ping();
  return ((duration_BL / 2) * 0.0343);
}
double get_distance_BR() {
  duration_BR = sonar_BR.ping();
  return ((duration_BR / 2) * 0.0343);
}

double get_distance_L1() {
  duration_L1 = sonar_L1.ping();
  fl = ((duration_L1 / 2) * 0.0343) - 4;
  //mean_fl = Filter(fl, mean_fl, Means_FL);
  return fl;
}

double get_distance_L2() {
  duration_L2 = sonar_L2.ping();
  bl = ((duration_L2 / 2) * 0.0343) - 4;
  //mean_bl = Filter(bl, mean_bl, Means_BL);
  return bl;
}

double get_distance_R1() {
  duration_R1 = sonar_R1.ping();
  fr = ((duration_R1 / 2) * 0.0343) - 4;
  //mean_fr = Filter(fr, mean_fr, Means_FR);
  return fr;
}

double get_distance_R2() {
  duration_R2 = sonar_R2.ping();
  br = ((duration_R2 / 2) * 0.0343) - 4;
  //mean_br = Filter(br, mean_br, Means_BR);
  return br;
}


double get_distance_D2R() {
  duration_D2R = sonar_D2R.ping();
  if (((duration_D2R / 2) * 0.0343) == 0)
    return 100;
  return ((duration_D2R / 2) * 0.0343);
}

double get_distance_D2L() {
  duration_D2L = sonar_D2L.ping();
  if (((duration_D2L / 2) * 0.0343) == 0)
    return 100;
  return ((duration_D2L / 2) * 0.0343) - 3;
}
void sensor() {
  dis_R1 = get_distance_R1();
  dis_L2 = get_distance_L2();
  delay(15);
  dis_L1 = get_distance_L1();
  dis_R2 = get_distance_R2();
  delay(15);
  dis_B = get_distance_B();
  dis_F = get_distance_F();
  delay(15);
  dis_D2L = get_distance_D2L();
  delay(10);
  dis_D2R = get_distance_D2R();
  delay(10);
  dis_BR = get_distance_BR();
  delay(10);
  dis_BL = get_distance_BL();
  delay(10);
}


void setup() {
  Serial.begin(115200); //begain a Serial reading of port 115200
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
  pinMode(D2R_trigger, OUTPUT);
  pinMode(D2R_echo, INPUT);
  pinMode(D2L_trigger, OUTPUT);
  pinMode(D2L_echo, INPUT);
  pinMode(BL_trigger, OUTPUT);
  pinMode(BL_echo, INPUT);
  pinMode(BR_trigger, OUTPUT);
  pinMode(BR_echo, INPUT);
  pinMode(B_trigger, OUTPUT);
  pinMode(B_echo, INPUT);
}
void loop() {
  sensor();

  Serial.print("dis_L1 ");
  Serial.println(dis_L1);
  Serial.print("dis_L2 ");
  Serial.println(dis_L2);
  Serial.print("dis_R1 ");
  Serial.println(dis_R1);
  Serial.print("dis_R2 ");
  Serial.println(dis_R2);
  Serial.print("dis_F ");
  Serial.println(dis_F);
  Serial.print("dis_B ");
  Serial.println(dis_B);
  Serial.print("dis_D2L ");
  Serial.println(dis_D2L);
  Serial.print("dis_D2R ");
  Serial.println(dis_D2R);
  Serial.print("dis_BR ");
  Serial.println(dis_BR);
  Serial.print("dis_BL ");
  Serial.println(dis_BL);
}
