// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>
#include <ros.h>
#include <std_msgs/Float32.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9
#define SWITCH_PIN 36

// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET 15

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

ros::NodeHandle nh;
std_msgs::Float32 float_msg;
ros::Publisher gyro_pub("gyro_sensor", &float_msg);
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;


void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}
void setup(void) {
  pinMode(SWITCH_PIN,INPUT_PULLDOWN);
  nh.initNode();
  nh.advertise(gyro_pub);
  Serial.begin(115200);
  nh.getHardware()->setPort(&Serial);
  while(!digitalRead(SWITCH_PIN)){delay(10);}
  delay(1000);
  bno08x.hardwareReset();
  // Try to initialize!
  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    while (1) { delay(10); }
  }


  setReports();
  
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
}


void loop() {
  int start=millis();

  if (bno08x.wasReset()) {
    setReports();
  }
  
  if (! bno08x.getSensorEvent(&sensorValue)) {
    return;
  }
  
  switch (sensorValue.sensorId) {
    
    case SH2_GAME_ROTATION_VECTOR:
      quaternionToEuler(sensorValue.un.gameRotationVector.real,sensorValue.un.gameRotationVector.i,sensorValue.un.gameRotationVector.j,sensorValue.un.gameRotationVector.k,&ypr,true);
      //Serial.print("Game Rotation Vector - yaw: ");
      float_msg.data = ypr.yaw;
      Serial.println(ypr.yaw);
      gyro_pub.publish(&float_msg);
      nh.spinOnce();
      break;
  }
  int end=millis();
  delay(35-(end-start));
}