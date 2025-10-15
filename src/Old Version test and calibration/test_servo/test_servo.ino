#include <Servo.h>

Servo myservo;  // create servo object to control a servo

void setup() {
  myservo.attach(8);  // attaches the servo on pin 8 to the servo object
}

void loop() {
  myservo.write(60);                  // sets the servo position according to the 60 value
  delay(1000); 
    myservo.write(90);                  // sets the servo position according to the 90 value
  delay(1000); 
    myservo.write(30);                  // sets the servo position according to the 30 value
  delay(1000); 
                           // waits for the servo to get there
}
