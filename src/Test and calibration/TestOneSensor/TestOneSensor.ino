#include <NewPing.h>

#define TRIGGER_PIN 24
#define ECHO_PIN 25
#define MAX_DISTANCE 300

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);


double duration = 0;
double d = 0;

double get_distance() {
  duration = sonar.ping();
  d = ((duration / 2) * 0.0343);
  //mean_fr = Filter(fr, mean_fr, Means_FR);
  return d;
}

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(9600);
}

void loop() {
  Serial.println(get_distance());
  delay(10);
}
