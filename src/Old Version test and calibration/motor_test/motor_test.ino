#define RPWM_Output_1 2 
#define LPWM_Output_1 3 
#define R_EN 34
#define L_EN 35

void setup() {
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(LPWM_Output_1, OUTPUT);
  pinMode(RPWM_Output_1, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
}

void loop() {
  analogWrite(LPWM_Output_1, 111);
  analogWrite(RPWM_Output_1, 0);
}
