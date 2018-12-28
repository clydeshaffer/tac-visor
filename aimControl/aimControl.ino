#include <Adafruit_PWMServoDriver.h>

#define ROOT3 1.732050

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int angle = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(5, INPUT);
  Serial.begin(9600);
  Serial.println("begin");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(100);
  pwm.setPWM(0, 0, 256);
  pwm.setPWM(1, 0, 256);
  pwm.setPWM(2, 0, 256);
  delay(1000);
  pwm.setPWM(0, 0, 512);
  pwm.setPWM(1, 0, 512);
  pwm.setPWM(2, 0, 512);
  delay(2000);
  pwm.setPWM(0, 0, 256);
  pwm.setPWM(1, 0, 256);
  pwm.setPWM(2, 0, 256);
  delay(2000);
}

void setMotors(double x, double y) {
  double l1, l2, l3;
  l1 = ((2 * y) + 1) / 3;
  l2 = (1 - y - (x*ROOT3)) / 3;
  l3 = 1 - (l1 + l2);

  pwm.setPWM(0, 0, l1 * 256 + 256);
  pwm.setPWM(1, 0, l2 * 256 + 256);
  pwm.setPWM(2, 0, l3 * 256 + 256);
}

void loop() {
  // put your main code here, to run repeatedly:
  angle++;
  angle = angle % 360;
  setMotors(cos(angle * PI / 180)/2, sin(angle * PI / 180)/2);
  Serial.println(angle);
  delay(50);
}
