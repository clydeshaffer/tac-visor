
#include <Adafruit_PWMServoDriver.h>

int buttonState = 0;    

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

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

struct Point {
  float x; float y;
};

float Dot(Point a, Point b) {
  return (a.x * b.x) + (a.y * b.y);
}

void Barycentric(Point p, Point a, Point b, Point c, float &u, float &v, float &w)
{
    Vector v0 = b - a, v1 = c - a, v2 = p - a;
    float d00 = Dot(v0, v0);
    float d01 = Dot(v0, v1);
    float d11 = Dot(v1, v1);
    float d20 = Dot(v2, v0);
    float d21 = Dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
}

void loop() {
  // put your main code here, to run repeatedly:
  buttonState = digitalRead(5);
  Serial.println(buttonState);
  if(buttonState == 1) {
    pwm.setPWM(0, 0, 512);
  } else {
    pwm.setPWM(0, 0, 256);
  }
  delay(500);
}
