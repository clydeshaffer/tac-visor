#include <AutoPID.h>

#include <Pixy.h>
#include <PixyI2C.h>
#include <PixySPI_SS.h>

#include <TPixy.h>

Pixy pixy;

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)       
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

#include <Adafruit_PWMServoDriver.h>

#define ROOT3 1.732050

//pid settings and gains
#define OUTPUT_MIN -50
#define OUTPUT_MAX 50
#define KP .12
#define KI .0003
#define KD 0


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

double panErr, panSet, panOut;
double tilErr, tilSet, tilOut;

AutoPID panPID(&panErr, &panSet, &panOut, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID tilPID(&tilErr, &tilSet, &tilOut, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

void setup() {
  // put your setup code here, to run once:
  pinMode(5, INPUT);
  Serial.begin(9600);
  Serial.println("begin");
  pixy.init();

  panSet = 0;
  tilSet = 0;
  panErr = 0;
  tilErr = 0;

  panPID.setTimeStep(10);
  tilPID.setTimeStep(10);
  
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
  int i, closest_i = 0;
  static int printcnt = 0;
  uint16_t blocks;
  char buf[32]; 
  int32_t panError, tiltError, panError2, tiltError2, dist, dist2;
  blocks = pixy.getBlocks();
  if(blocks) {
    dist=(1<<30)-1;
    for(i=0;i<blocks;i++) {
      panError2 = X_CENTER-pixy.blocks[0].x;
      tiltError2 = pixy.blocks[0].y-Y_CENTER;
      dist2 = panError2 * panError2 + tiltError2 * tiltError2;
      if(dist2 < dist) {
        dist = dist2;
        closest_i = i;
      }
    }
    
    panError = X_CENTER-pixy.blocks[closest_i].x;
    tiltError = pixy.blocks[closest_i].y-Y_CENTER;

    panErr = (double) panError;
    tilErr = (double) tiltError;

    panPID.run();
    tilPID.run();

    setMotors(panOut / 100, tilOut / 100);
    
    printcnt++;
    if(printcnt%50 == 0) {
      Serial.print("Tracking block ");
      Serial.println(closest_i);
      Serial.print(panOut);
      Serial.print("\t");
      Serial.println(tilOut);
    }
  } else {
    setMotors(0, 0);
  }
}
