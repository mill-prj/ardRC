#include <Wire.h>
#include <Servo.h>
#include "Adafruit_VL53L0X.h"

#define ruma sichi

#define SensorServoPin 9
#define EndBTN 22
#define resetLED 13

#define stMS 1000

// Motor Driver
#define RightF 10 // fwd
#define RightB 11 // bwd
#define LeftF 7 // fwd
#define LeftB 8 //bwd

#define forceX A0
#define forceY A1
#define forceZ A2

Servo sensorServo;

#define add_right 0x00
#define add_left 0x00
#define add_front 0x00
#define add_back 0x00


Adafruit_VL53L0X SensorRight = Adafruit_VL53L0X();
Adafruit_VL53L0X SensorLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X SensorFront = Adafruit_VL53L0X();
Adafruit_VL53L0X SensorBack = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t rightv, leftv, frontv, backv;

short rightPrcd = 0, leftPrcd = 0, frontPrcd = 0, backPrcd = 0;

short servoDegree = 0;
short targetedDirection = 1; 
/*
  1 is positive move
  -1 is netagive move
  0 is 0
*/

short ServoMoveCaul(short currentDegree, bool byPass = false);
int ServoMove(short degree);
void resetServo();
void readRange();
void endSystem();
void readForce();

typedef struct Forces {
  short x;
  short y;
  short z;
};

Forces cForce;
class motor {
  public:
  void allStop() {
    analogWrite(RightF, 0); analogWrite(RightB, 0);
    analogWrite(LeftF, 0); analogWrite(LeftB, 0);
  }

  void moveF(short spd) {
    allStop();
    analogWrite(RightF, spd); analogWrite(LeftF, spd);
  }

  void moveB(short spd) {
    allStop();
    analogWrite(RightB, spd); analogWrite(LeftB, spd);
  }
  void steerLeft() {
    allStop();
    analogWrite(RightB, 100); analogWrite(LeftF, 100);
  }

  void SteerRight() {
    allStop();
    analogWrite(RightF, 100); analogWrite(LeftB, 100);
  }
};

void setup() {
  Serial.begin(9600); // Computer Serial
  Serial1.begin(9600); // Bluetooth serial

  Serial.println("testing");
  Serial1.println("Out of Range = -1");
  Serial1.println("right, left, front, back, degree, forcex, forcey, forcez");

  Wire.begin();
  
  if(!SensorRight.begin(add_right)) {
    Serial1.println("Failed to Load East");
    while(1);
  }
  delay(10);
  if(!SensorLeft.begin(add_left)) {
    Serial1.println("Failed to Load West");
    while(1);
  }
  delay(10);
  if(!SensorBack.begin(add_back)) {
    Serial1.println("Failed to Load South");
    while(1);
  }
  delay(10);
  if(!SensorFront.begin(add_front)) {
    Serial1.println("Failed to Load North");
    while(1);
  } 

  // Ranging Sensor End
  // motordriver setup
  pinMode(RightF, OUTPUT); pinMode(RightB, OUTPUT); pinMode(LeftF, OUTPUT); pinMode(LeftB, OUTPUT);
  sensorServo.attach(SensorServoPin);

  // RESET BTN setup
  pinMode(EndBTN, INPUT); pinMode(resetLED, OUTPUT);

  // Force Sensor
  cForce.x = 0;
  cForce.y = 0;
  cForce.z = 0;
}

void loop() {
  readRange();
  String input = Serial1.readString();

  String out = String(rightPrcd) + "," + String(leftPrcd) + "," + String(frontPrcd) + "," + String(backPrcd) + ","
    + String(servoDegree) + "," // Servo Value
    + String(cForce.x)+ "," + String(cForce.y) + "," + String(cForce.z); // 가속도
  Serial1.println(out); 

  // obstacle utjugo system
  if(frontPrcd <= 100){
    motor().allStop();
    delay(780);
    motor().SteerRight();
    delay(/*declared seconds*/stMS);
    if(frontPrcd <= 100) {
      motor().allStop();
    motor().steerLeft();
    delay(/*declared seconds \TIMES 2*/ stMS * 2);
    }
    ServoMove(ServoMoveCaul(servoDegree));
  }
  
  if(digitalRead(EndBTN) == 1) {
    endSystem();
  }
}
short ServoMoveCaul(short currentDegree, bool byPass = false) {
  if(byPass == true) {
    goto mreturn;
  }
  if(currentDegree == 90 || currentDegree == -90) {
    targetedDirection *= -1;
    return ServoMoveCaul(currentDegree, true);
  }
  if(byPass == false) {
  mreturn:
    switch(targetedDirection) {
      case -1:
        return -5;
        break;
      case 1:
        return 5;
        break;
      default:
        endSystem();
    }
  }
  
}

int ServoMove(short degree) {
  if(degree == 0 || (servoDegree + degree) > 90 || (servoDegree + degree) < -90)
    return 1;
  else {
    servoDegree += degree;
    sensorServo.write(degree);
  }
}

void resetServo() {
  sensorServo.write(-1 * servoDegree);
  servoDegree = 0;
}

void readRange() {
  SensorRight.rangingTest(&rightv, false);
  SensorLeft.rangingTest(&leftv, false);
  SensorBack.rangingTest(&backv, false);
  SensorFront.rangingTest(&frontv, false);

  if(rightv.RangeStatus != 4) {
    rightPrcd = rightv.RangeMilliMeter;
  } else {
    rightPrcd = -1;
  }

  if(leftv.RangeStatus != 4) {
    leftPrcd = leftv.RangeMilliMeter;
  } else {
    leftPrcd = -1;
  }

  if(backv.RangeStatus != 4) {
    backPrcd = backv.RangeMilliMeter;
  } else {
    backPrcd = -1;
  }

  if(frontv.RangeStatus != 4) {
    frontPrcd = frontv.RangeMilliMeter;
  } else {
    frontPrcd = -1;
  }
  
}

void endSystem() {
  motor().allStop();
  Serial.println("Stopped Motors");
  resetServo();
  Serial.println("Stopped Servos");
  Serial.println("Finalzing...");

  rightPrcd = -2; leftPrcd = -2; frontPrcd = -2; backPrcd = -1;
  cForce.x = -2; cForce.y = -2; cForce.z = -2;
  String out = String(rightPrcd) + "," + String(leftPrcd) + "," + String(frontPrcd) + "," + String(backPrcd) + ","
    + String(servoDegree) + "," // Servo Value
    + String(cForce.x)+ "," + String(cForce.y) + "," + String(cForce.z); // 가속도
  Serial1.println(out);
  Serial1.println("You can Power off the board");
  Serial1.println("THIS IS LAST MSG IN DEVICE");

  for(int i = 0;1; i++) {
    Serial.println("Stopped." + String(i) + "sec");
    digitalWrite(resetLED, HIGH);
    delay(500);
    digitalWrite(resetLED, LOW);
    delay(500);
  }
}

void readForce() {
  cForce.x = analogRead(forceX);
  cForce.y = analogRead(forceY);
  cForce.z = analogRead(forceZ);
}
