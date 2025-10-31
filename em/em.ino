#include <Servo.h>

#define pServo 2
#define tServo 3

#define LeftF 4
#define LeftB 5
#define RightF 6
#define RightB 7

Servo artServo;
Servo topServo;

class motor {
public:
  void allStop() {
    analogWrite(RightF, 0);
    analogWrite(RightB, 0);
    analogWrite(LeftF, 0);
    analogWrite(LeftB, 0);
  }

  void moveF(short spd) {
    //allStop();
    analogWrite(RightF, spd);
    analogWrite(LeftF, spd);
  }

  void moveB(short spd) {
    //allStop();
    analogWrite(RightB, spd);
    analogWrite(LeftB, spd);
  }
  void steerLeft() {
    //allStop();
    analogWrite(RightB, 100);
    analogWrite(LeftF, 100);
  }

  void SteerRight() {
    //allStop();
    analogWrite(RightF, 100);
    analogWrite(LeftB, 100);
  }
};

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(pServo, OUTPUT);
  pinMode(tServo, OUTPUT);
  pinMode(LeftF, OUTPUT);
  pinMode(LeftB, OUTPUT);
  pinMode(RightF, OUTPUT);
  pinMode(RightB, OUTPUT);

  artServo.attach(pServo);
  topServo.attach(tServo);
}

void loop() {
  char in = Serial1.readString();
  // Movment + Direction = "Mㅁ"
  if(in == "MS") {
    motor().allStop();
  } else if (in == "MF") {
    motor().moveF();
  } else if (in == "MB") {
    motor().moveB();
  } else if (in == "MR") {
    motor().SteerRight();
  } else if (in == "ML") {
    motor().SteerLeft();
  } else if (in == "AU") {  // Arms up 포신을 올림
    artServo.write(5);
  } else if (in == "AD") {  //Arms down 포신을 내림
    artServo.write(-5);
  } else if (in == "AS") {  //Arms set 포신을 기본으로 설정함
    short deg = artServo.read();
    short mv = deg * (-1);
    artServo.write(mv);
  } else if("AF") { // 포 발사
    motor().allStop();
    delay(5000); // 방열을 위해 5초간 stop 한다.
  }else if ("TL") { // 주포 좌로 회전
    topServo.write(-5);
  } else if("TR") { // 주포 우로 회전
    topServo.write(5);
  } else if("TS") { // 주포 원상 복구
    short deg = topServo.read();
    short mv = deg * (-1);
    topServo.write(mv);
  }else {
    Serial1.println("Invalid Command");
  }
}