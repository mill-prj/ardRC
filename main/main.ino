#include <Wire.h>
#include <Servo.h>
#include "Adafruit_VL53L0X.h"

#define ruma sichi

#define SensorServoPin 9
#define EndBTN 22
#define resetLED 13

// ADXL335 아날로그 핀 설정
#define ADXL335_X_PIN A0
#define ADXL335_Y_PIN A1
#define ADXL335_Z_PIN A2

// ADXL335 캘리브레이션 값
#define ADXL335_ZERO_G 512    // 0G일때의 값 (중간값)
#define ADXL335_SCALE 102.4   // 1G당 변화값 (3.3V 기준)

// VL53L0X XSHUT pins
#define XSHUT_RIGHT 4
#define XSHUT_LEFT 5
#define XSHUT_FRONT 6
#define XSHUT_BACK 12

// Motor Driver (테스트 동안 비활성화)
const int RightF = 10; // fwd
const int RightB = 11; // bwd
const int LeftF = 7;  // fwd
const int LeftB = 8;  // bwd

Servo sensorServo;

#define VL53L0X_I2C_ADDR 0x29     // VL53L0X 기본 주소
#define SENSOR_RIGHT_ADDR 0x30
#define SENSOR_LEFT_ADDR 0x31
#define SENSOR_FRONT_ADDR 0x32
#define SENSOR_BACK_ADDR 0x33

// VL53L0X 센서들
Adafruit_VL53L0X SensorRight = Adafruit_VL53L0X();
Adafruit_VL53L0X SensorLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X SensorFront = Adafruit_VL53L0X();
Adafruit_VL53L0X SensorBack = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t rightv, leftv, frontv, backv;

short rightPrcd = 0, leftPrcd = 0, frontPrcd = 0, backPrcd = 0;

// 서보 제어 변수
int servoDegree = 45;      // 현재 서보 각도
unsigned long lastServoUpdate = 0;  // 마지막 서보 업데이트 시간
const int SERVO_DELAY = 3;         // 서보 업데이트 간격 (밀리초)
const int SERVO_STEP = 4;          // 한 번에 움직일 각도
bool servoDirection = true;        // true = 증가, false = 감소

// 데이터 수집 타이밍
unsigned long lastDataUpdate = 0;
const int DATA_INTERVAL = 10;      // 데이터 수집 간격 (밀리초)

void updateServoDirection();
void moveServo();
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

// VL53L0X 센서 초기화 함수
bool initVL53L0X(Adafruit_VL53L0X &sensor, int xshutPin, uint8_t newAddress, const char* sensorName) {
  digitalWrite(xshutPin, HIGH);
  delay(50);

  if (!sensor.begin(VL53L0X_I2C_ADDR)) {
    Serial1.print("Failed to init "); Serial1.println(sensorName);
    return false;
  }

  if (!sensor.setAddress(newAddress)) {
    Serial1.print("Failed to set address for "); Serial1.println(sensorName);
    return false;
  }

  delay(50);
  return true;
}

void setup() {

  Serial.begin(9600);   // Computer Serial
  Serial1.begin(9600);  // Bluetooth serial

  Serial.println("Sensor Test Mode - Motors Disabled");
  Serial.println("System initializing...");
  Serial1.println("Out of Range = -1");
  Serial1.println("right, left, front, back, degree, forcex, forcey, forcez");

  Wire.begin();

  // VL53L0X 센서 초기화
  // XSHUT 핀 설정
  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_BACK, OUTPUT);

  // 모든 센서를 비활성화
  digitalWrite(XSHUT_RIGHT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_BACK, LOW);
  delay(100);  // 완전한 비활성화를 위해 충분한 대기 시간

  bool initSuccess = true;

  // 각 센서 순차적으로 초기화
  initSuccess &= initVL53L0X(SensorRight, XSHUT_RIGHT, SENSOR_RIGHT_ADDR, "Right Sensor");
  initSuccess &= initVL53L0X(SensorLeft, XSHUT_LEFT, SENSOR_LEFT_ADDR, "Left Sensor");
  initSuccess &= initVL53L0X(SensorFront, XSHUT_FRONT, SENSOR_FRONT_ADDR, "Front Sensor");
  initSuccess &= initVL53L0X(SensorBack, XSHUT_BACK, SENSOR_BACK_ADDR, "Back Sensor");

  if (!initSuccess) {
    Serial1.println("One or more sensors failed to initialize. Check connections.");
    // 오류 상태 표시 (LED 깜박임)
    pinMode(LED_BUILTIN, OUTPUT);
    while(1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  Serial1.println("All sensors initialized successfully");

  // ADXL335 핀 모드 설정
  pinMode(ADXL335_X_PIN, INPUT);
  pinMode(ADXL335_Y_PIN, INPUT);
  pinMode(ADXL335_Z_PIN, INPUT);
  
  // MG996R 서보모터 초기화
  sensorServo.attach(SensorServoPin);
  servoDegree = 45;  // 초기 서보 각도 설정
  sensorServo.write(servoDegree);
  
  // 모터 드라이버 설정
  pinMode(RightF, OUTPUT); pinMode(RightB, OUTPUT); 
  pinMode(LeftF, OUTPUT); pinMode(LeftB, OUTPUT);

  // RESET BTN setup
  pinMode(EndBTN, INPUT); pinMode(resetLED, OUTPUT);

  // Force Sensor
  cForce.x = 0;
  cForce.y = 0;
  cForce.z = 0;
}

void loop() {

  unsigned long currentMillis = millis();
  
  // 서보모터 움직임 업데이트 (5ms 간격)
  moveServo();
  
  char in = Serial1.read();
  if(in == "" ||  in == NULL) {
    
  } else {
    if (in == 'F') {
      motor().moveF(200);
    } 
    else if (in == 'B') {
      motor().moveB(200);
    } 
    else if (in == 'L') {
      motor().steerLeft();
    } 
    else if (in == 'R') {
      motor().SteerRight();
    } 
    else {
  
    }

  }
  // 데이터 수집 및 전송 (10ms 간격 = 초당 100회)
  if (currentMillis - lastDataUpdate >= DATA_INTERVAL) {
    readRange();  // 거리 센서 읽기
    readForce();  // 가속도 센서 읽기
    
    String out = String(rightPrcd) + "," + String(leftPrcd) + "," + String(frontPrcd) + "," + String(backPrcd) + ","
      + String(servoDegree) + "," // Servo Value
      + String(cForce.x)+ "," + String(cForce.y) + "," + String(cForce.z); // 가속도
    Serial1.println(out);
    Serial.println(out);
    
    lastDataUpdate = currentMillis;
  }
  
  // 테스트를 위해 모터 동작은 비활성화
  motor().allStop();
  
  if(digitalRead(EndBTN) == 1) {
    endSystem();
  }

}
void moveServo() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastServoUpdate >= SERVO_DELAY) {
    // 방향에 따라 각도 조정
    if (servoDirection) {
      servoDegree += SERVO_STEP;
      if (servoDegree >= 90) {
        servoDegree = 90;
        servoDirection = false;
      }
    } else {
      servoDegree -= SERVO_STEP;
      if (servoDegree <= 0) {
        servoDegree = 0;
        servoDirection = true;
      }
    }
    
    // 서보모터 업데이트
    sensorServo.write(servoDegree);
    lastServoUpdate = currentMillis;
  }
}

void resetServo() {
  servoDegree = 45;  // 중앙 위치로 설정
  servoDirection = true;  // 방향 초기화
  sensorServo.write(servoDegree);
  delay(500);  // 서보가 원위치로 돌아갈 시간 제공
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
  // ADXL335에서 raw 데이터 읽기
  int rawX = analogRead(ADXL335_X_PIN);
  int rawY = analogRead(ADXL335_Y_PIN);
  int rawZ = analogRead(ADXL335_Z_PIN);
  
  // 0G를 기준으로 가속도 계산 (단위: G)
  float gX = (rawX - ADXL335_ZERO_G) / ADXL335_SCALE;
  float gY = (rawY - ADXL335_ZERO_G) / ADXL335_SCALE;
  float gZ = (rawZ - ADXL335_ZERO_G) / ADXL335_SCALE;
  
  // 값을 0-1023 범위로 매핑 (Python 코드와의 호환성을 위해)
  cForce.x = constrain(map((int)(gX * 100), -100, 100, 0, 1023), 0, 1023);
  cForce.y = constrain(map((int)(gY * 100), -100, 100, 0, 1023), 0, 1023);
  cForce.z = constrain(map((int)(gZ * 100), -100, 100, 0, 1023), 0, 1023);
}
