#include <Wire.h>

// XSHUT 핀 할당
#define XSHUT_RIGHT 4
#define XSHUT_LEFT 5
#define XSHUT_FRONT 6
#define XSHUT_BACK 12

// VL53L0X I2C 주소 관련
#define VL53L0X_DEFAULT_ADDRESS 0x29
#define VL53L0X_I2C_ID_REG      0xC0
#define VL53L0X_I2C_ADDR_REG    0x8A

void setup() {
  Serial.begin(9600);
  while(!Serial) { delay(1); }
  
  Wire.begin();
  Serial.println("\nVL53L0X I2C 주소 설정 시작");
  
  // 모든 센서 끄기
  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_BACK, OUTPUT);
  
  digitalWrite(XSHUT_RIGHT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_BACK, LOW);
  delay(100);
  
  // 센서 1 (RIGHT) - 0x30으로 설정
  Serial.println("\n=== 센서 1 (RIGHT) 초기화 ===");
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(100);
  
  if (checkSensor(VL53L0X_DEFAULT_ADDRESS)) {
    if (changeAddress(VL53L0X_DEFAULT_ADDRESS, 0x30)) {
      Serial.println("센서 1 주소 변경 성공 (0x30)");
    } else {
      Serial.println("센서 1 주소 변경 실패");
    }
  } else {
    Serial.println("센서 1 응답 없음 - 배선 확인 필요");
  }
  
  // 센서 2 (LEFT) - 0x31로 설정
  Serial.println("\n=== 센서 2 (LEFT) 초기화 ===");
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(100);
  
  if (checkSensor(VL53L0X_DEFAULT_ADDRESS)) {
    if (changeAddress(VL53L0X_DEFAULT_ADDRESS, 0x31)) {
      Serial.println("센서 2 주소 변경 성공 (0x31)");
    } else {
      Serial.println("센서 2 주소 변경 실패");
    }
  } else {
    Serial.println("센서 2 응답 없음 - 배선 확인 필요");
  }

  // 센서 3 (FRONT) - 0x32로 설정
  Serial.println("\n=== 센서 3 (FRONT) 초기화 ===");
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(100);
  
  if (checkSensor(VL53L0X_DEFAULT_ADDRESS)) {
    if (changeAddress(VL53L0X_DEFAULT_ADDRESS, 0x32)) {
      Serial.println("센서 3 주소 변경 성공 (0x32)");
    } else {
      Serial.println("센서 3 주소 변경 실패");
    }
  } else {
    Serial.println("센서 3 응답 없음 - 배선 확인 필요");
  }

  // 센서 4 (BACK) - 0x33으로 설정
  Serial.println("\n=== 센서 4 (BACK) 초기화 ===");
  digitalWrite(XSHUT_BACK, HIGH);
  delay(100);
  
  if (checkSensor(VL53L0X_DEFAULT_ADDRESS)) {
    if (changeAddress(VL53L0X_DEFAULT_ADDRESS, 0x33)) {
      Serial.println("센서 4 주소 변경 성공 (0x33)");
    } else {
      Serial.println("센서 4 주소 변경 실패");
    }
  } else {
    Serial.println("센서 4 응답 없음 - 배선 확인 필요");
  }
  
  Serial.println("\n=== I2C 버스 스캔 결과 ===");
  scanI2C();
}

bool checkSensor(byte address) {
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.print("I2C 장치 발견 at 0x");
    Serial.println(address, HEX);
    
    // VL53L0X ID 확인
    Wire.beginTransmission(address);
    Wire.write(VL53L0X_I2C_ID_REG);
    Wire.endTransmission(false);
    
    Wire.requestFrom(address, (byte)1);
    if (Wire.available()) {
      byte id = Wire.read();
      Serial.print("장치 ID: 0x");
      Serial.println(id, HEX);
      return true;
    }
  } else {
    Serial.print("통신 오류 코드: ");
    Serial.println(error);
  }
  return false;
}

bool changeAddress(byte oldAddr, byte newAddr) {
  Serial.print("주소 변경 시도: 0x");
  Serial.print(oldAddr, HEX);
  Serial.print(" -> 0x");
  Serial.println(newAddr, HEX);
  
  // I2C 주소 레지스터에 새 주소 쓰기
  Wire.beginTransmission(oldAddr);
  Wire.write(VL53L0X_I2C_ADDR_REG);
  Wire.write(newAddr);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    delay(100);  // 주소 변경 대기
    
    // 새 주소로 통신 테스트
    Wire.beginTransmission(newAddr);
    error = Wire.endTransmission();
    
    if (error == 0) {
      return true;
    }
  }
  return false;
}

void scanI2C() {
  byte error, address;
  int nDevices = 0;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0) {
      Serial.print("I2C 장치 발견: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("I2C 장치를 찾을 수 없음");
  }
}

void loop() {
  delay(1000);
}
