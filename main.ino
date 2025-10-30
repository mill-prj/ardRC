#include <Wire.h>
#include <Servo.h>
#include "Adafruit_VL53L0X.h"

#define ruma sichi

#define SenseorServo 9

// Motor Driver
#define RightA 10
#define RightB 11
#define LeftA 7
#define LeftB 8


#define add_e 0x00
#define add_w 0x00
#define add_n 0x00
#define add_s 0x00

#define sht_e 2
#define sht_w 3
#define sht_n 4
#define sht_s 5

Adafruit_VL53L0X SensorEast = Adafruit_VL53L0X();
Adafruit_VL53L0X SensorWest = Adafruit_VL53L0X();
Adafruit_VL53L0X SensorNorth = Adafruit_VL53L0X();
Adafruit_VL53L0X SensorSouth = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t eastv, westv, northv, southv;

short eastPrcd = 0, westPrcd = 0, northPrcd = 0, southPrcd = 0;

short servoDegree;

int ServoMove(short degree);
void readRange();


void setup() {
  Serial.begin(9600); // Computer Serial
  Serial1.begin(9600); // Bluetooth serial

  Serial.println("testing");
  Serial1.println("Out of Range = -1");
  Serial1.println("east, west, north, south, forcex, forcey, forcez");

  Wire.begin();

  // Setup Page - Ranging Modules
  pinMode(sht_e, OUTPUT); pinMode(sht_w, OUTPUT); pinMode(sht_n, OUTPUT); pinMode(sht_s, OUTPUT); 
    // all reset
    digitalWrite(sht_e, LOW); digitalWrite(sht_w, LOW); digitalWrite(sht_n, LOW); digitalWrite(sht_s, LOW);
    delay(10);
    // 
    digitalWrite(sht_e, HIGH); digitalWrite(sht_w, HIGH); digitalWrite(sht_n, HIGH); digitalWrite(sht_s, HIGH);

  if(!SensorEast.begin(add_e)) {
    Serial1.println("Failed to Load East");
    while(1);
  }
  delay(10);
  if(!SensorWest.begin(add_w)) {
    Serial1.println("Failed to Load West");
    while(1);
  }
  delay(10);
  if(!SensorSouth.begin(add_s)) {
    Serial1.println("Failed to Load South");
    while(1);
  }
  delay(10);
  if(!SensorNorth.begin(add_n)) {
    Serial1.println("Failed to Load North");
    while(1);
  } 

  // Ranging Sensor End


}

void loop() {
  readRange();
  String out = String(eastPrcd) + "," + String(westPrcd) + "," + String(southPrcd) + "," + String(northPrcd) + ","
    + String(servoDegree) + "," // Servo Value
    + "x" + "," + "y" + "," + "z"; // 가속도
  Serial1.println(out);
}

int ServoMove(short degree) {
  if(degree == 0)
    return 1;
  else {
    servoDegree += degree;


  }
}

void readRange() {
  SensorEast.rangingTest(&eastv, false);
  SensorWest.rangingTest(&westv, false);
  SensorSouth.rangingTest(&southv, false);
  SensorNorth.rangingTest(&northv, false);

  if(eastv.RangeStatus != 4) {
    eastPrcd = eastv.RangeMilliMeter;
  } else {
    eastPrcd = -1;
  }

  if(westv.RangeStatus != 4) {
    westPrcd = westv.RangeMilliMeter;
  } else {
    westPrcd = -1;
  }

  if(southv.RangeStatus != 4) {
    southPrcd = southv.RangeMilliMeter;
  } else {
    southPrcd = -1;
  }

  if(northv.RangeStatus != 4) {
    northPrcd = northv.RangeMilliMeter;
  } else {
    northPrcd = -1;
  }
  
}
