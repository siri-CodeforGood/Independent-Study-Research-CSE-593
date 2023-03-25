/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#define DEV_I2C Wire
#define SerialPort Serial

#ifndef LED_BUILTIN
  #define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN

// Components.
VL53L4CD sensor_vl53l4cd_sat(&DEV_I2C, A1);

/* Setup ---------------------------------------------------------------------*/
#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  //Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  //Serial.println("Starting..."); // print some text in Serial Monitor
  
   // Led.
  pinMode(LedPin, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);
  //SerialPort.println("Starting...");

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L4CD satellite component.
  sensor_vl53l4cd_sat.begin();

  // Switch off VL53L4CD satellite component.
  sensor_vl53l4cd_sat.VL53L4CD_Off();

  //Initialize VL53L4CD satellite component.
  sensor_vl53l4cd_sat.InitSensor();

  // Program the highest possible TimingBudget, without enabling the
  // low power mode. This should give the best accuracy
  sensor_vl53l4cd_sat.VL53L4CD_SetRangeTiming(200, 0);

  // Start Measurements
  sensor_vl53l4cd_sat.VL53L4CD_StartRanging();
}
void TOF()
{
  uint8_t NewDataReady = 0;
  VL53L4CD_Result_t results;
  uint8_t status;
  char report[64];

  do {
    status = sensor_vl53l4cd_sat.VL53L4CD_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  //Led on
  digitalWrite(LedPin, HIGH);

  if ((!status) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    sensor_vl53l4cd_sat.VL53L4CD_ClearInterrupt();

    // Read measured distance. RangeStatus = 0 means valid data
    sensor_vl53l4cd_sat.VL53L4CD_GetResult(&results);
    //snprintf(report, sizeof(report), "Status = %3u, Distance = %5u mm, Signal = %6u kcps/spad\r\n",
             //results.range_status,
             //results.distance_mm,
             //results.signal_per_spad_kcps);
    //Serial.print("Distance(mm) from TOF: ");         
    SerialPort.print(results.distance_mm);
    Serial.print("\t");
  }

  //Led off
  digitalWrite(LedPin, LOW);
}
void Ultrasonic() {
  delay(200);
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  //Serial.print("Distance(cm) from Ultrasonic: ");
  Serial.println(distance);
  //Serial.println(",");

  //Serial.println(" cm");
}
void loop() {
  // put your main code here, to run repeatedly:
  TOF();
  Ultrasonic();

}
