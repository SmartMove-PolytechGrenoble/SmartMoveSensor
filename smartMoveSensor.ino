/*
   Copyright (c) 2015 Intel Corporation.  All rights reserved.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
   This sketch example partially implements the standard Bluetooth Low-Energy Heart Rate service.
   For more information: https://developer.bluetooth.org/gatt/services/Pages/ServicesHome.aspx
*/

#include <CurieBLE.h>
#include "CurieIMU.h"

#define CHECK_RATE 1000
#define ACCE_RANGE 16
#define GYRO_RANGE 3200
int calibrateOffsets = 1; // int to determine whether calibration takes place or not


BLEPeripheral blePeripheral;       // BLE Peripheral Device (the board you're programming)
// BLEService heartRateService("180D"); // BLE Heart Rate Service
BLEService smartMoveService("c441eb6c-5753-4835-910b-5031c3b78e4c");

//Characteristics or the service
BLECharacteristic AX("A1da3b5e-96d3-4d62-82fb-e4881e3322f8", BLERead | BLENotify, 3); //Accelerometer X-axis
BLECharacteristic AY("A2da3b5e-96d3-4d62-82fb-e4881e3322f8", BLERead | BLENotify, 3); //Accelerometer Y-axis
BLECharacteristic AZ("A3da3b5e-96d3-4d62-82fb-e4881e3322f8", BLERead | BLENotify, 3); //Accelerometer Z-axis
BLECharacteristic GX("B1da3b5e-96d3-4d62-82fb-e4881e3322f8", BLERead | BLENotify, 3); //Gyroscope X-axis
BLECharacteristic GY("B2da3b5e-96d3-4d62-82fb-e4881e3322f8", BLERead | BLENotify, 3); //Gyroscope Y-axis
BLECharacteristic GZ("B3da3b5e-96d3-4d62-82fb-e4881e3322f8", BLERead | BLENotify, 3); //Gyroscope Z-axis

long previousMillis = 0;  // last time the heart rate was checked, in ms

void setup() {
  Serial.begin(9600);    // initialize serial communication
  pinMode(13, OUTPUT);   // initialize the LED on pin 13 to indicate when a central is connected

  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet */
  blePeripheral.setLocalName("SmartMoveSensor");
  blePeripheral.setAdvertisedServiceUuid(smartMoveService.uuid());  // add the service UUID
  blePeripheral.addAttribute(smartMoveService);   // Add the BLE smartmove service

  blePeripheral.addAttribute(AX);
  blePeripheral.addAttribute(AY);
  blePeripheral.addAttribute(AZ);
  blePeripheral.addAttribute(GX);
  blePeripheral.addAttribute(GY);
  blePeripheral.addAttribute(GZ);

  /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
  blePeripheral.begin();
  Serial.println("Bluetooth device active, waiting for connections...");

  // initialize device
  Serial.println("Initializing IMU device...");
  CurieIMU.begin();

  if (calibrateOffsets == 1) {
    // The board must be resting in a horizontal position for
    // the following calibration procedure to work correctly!
    CurieIMU.autoCalibrateGyroOffset();
  
    Serial.print("Starting Acceleration calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1); // 1 for gravity (target value)
  }
  
  CurieIMU.setAccelerometerRange(ACCE_RANGE); //16g max, readings are in mg
  CurieIMU.setGyroRange(GYRO_RANGE); //3200 °/s max
}

void loop() {
  // listen for BLE peripherals to connect:
  BLECentral central = blePeripheral.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(13, HIGH);
    
    // check the heart rate measurement every 200ms
    // as long as the central is still connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 200ms have passed, check the heart rate measurement:
      if (currentMillis - previousMillis >= CHECK_RATE) {
        previousMillis = currentMillis;
        updateMotion();
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(13, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

  int bidon = 0;

void updateMotion() {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the heart rate's measurement.
  */
  int ax,ay,az;
  int gx,gy,gz;
  
  CurieIMU.readMotionSensor(ax,ay,az,gx,gy,gz);
  //int heartRateMeasurement = fake_values();
  //int heartRate = map(heartRateMeasurement, 0, 1023, 0, 100);

  if(true){
    // first char (0) is for BLE flags, then first and second part of the value.
    const unsigned char AXCharArray[3]={0,(char)(ax>>8),(char)(ax)};  
    const unsigned char AYCharArray[3]={0,(char)(ay>>8),(char)(ay)};
    const unsigned char AZCharArray[3]={0,(char)(az>>8),(char)(az)};
    const unsigned char GXCharArray[3]={0,(char)(gx>>8),(char)(gx)};
    const unsigned char GYCharArray[3]={0,(char)(gy>>8),(char)(gy)};
    const unsigned char GZCharArray[3]={0,(char)(gz>>8),(char)(gz)};

    //change value sent with bluetooth, take the "const unsigned char" above as parameter.
    AX.setValue(AXCharArray,3);
    AY.setValue(AYCharArray,3);
    AZ.setValue(AZCharArray,3);
    GX.setValue(GXCharArray,3);
    GY.setValue(GYCharArray,3);
    GZ.setValue(GZCharArray,3);
  }
}

