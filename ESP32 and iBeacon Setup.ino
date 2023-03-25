#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

int scanTime = 30; //In seconds

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      //Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
      //BlueCharm_175540
      //N (Constant depends on the Environmental factor. Range 2-4)
      //Distance = 10 ^ ((Measured Power – RSSI)/(10 * N))
      //Measured Power(RSSI) at 1m = -72
      String DeviceName = advertisedDevice.getName().c_str();
      int rssi;
      //N varies from 2-4
      float Env_N= 5;
      int MeasuredPower = -80;
      float calculatedValue=0;
      float Distance = 0;
      if(DeviceName == "BlueCharm_178475"){
        //Serial.println("Found it");
        //Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
        rssi = advertisedDevice.getRSSI();
        calculatedValue = ((MeasuredPower + ((-1)*rssi)) / (10 * Env_N));
        //Serial.print("Rssi: ");
        //Serial.println(rssi);
        //Serial.print("CV: ");
        //Serial.println(calculatedValue);
        //Serial.print("Distance in M: ");
        Distance = pow(10,calculatedValue);
        Serial.print(rssi);
        Serial.print("\t");
        Serial.println(Distance);
        
        advertisedDevice.getScan()->stop();
      }
      
    }
    
}; 

void setup() {
  Serial.begin(115200);
  //Serial.println("Scanning...");

  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  BLEScanResults foundDevices = pBLEScan->start(scanTime);
//  Serial.print("Devices found: ");
//  Serial.println(foundDevices.getCount());
//  Serial.println("Scan done!");
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("Scanning...");

  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  BLEScanResults foundDevices = pBLEScan->start(scanTime);
//  Serial.print("Devices found: ");
//  Serial.println(foundDevices.getCount());
//  Serial.println("Scan done!");
  pBLEScan->clearResults(); 
  delay(1000);
}
