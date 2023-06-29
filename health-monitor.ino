/**
 * BAN Monitor Arduino Microcontroller Code
 * 
 * @author: Eric Huang
 * @since: JAN 29 2021
 * 
 * Description: This code gathers data from pulse oximeter, accelerometer, and temperature sensor. 
 * Data is then processed and pushed to Serial Monitor + Bluetooth. Code also incorporated Fall Detection algorithm.
 * 
 */

// Libraries
#include "Wire.h"
#include <SoftwareSerial.h>
#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <SparkFunMLX90614.h>

// Accelerometer and Fall Detection Parameters
const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;

boolean fall = false; //stores if a fall has occurred
boolean trigger1=false; //stores if first trigger (lower threshold) has occurred
boolean trigger2=false; //stores if second trigger (upper threshold) has occurred
boolean trigger3=false; //stores if third trigger (orientation change) has occurred

byte trigger1count=0; //stores the counts past since trigger 1 was set true
byte trigger2count=0; //stores the counts past since trigger 2 was set true
byte trigger3count=0; //stores the counts past since trigger 3 was set true
int angleChange=0;

// Bluetooth Module
SoftwareSerial MyBlue(6, 7); // RX | TX

// Temperature Sensor
//int tempPin = 0;
//float T;

// Pulse Oximeter
#define DEF_ADDR 0x55
int resPin = 4;
int mfioPin = 5;

SparkFun_Bio_Sensor_Hub bioHub(resPin, mfioPin); 

// IR Temperature Sensor
IRTherm therm;

bioData body;

void setup() {

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  MyBlue.begin(9600);

  Serial.println("Body Area Network system starting...");
  
  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH);

  therm.begin();
  therm.setUnit(TEMP_C);
  pinMode(LED_BUILTIN, OUTPUT);

  int result = bioHub.begin();
  int error = bioHub.configBpm(MODE_ONE);
  delay(4000); 
}

void loop() {
  
//  temp_read();
  digitalWrite(LED_BUILTIN, HIGH);
  therm.read();
  body = bioHub.readBpm();
  mpu_read();
  
  // 2050, 77, 1947 are values for calibration of accelerometer
  ax = (AcX-2050)/16384.00;
  ay = (AcY-77)/16384.00;
  az = (AcZ-1947)/16384.00;
  
  // 270, 351, 136 for gyroscope
  gx = (GyX+270)/131.07;
  gy = (GyY-351)/131.07;
  gz = (GyZ+136)/131.07;
  
  // calculating Amplitute vector for 3 axis
  float Raw_AM = pow(pow(ax,2)+pow(ay,2)+pow(az,2),0.5);
  int AM = Raw_AM * 10;  

  // Print data to Serial Monitor and Bluetooth
  Serial.print(body.heartRate);
  Serial.print("|");
  
  MyBlue.print(body.heartRate);
  MyBlue.print("|");
  
  Serial.print(String(therm.object(), 2));
  Serial.print("|");
  
  MyBlue.print(String(therm.object(), 2));
  MyBlue.print("|");
  
  Serial.print(body.oxygen);
  Serial.print("|");
  
  MyBlue.print(body.oxygen);
  MyBlue.print("|");

/*
 * Fall Detection Algorithm
 */
 
  if (AM<=2 && trigger2==false) { //if AM breaks lower threshold (0.4g)
     trigger1=true;
     //Serial.println("TRIGGER 1 ACTIVATED");
     }
     
  if (trigger1==true){
     trigger1count++;
     if (AM>=3){ //if AM breaks upper threshold (3g)
       trigger2=true;
       //Serial.println("TRIGGER 2 ACTIVATED");
       trigger1=false; trigger1count=0;
       }
     }
     
  if (trigger1count>=2){ //allow 0.5s for AM to break upper threshold
     trigger1=false; trigger1count=0;
     //Serial.println("TRIGGER 1 DECACTIVATED");
     }
  
  if (trigger2==true){
   trigger2count++;
   //angleChange=acos(((double)x*(double)bx+(double)y*(double)by+(double)z*(double)bz)/(double)AM/(double)BM);
   angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5); 
   //Serial.println(angleChange);
   if (angleChange>=30 && angleChange<=400){ //if orientation changes by between 80-100 degrees
     trigger3=true; trigger2=false; trigger2count=0;
     //Serial.println(angleChange);
     //Serial.println("TRIGGER 3 ACTIVATED");
     }
   }

  if (trigger2count>=2){ //allow 0.5s for orientation change
   trigger2=false; trigger2count=0;
   //Serial.println("TRIGGER 2 DECACTIVATED");
   }
   
  if (trigger3==true){
    trigger3count++;
    //Serial.println(trigger3count);
    if (trigger3count>=10){ 
       angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5);
       //delay(10);
       //Serial.println(angleChange); 
       if ((angleChange>=0) && (angleChange<=10)){ //if orientation changes remains between 0-10 degrees
           fall=true; trigger3=false; trigger3count=0;
           //Serial.println(angleChange);
             }
       else{ //user regained normal orientation
          trigger3=false; trigger3count=0;
          //Serial.println("TRIGGER 3 DEACTIVATED");
       }
    }
  }
  
  if (fall==true){ //in event of a fall detection
    
    fall_detected();
    digitalWrite(11, LOW);
    delay(20);
    digitalWrite(11, HIGH);
    fall=false;
   }
  
  if (fall==false) {
    
    Serial.println("0");
    MyBlue.println("0");
    Serial.println();
    MyBlue.println();
    delay(20);
  }
  delay(1000); // delay to not clog the port
}

/*
 * Function: Get accelerometer readings
 */
 
void mpu_read() {
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

/*
 * Function: Get temperature reading
 */
//void temp_read() {
//  
//  // Get raw voltage reading from temperature sensor
//  int reading = analogRead(tempPin);  
//  
//  // Convert reading to voltage
//  float voltage = reading * (5000 / 1024); 
//    
//  T = voltage/10;
//}

/*
 * Interupt Function: Perform when fall is detected
 */
 
void fall_detected(){

   Serial.println("0");
   MyBlue.println("0");

   Serial.println();
   MyBlue.println();
   
   for (int x = 0; x < 3; x++) {
    
     Serial.print(body.heartRate);
     Serial.print("|");
     
     MyBlue.print(body.heartRate);
     MyBlue.print("|");
     
     Serial.print(String(therm.object(), 2));
     Serial.print("|");
     
     MyBlue.print(String(therm.object(), 2));
     MyBlue.print("|");
     
     Serial.print(body.oxygen);
     Serial.print("|");
     
     MyBlue.print(body.oxygen);
     MyBlue.print("|");
     
     Serial.println("1");
     MyBlue.println("1");

     Serial.println();
     MyBlue.println();
     
     delay(120);
  }
  
  Serial.print(body.heartRate);
  Serial.print("|");
  
  MyBlue.print(body.heartRate);
  MyBlue.print("|");
  
  Serial.print(String(therm.object(), 2));
  Serial.print("|");
  
  MyBlue.print(String(therm.object(), 2));
  MyBlue.print("|");
  
  Serial.print(body.oxygen);
  Serial.print("|");
  
  MyBlue.print(body.oxygen);
  MyBlue.print("|"); 
  
}
