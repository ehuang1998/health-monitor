#include "Wire.h"
#include <SoftwareSerial.h>

/*-----------------
 * Bluetooth Parameters
 * ----------------
 */

 SoftwareSerial MyBlue(2, 3); // RX | TX
 
/*-----------------
 * Gyro/Accelerometer Parameters
 * ----------------
 */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_REALACCEL

//fall detection vars
float ax, ay, az;
float gx, gy, gz;

boolean fall = false; //stores if a fall has occurred
boolean trigger1=false; //stores if first trigger (lower threshold) has occurred
boolean trigger2=false; //stores if second trigger (upper threshold) has occurred
boolean trigger3=false; //stores if third trigger (orientation change) has occurred
byte trigger1count=0; //stores the counts past since trigger 1 was set true
byte trigger2count=0; //stores the counts past since trigger 2 was set true
byte trigger3count=0; //stores the counts past since trigger 3 was set true
int angleChange=0;

#define INTERRUPT_PIN 5  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container


//Interupt Detection
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

/*-----------------
 * Temperature Parameters
 * ----------------
 */
int ThermistorPin = 0;
int Vo;
float R1 = 10000;
float logR2, R2, T;
float c1 = 0.001125308852122, c2 = 0.000234711863267, c3 =  0.000000085663516;


/*-----------------
 * Heart Rate Parameters
 * ----------------
 */

#include <SparkFun_Bio_Sensor_Hub_Library.h>

// Reset pin, MFIO pin
int resPin = 4;
int mfioPin = 5;

// Takes address, reset pin, and MFIO pin.
SparkFun_Bio_Sensor_Hub bioHub(resPin, mfioPin); 

bioData body;

//setup
void setup() {

/*-----------------
 * Set up I2C transmission
 * ----------------
 */

  MyBlue.begin(9600);

  /*-----------------
 * Set up bluetooth transmission
 * ----------------
 */
  Wire.begin();
  Serial.begin(9600);

/*-----------------
 * Gyro/Accelerometer Setup
 * ----------------
 */

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();

    // Offsets for sensitivity
    mpu.setXGyroOffset(54);
    mpu.setYGyroOffset(-27);
    mpu.setZGyroOffset(14);
    mpu.setXAccelOffset(3580);
    mpu.setYAccelOffset(-6046);
    mpu.setZAccelOffset(1571);

    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;

    }

    pinMode(LED_PIN, OUTPUT);
    delay(500);


/*-----------------
 * Heart Rate Setup
 * ----------------
 */

int result = bioHub.begin();
int error = bioHub.configBpm(MODE_ONE);
delay(4000); 
 
}

void loop() {

 /*-----------------
 * Gyro/Accelerometer Reading
 * ----------------
 */

 if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

      #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            ax = aaReal.x;
            ay = aaReal.y;
            az = aaReal.z;
        #endif
        
        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            gx = euler[0] * 180/M_PI;
            gy = euler[1] * 180/M_PI;
            gz = euler[2] * 180/M_PI;
        #endif


//Calculate amplitude of acceleration
float Accel_Amp = pow(pow(ax,2)+pow(ay,2)+pow(az,2),0.5);

//Check if amplitude is lower than first threshold
if (Accel_Amp<=2 && trigger2==false)
{
   trigger1=true;
}

//If first threshold was met
if (trigger1==true)
{
   trigger1count++;
   //Check if upper threshold broken
   if (Accel_Amp>=12)
    {        
     trigger2=true; //set to true
     trigger1=false; trigger1count=0; //reset trigger 1
    }
}

//Check orientation magnitude if first two triggers met
if (trigger2==true)
{
  
   trigger2count++;
   angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5); //Calculate change in angle

   //If angle change also breaks threshold update triggers
   if (angleChange>=30 && angleChange<=400)
    {
     trigger3=true; trigger2=false; trigger2count=0;
    }
}

//If trigger 3 was met decide if fall detected
if (trigger3==true)
{
    trigger3count++;
    if (trigger3count>=10)
    { 
       angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5);

       //Check Orientation again
       if ((angleChange>=0) && (angleChange<=10))
       {           
           fall=true; trigger3=false; trigger3count=0; // Reset Counter and set fall to true if met
       }
       else
       {
          trigger3=false; trigger3count=0; //If orientation/balance regained reset the counters
       }
    }
}

//Reset trigger 1 if 0.5 seconds pass without fall
if (trigger2count>=6)
{
   trigger2=false; trigger2count=0;
}

//Reset trigger 2 if 0.5 seconds pass without fall
if (trigger1count>=6)
{
   trigger1=false; trigger1count=0;
}

}

/*-----------------
 * Temperature Reading
 * ----------------
 */
 
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;
  //T = (T * 9.0)/ 5.0 + 32.0; // If farenheit wanted
  
/*-----------------
 * Heart Rate Reading
 * ----------------
 */

body = bioHub.readBpm();

/*-----------------
 * Output Data
 * ----------------
 */

//heart rate

    Serial.print(body.heartRate); 
    Serial.print(" BPM");
    Serial.print("|");
    
    MyBlue.print(body.heartRate);
    MyBlue.print(" BPM"); 
    MyBlue.print("|");

//Temperature

  Serial.print(T);
  Serial.print(" °C");
  Serial.print("|");

  MyBlue.print(T);
  MyBlue.print(" °C");
  MyBlue.print("|");

//blood oxygen level

    Serial.print(body.oxygen); 
    Serial.print(" %");
    Serial.print("|");
 
    MyBlue.print(body.oxygen); 
    MyBlue.print(" %");
    MyBlue.print("|");

//Fall Detection

     if (fall==true)
     {               
        Serial.println("Fall Detected"); 
 
        MyBlue.println("Fall Detected"); 
 
        fall=false;
     }
     else
     {

        Serial.println("Normal"); 
 
        MyBlue.println("Normal"); 
        
     }

    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);


//confidence
    //MyBlue.print(body.confidence); 
    //MyBlue.print("|");
    
//status of reading
    //MyBlue.print(body.status); 
    //MyBlue.print("|");

delay(1500); 

}
