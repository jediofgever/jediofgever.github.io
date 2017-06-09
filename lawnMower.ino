//Mechatronics System Design'17
//Otonom Çim Biçme Robotu

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <PID_v1.h>



//MOTOR PIN BAGLANTILARI
int ML_FORWARD  = 2;                    // PWM Pinlerine baglanacak.
int ML_BACKWARD = 3;
int MR_FORWARD  = 4;
int MR_BACKWARD = 5;
int MCUT_FORWARD  = 6;
int MCUT_BACKWARD = 7;

int SPEED=40, T_SPEED=40;

int flag;

//////IMU
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 18                   //IMU'nun INT çikisi 8. Pin'e baglanacak.
uint8_t mpuIntStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
float r;

volatile bool mpuInterrupt = false;     
void dmpDataReady() {
  mpuInterrupt = true;
}

//////// GPS
SoftwareSerial mySerial(10, 9);                    //GPS'in RX ---> 9, TX---> 10' baglanacak. 
TinyGPS gps;

float REFERENCE_ANGLE, CURRENT_ANGLE;                       //Raspberry Pi'dan gelecek Referans Açi
////////// PID ////////////
double Setpoint, Input, Output;
double Kp=2, Ki=0.01, Kd=0.2;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID sPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);


//################################## SETUP #############################################

void setup() {

  Serial.begin(115200);

  pinMode(ML_FORWARD,    OUTPUT);
  pinMode(ML_BACKWARD,   OUTPUT);
  pinMode(MR_FORWARD,    OUTPUT);
  pinMode(MR_BACKWARD,   OUTPUT);
  pinMode(MCUT_FORWARD,  OUTPUT);
  pinMode(MCUT_BACKWARD, OUTPUT);

  analogWrite(ML_FORWARD,  0);
  analogWrite(ML_BACKWARD, 0);
  analogWrite(MR_FORWARD,  0);
  analogWrite(MR_BACKWARD, 0);

  myPID.SetMode(AUTOMATIC);
  sPID.SetMode(AUTOMATIC);
//  SetOutputLimits(0, 200)       // Bu durumda PWM deðeri 50 ile 250 arasýnda deðiþiyor. 

  /////////////// IMU Initialization /////////////////////
  Wire.begin();
  Wire.setClock(400000);
  pinMode(INTERRUPT_PIN, INPUT);
 
  mpu.initialize();
  mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  mpu.setDMPEnabled(true);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();  
  packetSize = mpu.dmpGetFIFOPacketSize();

  ///////////////// GPS Initialization /////////////////////
  mySerial.begin(9600);
  delay(1000);
}

//########################### Compass Function #############################################

float Get_Compass(){
  
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        } 
   else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        delay(10);
//        Serial.print("ypr\t");
//        Serial.print(ypr[0] * 180/M_PI);
//        Serial.print("\t");
//        Serial.print(ypr[1] * 180/M_PI);
//        Serial.print("\t");
//        Serial.println(ypr[2] * 180/M_PI);
  }
  
return ypr[0]*180/M_PI;
}

//########################### GPS Function #############################################
  
void Get_GPS(){
  float flat, flon;
  while (mySerial.available())
  gps.encode(mySerial.read());
  gps.f_get_position(&flat, &flon);
  
  Serial.print(flat, 7);Serial.print(flon, 7);
  delay(1);
}

//############################### MAIN LOOP #############################################


void loop() {

//  float REFERENCE_ANGLE=130;

float REFERENCE_ANGLE = Serial.parseFloat();

//if(Serial.available()>0) 
//   {
//    float REFERENCE_ANGLE = Serial.parseFloat();
//   }

  Serial.print(REFERENCE_ANGLE);
  Get_GPS();
  float CURRENT_ANGLE = Get_Compass();
  Serial.println(CURRENT_ANGLE);
//  Serial.print("\t");
//  if(CURRENT_ANGLE<0) CURRENT_ANGLE=CURRENT_ANGLE + 360;
//  Serial.println("REF = \t");
//  Serial.println(REFERENCE_ANGLE);
  delay(1);
  
  
  float DIFF = REFERENCE_ANGLE - CURRENT_ANGLE;

  // Dönüþ yönünün belirlenmesi (Burada hata olabilir)
  if(CURRENT_ANGLE >= 90 && CURRENT_ANGLE <= 180)
  {
    if(REFERENCE_ANGLE < 0)
    {
      flag = 1;  //Turn Right
    }
    else if(REFERENCE_ANGLE > 0 && REFERENCE_ANGLE < CURRENT_ANGLE)
    {
      flag = 2;  // Turn Left
    }
    else if(REFERENCE_ANGLE > 0 && REFERENCE_ANGLE > CURRENT_ANGLE)
    {
      flag = 1; //Turn Right
    }
  }

  if(CURRENT_ANGLE <= -90 && CURRENT_ANGLE >= -180)
  {
    if(REFERENCE_ANGLE > 0)
    {
      flag = 2;  //Turn Left
    }
    else if(REFERENCE_ANGLE < 0 && REFERENCE_ANGLE < CURRENT_ANGLE)
    {
      flag = 1;  // Turn Right
    }
    else if(REFERENCE_ANGLE < 0 && REFERENCE_ANGLE > CURRENT_ANGLE)
    {
      flag = 2; //Turn Left
    }
  }
//////////////////////////
  if(CURRENT_ANGLE >= 0 && CURRENT_ANGLE <= 90)
  {
    if(REFERENCE_ANGLE < 0)
    {
      flag = 2;  //Turn Left
    }
    else if(REFERENCE_ANGLE > 0 && REFERENCE_ANGLE < CURRENT_ANGLE)
    {
      flag = 2;  // Turn Left
    }
    else if(REFERENCE_ANGLE > 0 && REFERENCE_ANGLE > CURRENT_ANGLE)
    {
      flag = 1; //Turn Right
    }
  }
  
  if(CURRENT_ANGLE <= 0 && CURRENT_ANGLE >= -90)
  {
    if(REFERENCE_ANGLE > 0)
    {
      flag = 1;  //Turn Right
    }
    else if(REFERENCE_ANGLE < 0 && REFERENCE_ANGLE > CURRENT_ANGLE)
    {
      flag = 1;  // Turn Right
    }
    else if(REFERENCE_ANGLE < 0 && REFERENCE_ANGLE < CURRENT_ANGLE)
    {
      flag = 2; //Turn Left
    }
  }

  if(DIFF>0){
    Input = -DIFF;
    Setpoint = 0; 
  }
  else{
    Input    = DIFF;
    Setpoint = 0;
  }

  myPID.Compute();
//  Serial.print(Input);
//  Serial.print("\t");
//  Serial.println(Output);

  // Dönüþ Komutlarý
  if(flag==1 && abs(DIFF) > 10){
    analogWrite(ML_FORWARD, T_SPEED+Output);
    analogWrite(MR_FORWARD, T_SPEED);
//    Serial.println("R");
    }
  if(flag==2 && abs(DIFF) > 10){
    analogWrite(ML_FORWARD, T_SPEED);
    analogWrite(MR_FORWARD, T_SPEED+Output);
//    Serial.println("L"); 
  }
  if(abs(DIFF)<=10){
    analogWrite(ML_FORWARD, SPEED);
    analogWrite(MR_FORWARD, SPEED);
//    Serial.println("D");  
  }
}