#define TASK_100HZ 1
#define TASK_50HZ 2
#define TASK_10HZ 10
#define TASK_1HZ 100

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"
#include "BMP085.h"
#include "Wire.h"
#include "config.h"

unsigned long frameCounter = 0; // main loop executive frame counter

// main loop time variable
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;

// sub loop time variable
unsigned long oneHZpreviousTime = 0;
unsigned long tenHZpreviousTime = 0;
unsigned long fiftyHZpreviousTime = 0;
unsigned long hundredHZpreviousTime = 0;

MPU6050 mpu;

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
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

HMC5883L mag;

// magnetometer reading values
int16_t mx, my, mz;

BMP085 barometer;

float temperature;
float pressure;
float altitude;
int32_t lastBarReadingMicros;

void setup()
{
  Serial.begin(9600);
  
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  Serial.println(F("Initializing I2C devices..."));
  initializeAccelGyro();
  initializeMagnetometer();
  initializeBarometer();
}

void process100HzTask()
{
  hundredHZpreviousTime = currentTime;
  deltaTime = currentTime - hundredHZpreviousTime;
}

void process50HzTask()
{
  fiftyHZpreviousTime = currentTime;
  deltaTime = currentTime - fiftyHZpreviousTime;

  updateAccelGyroSensor(deltaTime);
  updateMagSensor(deltaTime);
  updateBarSensor(deltaTime);
}

void process10HzTask()
{
  tenHZpreviousTime = currentTime;
  deltaTime = currentTime - tenHZpreviousTime;
}

void process1HzTask()
{
  oneHZpreviousTime = currentTime;
  deltaTime = currentTime - oneHZpreviousTime;

  updateDisplay(deltaTime);
}


void displayGyroValues()
{
  Serial.print("quat\t");
  Serial.print(q.w);
  Serial.print("\t");
  Serial.print(q.x);
  Serial.print("\t");
  Serial.print(q.y);
  Serial.print("\t");
  Serial.println(q.z);

  Serial.print("euler\t");
  Serial.print(euler[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(euler[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(euler[2] * 180/M_PI);
  
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180/M_PI);
  
  Serial.print("areal\t");
  Serial.print(aaReal.x);
  Serial.print("\t");
  Serial.print(aaReal.y);
  Serial.print("\t");
  Serial.println(aaReal.z);
            
  Serial.print("aworld\t");
  Serial.print(aaWorld.x);
  Serial.print("\t");
  Serial.print(aaWorld.y);
  Serial.print("\t");
  Serial.println(aaWorld.z);
}

void displayMagValues()
{
  Serial.print("mag:\t");
  Serial.print(mx);
  Serial.print("\t");
  Serial.print(my);
  Serial.print("\t");
  Serial.println(mz);
}

void serialMagValues()
{
  Serial.print("$$");
  Serial.print(mx);
  Serial.print(" ");
  Serial.print(my);
  Serial.print(" ");
  Serial.println(mz);
}

void displayBarValues()
{
  Serial.print("T/P/A\t");
  Serial.print(temperature);
  Serial.print("\t");
  Serial.print(pressure);
  Serial.print("\t");
  Serial.println(altitude);
}

void updateDisplay(unsigned long dt)
{
  //displaySensorInfo();

  serialMagValues();
}

void displaySensorInfo()
{
  displayGyroValues();
  displayMagValues();
  displayBarValues();
}

void displayInfoText()
{
}

void loop()
{
  currentTime = micros();
  deltaTime = currentTime - previousTime;

  if (deltaTime >= 10000)
    {
      frameCounter++;
    
      process100HzTask();

      if (frameCounter % TASK_50HZ == 0) // 50 Hz tasks
        {
          process50HzTask();
        }

      if (frameCounter % TASK_10HZ == 0) // 10 Hz tasks
        {
          process10HzTask();
        }

      if (frameCounter % TASK_1HZ == 0) // 1 Hz tasks
        {
          process1HzTask();
        }
    
      previousTime = currentTime;
    }
  
  if (frameCounter >= 100)
    {
      frameCounter = 0;
    }
}
