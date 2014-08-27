#define TASK_100HZ 1
#define TASK_50HZ 2
#define TASK_10HZ 10
#define TASK_1HZ 100

#include "TaskScheduler.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"
#include "BMP085.h"
#include "Wire.h"

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

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup()
{
  Serial.begin(9600);
  
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing MPU6050 connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
    {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      Serial.println(F("Enabling Magnetometer..."));
      mpu.setI2CBypassEnabled(true);
      mag.initialize();

      Serial.println("Testing HMC5883L connections...");
      Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
  else
    {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
  
  Serial.println(F("Initializing Barometer..."));
  barometer.initialize();
  Serial.println("Testing BMP085 connections...");
  Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");
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

  updateGyroSensor(deltaTime);
  updateAccelSensor(deltaTime);
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

void updateGyroSensor(unsigned long dt)
{
  if(dmpReady)
    {
      if(mpuInterrupt || fifoCount > packetSize)
        {
          mpuInterrupt = false;
          mpuIntStatus = mpu.getIntStatus();
          
          // get current FIFO count
          fifoCount = mpu.getFIFOCount();

          if ((mpuIntStatus & 0x10) || fifoCount == 1024)
            {
              // reset so we can continue cleanly
              mpu.resetFIFO();
              //Serial.println(F("FIFO overflow!"));
              
              // otherwise, check for DMP data ready interrupt (this should happen frequently)
            }
          else if (mpuIntStatus & 0x02)
            {
              // wait for correct available data length, should be a VERY short wait
              while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
              
              // read a packet from FIFO
              mpu.getFIFOBytes(fifoBuffer, packetSize);
        
              // track FIFO count here in case there is > 1 packet available
              // (this lets us immediately read more without waiting for an interrupt)
              fifoCount -= packetSize;

              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetEuler(euler, &q);
              mpu.dmpGetGravity(&gravity, &q);
              mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
              
              // display real acceleration, adjusted to remove gravity
              mpu.dmpGetAccel(&aa, fifoBuffer);
              mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
              
              // display initial world-frame acceleration, adjusted to remove gravity
              // and rotated based on known orientation from quaternion
              mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            }
        }
    }
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

void updateAccelSensor(unsigned long dt)
{
}

void displayAccelValues()
{
}

void updateMagSensor(unsigned long dt)
{
  mag.getHeading(&mx, &my, &mz);
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

void updateBarSensor(unsigned long dt)
{
  barometer.setControl(BMP085_MODE_TEMPERATURE);
  temperature = barometer.getTemperatureC();

  barometer.setControl(BMP085_MODE_PRESSURE_3);
  if(micros() - lastBarReadingMicros > barometer.getMeasureDelayMicroseconds())
    {
      lastBarReadingMicros = micros();
      pressure = barometer.getPressure();
      altitude = barometer.getAltitude(pressure);
    }
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
  displaySensorInfo();
}

void displaySensorInfo()
{
  displayGyroValues();
  displayAccelValues();
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
