#define TASK_100HZ 1
#define TASK_50HZ 2
#define TASK_10HZ 10
#define TASK_1HZ 100

#include <TFT.h>
#include <SPI.h>
#include "TaskScheduler.h"

#define cs   10
#define dc   9
#define rst  8

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

TFT tft = TFT(cs, dc, rst);

unsigned long lightSensorValue;
char lightSensorPrintout[4];

void setup()
{
  Serial.begin(9600);
  
  tft.begin();

  displayInfoText();

  delay(2000);
  tft.background(0, 0, 0);
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

  updateLightSensor(fiftyHZpreviousTime);
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

void updateLightSensor(unsigned long dt)
{
  lightSensorValue = analogRead(A0);
  String sensorValue = String(lightSensorValue);
  sensorValue.toCharArray(lightSensorPrintout, 4);
}

void updateDisplay(unsigned long dt)
{
  displaySensorInfo();
}

void displaySensorInfo()
{
  tft.background(0, 0, 0);
  tft.stroke(255, 255, 255);
  tft.setTextSize(1);
  tft.text("Light: ", 0, 0);
  tft.stroke(0, 230, 30);
  tft.text(lightSensorPrintout, 40, 0); 
}

void displayInfoText()
{
  tft.background(0, 0, 0);

  tft.stroke(255, 255, 255);
  tft.setTextSize(1);
  tft.text("Quadcopter software", tft.width()/2 - 60, tft.height()/2 - 30);
  tft.text("version 0.1", tft.width()/2 - 30, tft.height()/2);
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
