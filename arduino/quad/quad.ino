#define TASK_100HZ 1
#define TASK_50HZ 2
#define TASK_10HZ 10
#define TASK_1HZ 100

#include "TaskScheduler.h"

unsigned long frameCounter = 0; // main loop executive frame counter

// main loop time variable
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;

// sub loop time variable
unsigned long oneHZpreviousTime = 0;
unsigned long tenHZpreviousTime = 0;
unsigned long lowPriorityTenHZpreviousTime = 0;
unsigned long lowPriorityTenHZpreviousTime2 = 0;
unsigned long fiftyHZpreviousTime = 0;
unsigned long hundredHZpreviousTime = 0;

void setup()
{
  Serial.begin(9600);
}

void process100HzTask()
{
  
}

void process50HzTask()
{
  
}

void process10HzTask1()
{
  
}

void process10HzTask2()
{
  
}

void process10HzTask3()
{
  
}

void process1HzTask()
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
          process10HzTask1();
        }
      else if ((currentTime - lowPriorityTenHZpreviousTime) > 100000)
        {
          process10HzTask2();
        }
      else if ((currentTime - lowPriorityTenHZpreviousTime2) > 100000)
        {
          process10HzTask3();
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