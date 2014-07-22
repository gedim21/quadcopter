#ifndef __TASK_SCHEDULER_H__
#define __TASK_SCHEDULER_H__

#include <Arduino.h>
#include "Task.h"

class TaskScheduler
{
public:
  
  /**
     @brief Create a new task scheduler.  
     Tasks are scheduled in priority order, where the highest priority task is first in the array, 
     and the lowest priority task is the last.
     @param[in] tasks array of task pointers.
     @param[in] numTasks number of tasks in the array.
  */
  TaskScheduler(Task** tasks, unsigned long numTasks) : mTasks(tasks),
                                                        mNumTasks(numTasks)
  {
  }

  /**
     @brief Start the task scheduler running.
  */
  void run()
  {
    while(true)
      {
        unsigned long now = millis();
        Task** tasks = mTasks;
        for (int t = 0; t < mNumTasks; t++)
          {
            Task* task = *tasks;
            if (task->canRun(now))
              {
                task->run(now);
                break;
              }
            tasks++;
          }
      }
  }

private:
  /**
     @brief The Tasks that are managed by the TaskManager.
  */
  Task** mTasks;
  
  int mNumTasks;
};

#endif
