#ifndef __TIMED_TASK_H__
#define __TIMED_TASK_H__

#include "Task.h"

/**
   @brief A task that is run on a periodic basis.
*/
class TimedTask : public Task
{
public:
  
  /**
     @brief Create a periodically executed task.
     @param[in] period Determines the frequency that the task will run.
  */
  inline TimedTask(unsigned long period)
  {
    mNextRun = 0;
    mPeriod = period;
  }

  virtual bool canRun(unsigned long now) {return now >= mNextRun;};

  /**
     @brief Set the system clock tick when the task can next run.
     @param[in] nextRun The system clock tick when the task should run, in milliseconds.
  */
  inline void setNextRun(unsigned long nextRun) {mNextRun = nextRun;}

  /**
     @brief Get the system clock tick when the task can next run.
     @return system clock tick when the task is next due to run.
  */
  inline unsigned long getNextRun() {return mNextRun;}

protected:
  
  /**
     @brief The  system clock tick when the task can next run.
   */
  unsigned long mNextRun;
  
  unsigned long mLastRun;
  long mPeriod;
};

#endif
