#ifndef __TRIGGERED_TASK_H__
#define __TRIGGERED_TASK_H__

#include "Task.h"

/**
   @brief A task that is triggered by an external event.
*/
class TriggeredTask : public Task
{

public:

  virtual bool canRun(unsigned long now) {return enabled;};

  /**
     @brief Enables the Task so that it will run again.
  */
  inline void enable() {enabled = true;}

  /**
     @brief Disables the Task so that it won't run again.
  */
  inline void disable() {enabled = false;}

protected:
  bool enabled;   // True if the task is currently runnable.
};

#endif
