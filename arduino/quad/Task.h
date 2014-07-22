#ifndef __TASK_H__
#define __TASK_H__

/**
   @brief A base for other Tasks.
*/
class Task
{
public:
  
  /**
     @brief Determines if the Task can run now.
  */
  virtual bool canRun(unsigned long now) = 0;

  /**
     @brief 
  */
  virtual void run(unsigned long now) = 0;
};

#endif
