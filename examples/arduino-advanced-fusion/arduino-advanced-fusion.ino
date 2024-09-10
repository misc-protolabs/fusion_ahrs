#include <TaskScheduler.h>
#include "vfb.h"
#include "app.h"

// scheduler
Scheduler sched;

// app tasks
Task vfb_app_100Hz ( 10 * TASK_MILLISECOND, -1, &vfb_step_100Hz, &sched, true);
Task tsk_app_100Hz ( 10 * TASK_MILLISECOND, -1, &app_step_100Hz, &sched, true );

void setup( void)
{
  vfb_init();
  app_init();
}

void loop( void)
{
  sched.execute();
}
