#include <TaskScheduler.h>
#include "vfb.h"
#include "app.h"

// scheduler
Scheduler sched;

// app tasks
Task vfb_100Hz ( 10 * TASK_MILLISECOND, -1, &vfb_step_100Hz, &sched, true);
Task app_100Hz ( 10 * TASK_MILLISECOND, -1, &app_step_100Hz, &sched, true );
Task app_10Hz ( 100 * TASK_MILLISECOND, -1, &app_step_10Hz, &sched, true );
Task app_1Hz ( 1000 * TASK_MILLISECOND, -1, &app_step_1Hz, &sched, true );

void setup( void)
{
  vfb_init();
  app_init();
}

void loop( void)
{
  sched.execute();
}
