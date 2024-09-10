#ifndef _APP_H_
#define _APP_H_

extern bool app_init( void);
extern void app_step_100Hz( void);
extern void app_step_1Hz( void);

extern float gx, gy, gz;
extern float ax, ay, az;
extern float mx, my, mz;

#endif
