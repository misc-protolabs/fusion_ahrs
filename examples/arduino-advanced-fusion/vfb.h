#ifndef _VFB_H_
#define _VFB_H_

extern bool vfb_init( void);
extern unsigned int vfb_new_log( void);
extern void vfb_step_100Hz( void);
extern void vfb_deep_sleep( void);

// Tested with Espressif ESP32 v2.0.5 and the "ESP32 Dev Module" board definition
#define EN_3V3_SW 32 // The 3.3V_SW regulator Enable pin is connected to D32
#define BOOT_BTN 0 // DataLogger IoT – 9DoF Boot Button is connected to D0
#define STAT_LED 25 // DataLogger IoT – 9DoF STAT LED is connected to D25
#define RGB_LED 26 // DataLogger IoT – 9DoF RGB LED is connected to D26
#define IMU_CS 5
#define IMU_INT1 34
#define MAG_CS 27
#define MAG_INT 35

extern bool boot_btn;
extern bool stat_led;
#endif
