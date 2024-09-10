#include "Arduino.h"
#include "vfb.h"
#include "imu.h"
#include "mag.h"
#include "app.h"

bool vfb_init( void)
{
  Serial.begin( 115200);
  //Wire.begin();

  Serial.println( "- io-init...");
  pinMode(EN_3V3_SW, OUTPUT);
  digitalWrite(EN_3V3_SW, HIGH);

  pinMode(BOOT_BTN, INPUT);
  pinMode(STAT_LED, OUTPUT);

  pinMode(IMU_CS, OUTPUT);
  digitalWrite(IMU_CS, HIGH);

  pinMode(IMU_INT1, INPUT);

  pinMode(MAG_CS, OUTPUT);
  digitalWrite(MAG_CS, HIGH);

  delay(100);

  Serial.println( "- imu-init...");
  imu_init();
  //Serial.println( "- mag-init...");
  //mag_init();
  delay(100);

  digitalWrite( STAT_LED, LOW);

  return 1;
}

void vfb_step_100Hz( void)
{
  digitalWrite( STAT_LED, HIGH);
  //imu_step();
  //mag_step();
}
