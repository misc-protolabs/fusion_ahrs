#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"

#include "vfb.h"
#include "imu.h"
#include "mag.h"
#include "sd_log.h"
#include "lipo.h"
#include "app.h"

bool boot_btn;
bool stat_led;

bool vfb_init( void)
{
  Serial.begin( 115200);

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

  Serial.println( "- spi-init...");
  SPI.begin( 18, 19, 23, 5);

  Serial.println( "- i2c-init...");
  Wire.begin();
  Wire.setClock( 400000); // [10000,100000,400000,1000000,3400000]

  Serial.println( "- imu-init...");
  imu_init();
  //Serial.println( "- mag-init...");
  //mag_init();
  Serial.println( "- lipo-init...");
  lipo_init();
  Serial.println( "- sd-init...");
  sd_log_init();
  delay(100);

  digitalWrite( STAT_LED, LOW);

  return 1;
}

void vfb_step_100Hz( void)
{
  boot_btn = !digitalRead( BOOT_BTN);
  digitalWrite( STAT_LED, stat_led);
  //imu_step();
  //mag_step();
}

