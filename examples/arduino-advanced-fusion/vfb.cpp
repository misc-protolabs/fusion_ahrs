#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"
#include "driver/rtc_io.h"

#include "vfb.h"
#include "imu.h"
#include "mag.h"
#include "sd_log.h"
#include "lipo.h"
#include "baro.h"
#include "app.h"

void vfb_deep_sleep( void);
void vfb_print_wakeup_reason( void);

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
  Wire.setClock( 1000000); // [10000,100000,400000,1000000,3400000]

  Serial.println( "- imu-init...");
  imu_init();
  Serial.println( "- baro-init...");
  baro_init();
  Serial.println( "- mag-init...");
  mag_init();
  Serial.println( "- lipo-init...");
  lipo_init();
  Serial.println( "- sd-init...");
  sd_log_init();
  delay(100);

  vfb_print_wakeup_reason();
  digitalWrite( STAT_LED, LOW);

  return 1;
}

void vfb_step_100Hz( void)
{
  boot_btn = !digitalRead( BOOT_BTN);
  digitalWrite( STAT_LED, stat_led);

  imu_step( &ax, &ay, &az, &gx, &gy, &gz);
  baro_step( &altitude, &degC);
  mag_step( &mx, &my, &mz);
  sd_log_srvr_step();
}

void vfb_print_wakeup_reason( void) {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:     Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1:     Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER:    Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP:      Serial.println("Wakeup caused by ULP program"); break;
    default:                        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void vfb_deep_sleep( void) {

  // shut everything down
  if( sd_log.logging) {
    sd_log_close();
  }

  // put the esp32 to sleep
  esp_sleep_enable_ext0_wakeup( (gpio_num_t)(BOOT_BTN), 0);  //1 = High, 0 = Low
  // Configure pullup/downs via RTCIO to tie wakeup pins to inactive level during deepsleep.
  // EXT0 resides in the same power domain (RTC_PERIPH) as the RTC IO pullup/downs.
  // No need to keep that power domain explicitly, unlike EXT1.
  rtc_gpio_pullup_dis( (gpio_num_t)(BOOT_BTN));
  rtc_gpio_pulldown_en( (gpio_num_t)(BOOT_BTN));
  Serial.println( "going to sleep...\n");
  esp_deep_sleep_start();
}
