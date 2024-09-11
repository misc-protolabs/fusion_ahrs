#include "Fusion.h"
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include "app.h"
#include "vfb.h"
#include "imu.h"
#include "mag.h"
#include "lipo.h"
#include "sd_log.h"

float gx, gy, gz;
float ax, ay, az;
float mx, my, mz;

#define SAMPLE_RATE (100) // replace this with actual sample rate

// Define calibration (replace with actual calibration data if available)
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {-0.21f, 0.56f, 0.77f};
//const FusionVector gyroscopeOffset = {-0.004887 * (180.0/PI), +0.009774 * (180.0/PI), +0.014661 * (180.0/PI)};
//const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
//const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
//const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix softIronMatrix = {1.0190, 0.0037, 0.0070, 0.0037, 0.9851, 0.0161, 0.0070, 0.0161, 0.9965};
const FusionVector hardIronOffset = {-2.9743, 16.0530, -4.9251};

// Set AHRS algorithm settings
const FusionAhrsSettings settings = {
        .convention = FusionConventionNed,
        .gain = 0.5f,
        .gyroscopeRange = 4000.0f, /* replace this with actual gyroscope range in degrees/s */
        .accelerationRejection = 10.0f,
        .magneticRejection = 10.0f,
        .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
};

FusionOffset offset;
FusionAhrs ahrs;

bool app_init( void)
{
  printf( "- fusion-init...");
  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);

  FusionAhrsSetSettings(&ahrs, &settings);

  ax = 0; ay = 0; az = 0;
  gx = 0; gy = 0; gz = 0;
  mx = 0; my = 0; mz = 0;
  
  return 1;
}

void app_step_100Hz( void)
{
  // Acquire latest sensor data
  const clock_t timestamp = clock(); // replace this with actual gyroscope timestamp

  FusionVector gyroscope = {gx, gy, gz}; // replace this with actual gyroscope data in degrees/s
  FusionVector accelerometer = {ax, ay, az}; // replace this with actual accelerometer data in g
  FusionVector magnetometer = {mx, my, mz}; // replace this with actual magnetometer data in arbitrary units

  // Apply calibration
  gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
  magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

  // Update gyroscope offset correction algorithm
  gyroscope = FusionOffsetUpdate(&offset, gyroscope);

  // Calculate delta time (in seconds) to account for gyroscope sample clock error
  static clock_t previousTimestamp;
  const float deltaTime = (float) (timestamp - previousTimestamp) / (float) CLOCKS_PER_SEC;
  previousTimestamp = timestamp;

  // Update gyroscope AHRS algorithm
  FusionAhrsUpdateNoMagnetometer( &ahrs, gyroscope, accelerometer, deltaTime);
  //FusionAhrsUpdate( &ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

  // Print algorithm outputs
  const FusionEuler euler = FusionQuaternionToEuler( FusionAhrsGetQuaternion(&ahrs));
  const FusionVector earth = FusionAhrsGetEarthAcceleration( &ahrs);
  const FusionVector lin_acc = FusionAhrsGetLinearAcceleration( &ahrs);
  
  float pitch = euler.angle.pitch;
  float roll = euler.angle.roll;
  float yaw = euler.angle.yaw;

  float fe_ax = earth.axis.x;
  float fe_ay = earth.axis.y;
  float fe_az = earth.axis.z;

  float fl_ax = lin_acc.axis.x;
  float fl_ay = lin_acc.axis.y;
  float fl_az = lin_acc.axis.z;

  float v_batt = lipo_v();

  printf( "%5.3f (s), %5.1f (v) : ", deltaTime, v_batt);
  printf( " [% 5.2f, % 5.2f, % 5.2f] (g)", ax, ay, az); // g @ rest) - z==+1.0 is flat and upside down z==-1.0 is flat and right side up (i.e., in-flight for most throws)
  printf( " [% 7.2f, % 7.2f, % 7.2f] (deg/sec)", gx, gy, gz); // deg/sec @ rest - noise ~0.002 rad/sec
  //printf( " [% 7.2f, % 7.2f, % 7.2f] (uT)", mx, my, mz); // @ rest ~15-50uT
  printf( " [% 7.1f, % 7.1f, % 7.1f] (deg)", pitch, roll, yaw); // deg
  printf( " [% 7.2f, % 7.2f, % 7.2f] (g)", fe_ax, fe_ay, fe_az); // @ rest ~0
  //printf( " [% 7.2f, % 7.2f, % 7.2f] (g)", fl_ax, fl_ay, fl_az); // @ rest ~0
  printf( "\n");

  if( sd_log.logging) {
    sd_log.len = sprintf( (char*)(&sd_log.buf[0]), "%lu,%f,%f, %f,%f,%f, %f,%f,%f, %f,%f,%f, %f,%f,%f, %f,%f,%f, %f,%f,%f\n",
      sd_log.log_idx++, deltaTime, v_batt,
      ax, ay, az,
      gx, gy, gz,
      mx, my, mz,
      pitch, roll, yaw,
      fe_ax, fe_ay, fe_az,
      fl_ax, fl_ay, fl_az);
    sd_log_write();
  }
}

void app_step_10Hz( void)
{

  static unsigned char boot_btn_dly = 2;

  if( sd_log.logging) {
    stat_led = !stat_led;
  }
}

void app_step_1Hz( void)
{

  static unsigned char boot_btn_dly = 2;

  if( sd_log.logging) {
    stat_led = !stat_led;
  }
  if( boot_btn) {
    boot_btn_dly--;
    if( boot_btn_dly == 0) {
      boot_btn_dly = 3;
      if( sd_log.logging) {
        sd_log_close();
      } else {
        sd_log_new();
      }
    }
  }
}
