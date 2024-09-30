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
#include "util.h"

float gx, gy, gz;
float ax, ay, az;
float mx, my, mz;
float pitch, roll, yaw;
float fe_ax, fe_ay, fe_az;
float fl_ax, fl_ay, fl_az;
float altitude, degC;
float altitude_filt, degC_filt;
float acc_filt, gz_filt;

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
  altitude = 0;
  degC = 0;
  acc_filt = 0.0;
  gz_filt = 0.0;
  
  return 1;
}

void app_step_100Hz( void)
{
  static float altitude_filt_z1;
  static float degC_filt_z1;
  // Acquire latest sensor data
  const clock_t timestamp = clock(); // replace this with actual gyroscope timestamp

  FusionVector gyroscope = {gx, gy, gz}; // replace this with actual gyroscope data in degrees/s - NED?
  FusionVector accelerometer = {ax, ay, az}; // replace this with actual accelerometer data in g - NED?
  FusionVector magnetometer = {mx, my, mz}; // replace this with actual magnetometer data in arbitrary units - NED?

  // Apply calibration
  gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
  if( mag_present) {
    magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);
  }

  // Update gyroscope offset correction algorithm
  gyroscope = FusionOffsetUpdate(&offset, gyroscope);

  // Calculate delta time (in seconds) to account for gyroscope sample clock error
  static clock_t previousTimestamp;
  const float deltaTime = (float) (timestamp - previousTimestamp) / (float) CLOCKS_PER_SEC;
  previousTimestamp = timestamp;

  // Update gyroscope AHRS algorithm
  if( !mag_present) {
    FusionAhrsUpdateNoMagnetometer( &ahrs, gyroscope, accelerometer, deltaTime);
  } else {
    FusionAhrsUpdate( &ahrs, gyroscope, accelerometer, magnetometer, deltaTime);
  }

  // Print algorithm outputs
  const FusionEuler euler = FusionQuaternionToEuler( FusionAhrsGetQuaternion(&ahrs));
  const FusionVector earth = FusionAhrsGetEarthAcceleration( &ahrs);
  const FusionVector lin_acc = FusionAhrsGetLinearAcceleration( &ahrs);
  
  pitch = euler.angle.pitch;
  roll = euler.angle.roll;
  yaw = euler.angle.yaw;

  fe_ax = earth.axis.x;
  fe_ay = earth.axis.y;
  fe_az = earth.axis.z;

  fl_ax = lin_acc.axis.x;
  fl_ay = lin_acc.axis.y;
  fl_az = lin_acc.axis.z;

  float v_batt = lipo_v();

  //altitude_filt = filt_1ord( altitude, altitude_filt_z1, 0.10, 0.01);
  //degC_filt = filt_1ord( degC, degC_filt_z1, 1.0, 0.01);

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

  //altitude_filt_z1 = altitude_filt;
  //degC_filt_z1 = degC_filt;
}

void app_step_10Hz( void)
{

  static unsigned char boot_btn_dly = 2;
  static float acc_filt_z1, gz_filt_z1;

  float acc = sqrt( ax*ax + ay*ay + az*az);
  acc_filt = filt_1ord( acc, acc_filt_z1, 1.0, 0.10);
  gz_filt = filt_1ord( gz, gz_filt_z1, 1.0, 0.10);

  if( sd_log.logging) {
    float v_batt = lipo_v();
    printf( "%5.1f (v) : ", v_batt);
    printf( " [% 5.1f, % 5.1f, % 5.1f] (g)", ax, ay, az); // g @ rest) - z==+1.0 is flat and upside down z==-1.0 is flat and right side up (i.e., in-flight for most throws)
    printf( " [% 6.1f, % 6.1f, % 6.1f] (deg/sec)", gx, gy, gz); // deg/sec @ rest - noise ~0.002 rad/sec
    printf( " [% 7.2f, % 7.2f, % 7.2f] (uT)", mx, my, mz); // @ rest ~15-50uT
    printf( " [% 6.1f, % 6.1f, % 6.1f] (deg)", roll, pitch, yaw); // deg
    //printf( " [% 7.1f, % 6.1f]", altitude_filt, degC_filt); // m, degC
    printf( " [% 6.1f, % 6.1f, % 6.1f] (g)", fe_ax, fe_ay, fe_az); // @ rest ~0
    //printf( " [% 7.2f, % 7.2f, % 7.2f] (g)", fl_ax, fl_ay, fl_az); // @ rest ~0
    printf( "\n");
/*
    printf( "%f,%f,%f", ax, ay, az); // g @ rest) - z==+1.0 is flat and upside down z==-1.0 is flat and right side up (i.e., in-flight for most throws)
    printf( "%f,%f,%f", gx, gy, gz); // deg/sec @ rest - noise ~0.002 rad/sec
    printf( "%f,%f,%f", mx, my, mz); // @ rest ~15-50uT
    printf( "%f,%f,%f", roll, pitch, yaw); // deg
    printf( "\n");
*/
    stat_led = !stat_led;
  }

  acc_filt_z1 = acc_filt;
  gz_filt_z1 = gz_filt;
}

short k_sleep_dly = 30;
void app_step_1Hz( void)
{

  static unsigned char boot_btn_dly = 2;

  if( !sd_log.logging) {
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

  static short sleep_dly = k_sleep_dly;
  bool acc_ok = acc_filt <= 1.1;
  bool gyro_ok = abs( gz_filt) <= 1.5;
  if( acc_ok && gyro_ok) {
    if( sleep_dly-- <= 0) {
      sleep_dly = k_sleep_dly;
      vfb_deep_sleep();
    }
  } else {
    sleep_dly = k_sleep_dly;
  }
}
