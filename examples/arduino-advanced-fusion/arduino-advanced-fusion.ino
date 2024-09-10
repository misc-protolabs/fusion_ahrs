#include <TaskScheduler.h>
#include "Fusion.h"
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <SparkFun_ISM330DHCX.h> //Click here to get the library: http://librarymanager/All#SparkFun_6DoF_ISM330DHCX
#include <SparkFun_MMC5983MA_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_MMC5983MA

void step_100Hz( void);
void imu_init( void);
void mag_init( void);
void imu_step( void);
void imu_init( void);

// scheduler
Scheduler sched;

// app tasks
Task tsk_app_100Hz ( 10 * TASK_MILLISECOND, -1, &step_100Hz, &sched, true );

SparkFun_ISM330DHCX_SPI imu6dof; 

SFE_MMC5983MA mag3dof;
uint32_t k_mag_x_offst = 131072;
uint32_t k_mag_y_offst = 131072;
uint32_t k_mag_z_offst = 131072;

float gx, gy, gz;
float ax, ay, az;
float mx, my, mz;

#define SAMPLE_RATE (100) // replace this with actual sample rate

// Tested with Espressif ESP32 v2.0.5 and the "ESP32 Dev Module" board definition
#define EN_3V3_SW 32 // The 3.3V_SW regulator Enable pin is connected to D32
#define BOOT_BUTTON 0 // DataLogger IoT – 9DoF Boot Button is connected to D0
#define STAT_LED 25 // DataLogger IoT – 9DoF STAT LED is connected to D25
#define RGB_LED 26 // DataLogger IoT – 9DoF RGB LED is connected to D26
#define IMU_CS 5
#define IMU_INT1 34
#define MAG_CS 27
#define MAG_INT 35

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

//    FusionConventionNwu, /* North-West-Up */
//    FusionConventionEnu, /* East-North-Up */
//    FusionConventionNed, /* North-East-Down */
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

void setup( void)
{
  Serial.begin( 115200);
  Wire.begin();

  Serial.println( "- io-init...");
  pinMode(EN_3V3_SW, OUTPUT);
  digitalWrite(EN_3V3_SW, HIGH);

  pinMode(BOOT_BUTTON, INPUT);
  pinMode(STAT_LED, OUTPUT);

  pinMode(IMU_CS, OUTPUT);
  digitalWrite(IMU_CS, HIGH);

  pinMode(IMU_INT1, INPUT);

  pinMode(MAG_CS, OUTPUT);
  digitalWrite(MAG_CS, HIGH);

  digitalWrite( STAT_LED, HIGH);

  delay(100);

  Serial.println( "- imu-init...");
  imu_init();
  //Serial.println( "- mag-init...");
  //mag_init();
  delay(100);

  Serial.println( "- fusion-init...");
  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);

  FusionAhrsSetSettings(&ahrs, &settings);
}

void loop( void)
{
  sched.execute();
}

void step_100Hz( void)
{
  // Acquire latest sensor data
  const clock_t timestamp = clock(); // replace this with actual gyroscope timestamp
  imu_step();
  //mag_step();
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
  //const FusionAhrsInternalStates = FusionAhrsGetInternalStates( &ahrs);
  //const FusionAhrsFlags = FusionAhrsGetFlags( &ahrs);
  
  float pitch = euler.angle.pitch;
  float roll = euler.angle.roll;
  float yaw = euler.angle.yaw;

  Serial.printf( "%5.3f (s) : ", deltaTime);
  Serial.printf( " [% 5.2f, % 5.2f, % 5.2f] (g)", ax, ay, az); // g @ rest) - z==+1.0 is flat and upside down z==-1.0 is flat and right side up (i.e., in-flight for most throws)
  Serial.printf( " [% 7.2f, % 7.2f, % 7.2f] (deg/sec)", gx, gy, gz); // deg/sec @ rest - noise ~0.002 rad/sec
  Serial.printf( " [% 7.2f, % 7.2f, % 7.2f] (uT)", mx, my, mz); // @ rest ~15-23uT
  Serial.printf( " [% 7.1f, % 7.1f, % 7.1f] (deg)", pitch, roll, yaw); // deg
  Serial.println();
}

void imu_init( void) {
  // Begin the IMU
  if( !imu6dof.begin(IMU_CS) )
  {
    //Serial.println(F("IMU did not begin. Freezing..."));
    while(1);
  }

  // Reset the device to default settings
  // This if helpful is you're doing multiple uploads testing different settings. 
  imu6dof.deviceReset();

  // Wait for it to finish reseting
  while( !imu6dof.getDeviceReset() )
  { 
    delay(1);
  } 

  //Serial.println(F("IMU has been reset."));
  //Serial.println(F("Applying settings..."));
  delay(100);
  
	imu6dof.setDeviceConfig();
	imu6dof.setBlockDataUpdate();
	
	// Set the output data rate and precision of the accelerometer
	imu6dof.setAccelDataRate(ISM_XL_ODR_104Hz);
	imu6dof.setAccelFullScale(ISM_16g); 

	// Set the output data rate and precision of the gyroscope
	imu6dof.setGyroDataRate(ISM_GY_ODR_104Hz);
	imu6dof.setGyroFullScale(ISM_4000dps); 

	// Turn on the accelerometer's filter and apply settings. 
	imu6dof.setAccelFilterLP2();
	imu6dof.setAccelSlopeFilter( ISM_LP_ODR_DIV_100);

	// Turn on the gyroscope's filter and apply settings. 
	imu6dof.setGyroFilterLP1();
	imu6dof.setGyroLP1Bandwidth( ISM_ULTRA_LIGHT);
}

void imu_step( void) {
  sfe_ism_data_t accel_raw;
  sfe_ism_data_t gyro_raw;

  if( imu6dof.checkStatus()) {
    imu6dof.getAccel( &accel_raw);        // mg
    imu6dof.getGyro( &gyro_raw);          // mdps
  }

  ax = ( accel_raw.xData * 0.001); // g
  ay = ( accel_raw.yData * 0.001); // g
  az = ( accel_raw.zData * 0.001); // 1g =~ 9.806 m/s^2

  gx = ( gyro_raw.xData * 0.001); // deg/s
  gy = ( gyro_raw.yData * 0.001); // deg/s
  gz = ( gyro_raw.zData * 0.001); // deg/s
}

void mag_init( void) {
  // Begin the Mag
  if( !mag3dof.begin(MAG_CS) )
  {
    Serial.println(F("Mag did not begin. Freezing..."));
    while(1);
  }

  mag3dof.softReset();

  Serial.println("MMC5983MA connected");

  Serial.println("Setting filter bandwith to 800 Hz for continuous operation...");
  mag3dof.setFilterBandwidth(100);
  Serial.print("Reading back filter bandwith: ");
  Serial.println(mag3dof.getFilterBandwith());

  Serial.println("Setting continuous measurement frequency to 100 Hz...");
  mag3dof.setContinuousModeFrequency(100);
  Serial.print("Reading back continuous measurement frequency: ");
  Serial.println(mag3dof.getContinuousModeFrequency());

  Serial.println("Enabling auto set/reset...");
  //mag3dof.enableAutomaticSetReset();
  mag3dof.disableAutomaticSetReset();
  Serial.print("Reading back automatic set/reset: ");
  Serial.println(mag3dof.isAutomaticSetResetEnabled() ? "enabled" : "disabled");

  Serial.println("Enabling continuous mode...");
  mag3dof.enableContinuousMode();
  Serial.print("Reading back continuous mode: ");
  Serial.println(mag3dof.isContinuousModeEnabled() ? "enabled" : "disabled");

  mag_update_offsts( &k_mag_x_offst, &k_mag_y_offst, &k_mag_z_offst);
  Serial.printf( "mag-offst = [ %7i, %7i, %7i]\n", k_mag_x_offst, k_mag_y_offst, k_mag_z_offst);
}

void mag_step( void) {
  uint32_t mag_raw_x, mag_raw_y, mag_raw_z;
  static float mx_z1, my_z1, mz_z1;

  mag3dof.getMeasurementXYZ( &mag_raw_x, &mag_raw_y, &mag_raw_z);

  // The magnetic field values are 18-bit unsigned. The zero (mid) point is 2^17 (131072).
  // Normalize each field to +/- 800.0 uT 
  mx = ((float)(mag_raw_x) - (float)( k_mag_x_offst)) * (800.0 / 131072.0); // uT
  my = ((float)(mag_raw_y) - (float)( k_mag_y_offst)) * (800.0 / 131072.0); // uT
  mz = ((float)(mag_raw_z) - (float)( k_mag_z_offst)) * (800.0 / 131072.0); // 100 uT == 1 G

  if( abs( mx) >= 200.0) { mx = mx_z1;}
  if( abs( my) >= 200.0) { my = my_z1;}
  if( abs( mz) >= 200.0) { mz = mz_z1;}

  mx_z1 = mx;
  my_z1 = my;
  mz_z1 = mz;
}

bool mag_update_offsts(uint32_t *offsetX, uint32_t *offsetY, uint32_t *offsetZ) // Update the offsets
{
  bool success = true; // Use AND (&=) to record if any one command fails

  success &= mag3dof.performSetOperation(); // Perform the SET operation

  uint32_t setX = 131072;
  uint32_t setY = 131072;
  uint32_t setZ = 131072;

  success &= mag3dof.getMeasurementXYZ(&setX, &setY, &setZ); // Read all three channels
  success &= mag3dof.getMeasurementXYZ(&setX, &setY, &setZ); // Do it twice - just in case there is noise on the first

  success &= mag3dof.performResetOperation(); // Perform the RESET operation

  uint32_t resetX = 131072;
  uint32_t resetY = 131072;
  uint32_t resetZ = 131072;

  success &= mag3dof.getMeasurementXYZ(&resetX, &resetY, &resetZ); // Read all three channels
  success &= mag3dof.getMeasurementXYZ(&resetX, &resetY, &resetZ); // Do it twice - just in case there is noise on the first

  // Calculate the offset - as per the datasheet.
  // The measurements are 18-bit so it's OK to add them directly.
  if (success)
  {
    *offsetX = (setX + resetX) / 2;
    *offsetY = (setY + resetY) / 2;
    *offsetZ = (setZ + resetZ) / 2;
  }

  return success;
}
