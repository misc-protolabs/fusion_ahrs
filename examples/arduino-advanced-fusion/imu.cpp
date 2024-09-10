#include <SparkFun_ISM330DHCX.h> //Click here to get the library: http://librarymanager/All#SparkFun_6DoF_ISM330DHCX

#include "imu.h"
#include "vfb.h"

// SPI instance class call
SparkFun_ISM330DHCX_SPI imu6dof; 

bool imu_int_isr_flg;
float k_acc_x_offst = 0;
float k_acc_y_offst = 0;
float k_acc_z_offst = 0;
float k_gyro_x_offst = -0.004887;
float k_gyro_y_offst = +0.009774;
float k_gyro_z_offst = +0.014661;

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

void imu_step( float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {
  sfe_ism_data_t accel_raw;
  sfe_ism_data_t gyro_raw;

  if( imu6dof.checkStatus()) {
    imu6dof.getAccel( &accel_raw);        // mg
    imu6dof.getGyro( &gyro_raw);          // mdps
  }

  *ax = ( accel_raw.xData * 0.001); // g
  *ay = ( accel_raw.yData * 0.001); // g
  *az = ( accel_raw.zData * 0.001); // 1g =~ 9.806 m/s^2

  *gx = ( gyro_raw.xData * 0.001); // deg/s
  *gy = ( gyro_raw.yData * 0.001); // deg/s
  *gz = ( gyro_raw.zData * 0.001); // deg/s
}
