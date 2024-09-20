#include <SparkFun_MMC5983MA_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_MMC5983MA

#include "mag.h"
#include "vfb.h"

bool mag_update_offsts(uint32_t *offsetX, uint32_t *offsetY, uint32_t *offsetZ);

bool mag_present;
SFE_MMC5983MA mag3dof;

uint32_t k_mag_x_offst = 131072;
uint32_t k_mag_y_offst = 131072;
uint32_t k_mag_z_offst = 131072;

// these values were derived using magcal
float k_mag_cal_b[3] = {-2.9743, 16.0530, -4.9251};
float k_mag_cal_A[3][3] = {
  {1.0190, 0.0037, 0.0070},
  {0.0037, 0.9851, 0.0161},
  {0.0070, 0.0161, 0.9965}
};

void mag_init( void) {
  // Begin the Mag
  mag_present = 0;
  if( !mag3dof.begin(MAG_CS) )
  {
    Serial.println(F("Mag did not begin. Freezing..."));
  }
  mag_present = 1;

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

void mag_step( float* mx, float* my, float* mz) {
  static float mx_z1, my_z1, mz_z1;
  uint32_t mag_raw_x, mag_raw_y, mag_raw_z;
  float mx_uT, my_uT, mz_uT;

  if( !mag_present) {
    *mx = 0.0;
    *my = 0.0;
    *mz = 0.0;
    return;
  }
  mag3dof.getMeasurementXYZ( &mag_raw_x, &mag_raw_y, &mag_raw_z);

  // The magnetic field values are 18-bit unsigned. The zero (mid) point is 2^17 (131072).
  // Normalize each field to +/- 800.0 uT 
  mx_uT = ((float)(mag_raw_x) - (float)( k_mag_x_offst)) * (800.0 / 131072.0); // uT
  my_uT = ((float)(mag_raw_y) - (float)( k_mag_y_offst)) * (800.0 / 131072.0); // uT
  mz_uT = ((float)(mag_raw_z) - (float)( k_mag_z_offst)) * (800.0 / 131072.0); // 100 uT == 1 G

  // reject samples > 200 uT as this is likely erroneous - during calibration
  if( abs( mx_uT) >= 200.0) { mx_uT = mx_z1;}
  if( abs( my_uT) >= 200.0) { my_uT = my_z1;}
  if( abs( mz_uT) >= 200.0) { mz_uT = mz_z1;}
  // reject samples > 200 uT as this is likely erroneous - during calibration

  // average k and k-1 samples
  *mx = (mx_uT + mx_z1) / 2.0;
  *my = (my_uT + my_z1) / 2.0;
  *mz = (mz_uT + mz_z1) / 2.0;

  // update states
  mx_z1 = mx_uT;
  my_z1 = my_uT;
  mz_z1 = mz_uT;
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
