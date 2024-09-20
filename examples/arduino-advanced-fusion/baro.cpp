#include <Wire.h>
#include "SparkFunMPL3115A2.h"

#include "baro.h"
#include "vfb.h"

void set_sealevel(float pressure);
void I2C_Write(byte regAddr, byte value);

//Create an instance of the object
MPL3115A2 baro;

void baro_init(void) {
  baro.begin(); // Get sensor online

  //Configure the sensor
  baro.setModeAltimeter(); // Measure altitude above sea level in meters
  //baro.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa

  baro.setOversampleRate(2); // Set Oversample to the minimum (highest bandwidth)
  baro.enableEventFlags(); // Enable all three pressure and temp event flags
  set_sealevel( 101.325);
  baro.setModeActive();
}

void baro_step( float* altitude, float* degC) {
  //float pressure = baro.getPressure();
  //*altitude = baro.getAltitude();
  //*degC = baro.getTemperature();
  //*altitude = baro.getLastConversionResults( MPL3115A2_ALTITUDE);
  //*degC = baro.getLastConversionResults( MPL3115A2_TEMPERATURE);
  //*degC = 0;
  //baro.startOneShot();
  //float pressure = baro.readPressure();
  //float p_term = pow((pressure/101326.0), 0.1902632);
  *altitude = baro.readAltitude();
  *degC = baro.readTemp();
}

void set_sealevel(float pressure) {
  uint16_t P_value = pressure*500;
  uint8_t register_list[2] = {P_value>>8, P_value};
  //reg_w( MPL3115_BAR_IN_MSB, register_list, 2);
  I2C_Write( BAR_IN_MSB, register_list[0]);
  I2C_Write( BAR_IN_LSB, register_list[1]);
}

void I2C_Write(byte regAddr, byte value)
{
  // This function writes one byto over IIC
  Wire.beginTransmission(0x60);
  Wire.write(regAddr);
  Wire.write(value);
  Wire.endTransmission(true);
}
 //References: 
  //Definition of "altimeter setting": http://www.crh.noaa.gov/bou/awebphp/definitions_pressure.php
  //Altimeter setting: http://www.srh.noaa.gov/epz/?n=wxcalc_altimetersetting
  //Altimeter setting: http://www.srh.noaa.gov/images/epz/wxcalc/altimeterSetting.pdf
  //Verified against Boulder, CO readings: http://www.crh.noaa.gov/bou/include/webpres.php?product=webpres.txt
