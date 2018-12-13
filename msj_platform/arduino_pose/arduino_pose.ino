#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "wirelessLove.hpp"

WirelessLove *wifi;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055();

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
 
  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }
 
  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  adafruit_bno055_offsets_t calibData;
  calibData.accel_offset_x = 20;
  calibData.accel_offset_y = 10;
  calibData.accel_offset_z = 6;

  calibData.gyro_offset_x = -5;
  calibData.gyro_offset_y = 0;
  calibData.gyro_offset_z = 0;

  calibData.mag_offset_x = 24;
  calibData.mag_offset_y = 214;
  calibData.mag_offset_z = 104;

  calibData.accel_radius = 1000;
  calibData.mag_radius = 688;

  bno.setSensorOffsets(calibData);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  IPAddress ip(192,168,255,255);
  wifi = new WirelessLove("roboy","wiihackroboy",ip);
  while(!wifi->connected ){
    if(wifi->connected  && wifi->initUDPSockets()){
      Serial.println("connected");
    }else{
      Serial.println("not connected");
    }
    delay(1000);
  }
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

//  /* Display the floating point data */
//  Serial.print("X: ");
//  Serial.print(euler.x());
//  Serial.print(" Y: ");
//  Serial.print(euler.y());
//  Serial.print(" Z: ");
//  Serial.print(euler.z());
//  Serial.print("\t\t");

  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  float qx = quat.x(), qy = quat.y(), qz = quat.z(), qw = quat.w();
  union{
    uint32_t q[4];
    uint8_t data[16];
  }q;
  
  q.q[0] = pack754_32(qx);
  q.q[1] = pack754_32(qy);
  q.q[2] = pack754_32(qz);
  q.q[3] = pack754_32(qw);

  static int counter = 0;
  if(counter++%100==0){
    // Quaternion data
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qZ: ");
    Serial.println(quat.z(), 4);
    displayCalStatus();
//    adafruit_bno055_offsets_t newCalib;
//    bno.getSensorOffsets(newCalib);
//    displaySensorOffsets(newCalib);
    uint8_t data[4];
    bno.getCalibration(&data[0], &data[1], &data[2], &data[3]);
    Serial.print("Sys:");
    Serial.print(data[0], DEC);
    Serial.print(" G:");
    Serial.print(data[1], DEC);
    Serial.print(" A:");
    Serial.print(data[2], DEC);
    Serial.print(" M:");
    Serial.println(data[3], DEC);
    wifi->broadcast_send(data,4);
  }

  wifi->broadcast_send(q.data,16);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
