#include "ImuUtils.h"


#include "quaternionFilters.h"
// Create a connection to the inertial measurement unit
MPU9250 IMU;

/**
   Initialize the IMU and verify that it is functioning.
*/
void imu_setup()
{
  // Initialize the I2C bus (to which the IMU is connected)
  Wire.begin();

  // Test the connection to the IMU
  byte c = IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (c == 0x71) {
    Serial.printf("IMU initialized successfully\n");
  } else {
    Serial.printf("*** UNABLE TO CONTACT IMU ***\n");
    // Loop forever
    while (1) {
      delay(10);
    }
  }

  delay(10);

  // Calibrate the gyros and accelerometers.
  // NOTE: THE HOVERCRAFT MUST BE STATIONARY DURING THIS CALIBRATION
  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);

  delay(10);

  // Initialize the IMU
  IMU.initMPU9250();
  delay(10);

  // Check the magnetometer
  c = IMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  if (c == 0x48) {
    Serial.printf("Magnetometer initialized successfully\n");
  } else {
    Serial.printf("*** UNABLE TO CONTACT MAGNETOMETER ***\n");
    // Loop forever
    while (1) {
      delay(10);
    }
  }
  delay(10);


  ///////////////////////////////////////////////////////////
  /// NEW
  // Retrieve magnetometer calibration
  IMU.initAK8963(IMU.magCalibration);
  delay(10);
  /*
    IMU.count = millis();
    IMU.sumCount = 0;
    IMU.sum = 0;
  */

  // Hard-coded bias: replace

  IMU.magbias[0] = 58.116657;
  IMU.magbias[1] = -95.662689;
  IMU.magbias[2] = 166.964645;

  // Read the resolutions from the IMU
  IMU.getAres();
  IMU.getGres();
  IMU.getMres();

}

const int NUM_CALIBRATION_SAMPLES = 2000;

/**
   Calibrate the magnetometer biases.   The craft must be turned 2 full cycles while this data
   collection process is executing (10 seconds).

   In addition to calibrating the biases, this function will print out 3 lines of code that can
   be used in place of calling this function in the future (avoiding the need to do this calibration
   every time the program is started).

   The process is to take a set of samples from the magnetometer & compute the average.  This
   average becomes the new biases.
*/
void imu_calibrate_magbias()
{
  Serial.printf("Slowly turn the craft now (your goal is four complete rotations in 40 seconds)\n");
  Serial.printf("Starting in...\n");
  for (int i = 5; i > 0; --i) {
    Serial.printf("%d\n", i);
    delay(1000);
  }

  // Magnetometer resolution
  IMU.getMres();
  // Sum of 1000 magnetometer readings
  float bias[] = {0.0, 0.0, 0.0};
  int counter = 0;

  Serial.printf("Go\n");
  // Loop over a set of samples.
  for (int i = 0; i < NUM_CALIBRATION_SAMPLES; ++i)
  {
    // Read the magnetometer
    IMU.readMagData(IMU.magCount);
    // Add to the accumulated sum
    bias[0] += (float)IMU.magCount[0] * IMU.mRes * IMU.magCalibration[0];
    bias[1] += (float)IMU.magCount[1] * IMU.mRes * IMU.magCalibration[1];
    bias[2] += (float)IMU.magCount[2] * IMU.mRes * IMU.magCalibration[2];

    if (++counter % 50 == 0) {
      Serial.printf("%d\n", counter / 50);
      Serial.flush();
    }
    delay(20);
  }
  Serial.printf("\nDone\n");

  // Now compute the average: this will be our new bias
  for (int i = 0; i < 3; ++i)
  {
    IMU.magbias[i] = bias[i] / NUM_CALIBRATION_SAMPLES;
  }

  // Report the results
  Serial.printf("You may use this code instead of a call to imu_calibrate_magbias():\n");
  for (int i = 0; i < 3; ++i)
  {
    Serial.printf("IMU.magbias[%d] = %f;\n", i, IMU.magbias[i]);
  }

  Serial.printf("Sleeping while you copy this...\n");
  delay(10000);
}


/**
   Update the state of the IMU by reading calibrated linear acceleration and rotation
   rate information.

   Inputs:
    n/a
   Outputs:
    (implicit): the state of variables IMU.ax, IMU.ay and IMU.az encode linear acceleration in
                  g/sec
    (implicit): the state of variables IMU.gx, IMU.gy and IMU.gz encode rotation rate in
                  deg/sec
    (implicit): the state of the variables IMU.roll, IMU.pitch, IMU.yaw encode the current
                  orientation in deg

   References: MPU9250 Basic Example Code, Kris Winer (April 1, 2014)
*/
void imu_update()
{
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    // Read the accelerometer values and compute calibrated accelerations
    IMU.readAccelData(IMU.accelCount);
    IMU.ax = (float) IMU.accelCount[0] * IMU.aRes;
    IMU.ay = (float) IMU.accelCount[1] * IMU.aRes;
    IMU.az = (float) IMU.accelCount[2] * IMU.aRes;

    // Read the gyros and compute calibrated velocities
    IMU.readGyroData(IMU.gyroCount);
    IMU.gx = (float)IMU.gyroCount[0] * IMU.gRes;
    IMU.gy = (float)IMU.gyroCount[1] * IMU.gRes;
    IMU.gz = (float)IMU.gyroCount[2] * IMU.gRes;

    ////////////////////////////////////////////////////////////////
    // Read the magnetometer
    IMU.readMagData(IMU.magCount);

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    IMU.mx = (float)IMU.magCount[0] * IMU.mRes * IMU.magCalibration[0] -
             IMU.magbias[0];
    IMU.my = (float)IMU.magCount[1] * IMU.mRes * IMU.magCalibration[1] -
             IMU.magbias[1];
    IMU.mz = (float)IMU.magCount[2] * IMU.mRes * IMU.magCalibration[2] -
             IMU.magbias[2];
  }
  // Compute orientation
  // Must be called before updating quaternions!
  IMU.updateTime();
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientation mismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(IMU.ax, IMU.ay, IMU.az, IMU.gx * DEG_TO_RAD,
                         IMU.gy * DEG_TO_RAD, IMU.gz * DEG_TO_RAD, IMU.my,
                         IMU.mx, IMU.mz, IMU.deltat);

  // Now compute RPY
  // Define output variables from updated quaternion---these are Tait-Bryan
  // angles, commonly used in aircraft orientation. In this coordinate system,
  // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
  // x-axis and Earth magnetic North (or true North if corrected for local
  // declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the
  // Earth is positive, up toward the sky is negative. Roll is angle between
  // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
  // arise from the definition of the homogeneous rotation matrix constructed
  // from quaternions. Tait-Bryan angles as well as Euler angles are
  // non-commutative; that is, the get the correct orientation the rotations
  // must be applied in the correct order which for this configuration is yaw,
  // pitch, and then roll.
  // For more see
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // which has additional links.
  IMU.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
                            *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1) * *(getQ() + 1)
                    - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3) * *(getQ() + 3));
  IMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                            *(getQ() + 2)));
  IMU.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) *
                            *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1) * *(getQ() + 1)
                    - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3) * *(getQ() + 3));
  IMU.pitch *= RAD_TO_DEG;
  IMU.yaw   *= RAD_TO_DEG;
  // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
  //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
  // - http://www.ngdc.noaa.gov/geomag-web/#declination
  // We will not hardcode a declination here
  //IMU.yaw   -= 8.5;
  IMU.roll  *= RAD_TO_DEG;
}

