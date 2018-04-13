/*
  Project: Project 5
  Group: 11
  Date: 2017-03-30

  Authors: Forest Plasencia, Blake Herren, Jacob Burcham

  Responsible author: Forest Plasencia
*/

// Declare enum state for each sensor as a global variable

#include "project.h"
#include "OpticalFlowCamera.h"
#include "PeriodicAction.h"
#include "MPU9250.h"


////////////////////////////////////////////////////
// Global constants

// Total number lateral motors
const int NUM_LAT_MOTORS = 3;

// Motor Pins
const int MOTOR_LIFT = 29;
const int MOTOR_RIGHT = 37;
const int MOTOR_LEFT = 30;
const int MOTOR_BACK = 38;

// Total number of cameras
const int NUM_CAMERAS = 3;

// Select pins for the 3 cameras
const uint8_t CAMERA_SELECT[NUM_CAMERAS] = {33, 34, 35};

// Common reset pin
const uint8_t RESET_PIN = 36;

// Sensor Ports
const int16_t leftSensor = 7;
const int16_t rightSensor = 8;

// Array for x coefficients from linear regression of raw movement data
const float xCoef[7] =
{
  0.4446935015,
  0.0000280208,
  -0.0000762170,
  0.00000594226,
  -0.0000120181,
  0.0001457229,
  0.0001750026
};

// Array for y coefficients from linear regression of raw movement data
const float yCoef[7] =
{
  -0.1066484176,
  0.0000232153,
  0.0000143951,
  0.0001553162,
  -0.0000371591,
  -0.0002395083,
  0.0001435592
};

// Array of theta coefficients from linear regression of raw movement data
const float tCoef[7] =
{
  4.1591854032,
  -0.0003219264,
  0.0003884384,
  -0.0013492435,
  0.0003089894,
  0.0005892711,
  -0.0020015830
};


/////////////////////////////////////////////////////
// Global variables

// Camera interface object
OpticalFlowCamera cameras(RESET_PIN);

// Arrays for accumulated slip values
int32_t adx[NUM_CAMERAS];
int32_t ady[NUM_CAMERAS];
int32_t qualityAr[NUM_CAMERAS];
float motion[NUM_CAMERAS];

// Arrays for motors
int16_t magnitudes[NUM_LAT_MOTORS];
int32_t motorPins[NUM_LAT_MOTORS] = {MOTOR_LEFT, MOTOR_RIGHT, MOTOR_BACK};


//////// **** IMU SETUP CODE **** //////////

// Promise that we will implement this function later
void control_step();

// Create a task that will be executed once per 10 ms
PeriodicAction control_task(10, control_step);

// Create a connection to the inertial measurement unit
MPU9250 IMU;

/**
   Initialize the IMU and verify that it is functioning.

   References: MPU9250 Basic Example Code, Kris Winer (April 1, 2014)
*/
void setup_imu()
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

  // Calibrate the gyros and accelerometers.
  // NOTE: THE HOVERCRAFT MUST BE STATIONARY DURING THIS CALIBRATION
  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);

  // Initialize the IMU
  IMU.initMPU9250();

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

  // Retrieve magnetometer calibration
  IMU.initAK8963(IMU.magCalibration);
}


/*
  Function Name: setup

  Initialize serial port on Teensy

  Inputs:
  none

  Outputs/Effects
  Initialize inputs and outputs
*/
int ret;
void setup() {
  // Initialize serial port & send/recieve @ 115,200 bit/s
  Serial.begin(115200);

  // Initialize the IMU
  setup_imu();


  // Velocity Ports
  PORTA_PCR16 = PORT_PCR_MUX(0x1); // top
  PORTA_PCR15 = PORT_PCR_MUX(0x1);
  PORTA_PCR14 = PORT_PCR_MUX(0x1);
  PORTA_PCR5 = PORT_PCR_MUX(0x1);
  PORTA_PCR12 = PORT_PCR_MUX(0x1); // bottom


  // Motor initialization
  PORTD_PCR0 = PORT_PCR_MUX(0x1);
  PORTD_PCR1 = PORT_PCR_MUX(0x1);
  PORTD_PCR2 = PORT_PCR_MUX(0x1);
  PORTD_PCR3 = PORT_PCR_MUX(0x1);
  PORTD_PCR4 = PORT_PCR_MUX(0x1);
  PORTD_PCR7 = PORT_PCR_MUX(0x1);

  // LED initialization

  GPIOA_PDDR = 0x1D020;

  // Initialize pins for motors
  GPIOD_PDDR = 0x9F;

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

   References: MPU9250 Basic Example Code, Kris Winer (April 1, 2014)
*/
void update_imu()
{
  // Read the accelerometer values and compute calibrated accelerations
  IMU.readAccelData(IMU.accelCount);
  IMU.getAres();
  IMU.ax = (float) IMU.accelCount[0] * IMU.aRes;
  IMU.ay = (float) IMU.accelCount[1] * IMU.aRes;
  IMU.az = (float) IMU.accelCount[2] * IMU.aRes;

  // Read the gyros and compute calibrated velocities
  IMU.readGyroData(IMU.gyroCount);
  IMU.getGres();
  IMU.gx = (float)IMU.gyroCount[0] * IMU.gRes;
  IMU.gy = (float)IMU.gyroCount[1] * IMU.gRes;
  IMU.gz = (float)IMU.gyroCount[2] * IMU.gRes;
}

/*
  Function Name: clip

  loop through main program, getting the calibrated sensor distance
  and print each distance out

  Inputs:
  value, min_value, max_value

  Outputs/Effects
  Compares value to max_value and min_value. If value is less than
  minimum, function clips to min_value. If value is greater than
  max_value, function clips to max_value. Otherwise, value is
  returned.
*/

int16_t clip(int16_t value, int16_t min_value, int16_t max_value) {
  if (value < min_value) {   //clips to minimum
    value = min_value;
  }
  else if (value > max_value) {   //clips to maximum
    value = max_value;
  }
  return (value);
}

/*
  Function Name: set_lift_motor_magnitude

  sets thrust magnitude for lift fan. ensures the magnitude
  is between 0 and 255 which corresponds to 0%-100% duty PWM cycle.

  Inputs:
  magnitude

  Outputs/Effects
  sets the desired duty cycle for the lift fan and clips the magnitude
  value if it lies outside of 0-255.
*/

void set_lift_motor_magnitude(int16_t magnitude) {
  magnitude  =  clip(magnitude, 0, 255); // clips value to acceptable range of 0-255
  analogWrite(MOTOR_LIFT, magnitude); // Assigns lift motor power
  //return;
}

/*
  Function Name: set_lateral_motor_magnitudes

  sets thrust magnitudes for three lateral fans. positions 0,1,2
  correspond to fans LEFT,RIGHT,BACK respectively.

  Inputs:
  magnitudes[3] (array with 3 components)

  Outputs/Effects
  This functions clips the values if the lie outside a range of
  -255-255. -255 corresponds to a 100% duty cycle in the opposite
  direction that 255 represents.
*/

void set_lateral_motor_magnitudes(int16_t magnitudes[3]) {
  for (int i = 0; i < 3; i++) {     //cycles through array
    magnitudes[i] = clip(magnitudes[i], -255, 255); // clips magnitudes to range -255-255
    if (magnitudes[i] < 0) {
      magnitudes[i] *= -1; // changes negative magnitudes to positive
      set_lateral_direction(i, BACKWARD);  // Sets motor direction as backward
    } else if (magnitudes[i] > 0) {
      set_lateral_direction(i, FORWARD);  // Sets motor rotation as Forward
    }


    analogWrite(motorPins[i], magnitudes[i]);  // Assigns magnitudes to motor output
  }
  return;
}

/*
  Function Name: set_lateral_motor_magnitudes

  sets thrust magnitudes for three lateral fans. positions 0,1,2
  correspond to fans LEFT,RIGHT,BACK respectively.

  Inputs:
  magnitudes[3] (array with 3 components)

  Outputs/Effects
  This functions clips the values if the lie outside a range of
  -255-255. -255 corresponds to a 100% duty cycle in the opposite
  direction that 255 represents.
*/

void set_lateral_direction(int16_t motor, lateralDirection direc) {
  switch (motor) {
    case 0:
      // Left motor
      if (direc == FORWARD) {
        GPIOD_PDOR = (GPIOD_PDOR & ~0x90) | (0x80);
      } else if (direc == BACKWARD) {
        GPIOD_PDOR = (GPIOD_PDOR & ~0x90) | (0x10);
      }
      break;
    case 1:
      // right motor
      if (direc == FORWARD) {
        GPIOD_PDOR = (GPIOD_PDOR & ~0xC) | (0x04);
      } else if (direc == BACKWARD) {
        GPIOD_PDOR = (GPIOD_PDOR & ~0xC) | (0x08);
      }
      break;
    case 2:
      // back motor
      if (direc == FORWARD) {
        GPIOD_PDOR = (GPIOD_PDOR & ~0x3) | (0x01);
      } else if (direc == BACKWARD) {
        GPIOD_PDOR = (GPIOD_PDOR & ~0x3) | (0x02);
      }
      break;
  }

}

/*
  Function Name: display_heading_velocity

  Light appropriate LED Pair according to Velocity input in
  tenths of degrees per second


  Inputs:
   int16_t velocity: a velocity in 10ths of a degree per second

  Outputs/Effects
   Light the appropriate LED Pair according to heading velocity

*/

void display_heading_velocity(int16_t velocity) {
  // pins to set
  int x = 0x00;
  // Branches convert heading velocity to LED Light Pair
  if (velocity > 2333) {
    // Pair 1
    x = 0x10000;
  } else if (velocity > 1667 && velocity <= 2333) {
    // Pair 2
    x = 0x10000;
  } else if (velocity > 1000 && velocity <= 1667) {
    // Pair 3 - Middle
    x = 0x08000;
  } else if (velocity > 333 && velocity <= 1000) {
    // Pair 4
    x = 0x08000;
  } else if (velocity >= -333 && velocity <= 333) {
    // Pair 5 - Middle
    x = 0x04000;
  } else if (velocity >= -1000 && velocity < -334) {
    // Pair 6
    x = 0x00020;
  } else if (velocity >= -1667 && velocity < -1000) {
    // Pair 7
    x = 0x00020;
  } else if (velocity >= -2333 && velocity < -1667) {
    // Pair 8
    x = 0x01000;
  } else if (velocity < -2333) {
    // Pair 9
    x = 0x01000;
  }

  // update led bar
  GPIOA_PDOR = (GPIOA_PDOR & ~0x1D020) | (x);
}


/*
  Function Name: set_hovercraft_acceleration

  This function accepts as input a desired acceleration of the hovercraft,
  translates this desired acceleration into an appropriate thrust level for each
  of the lateral fans, and changes the thrust state of these fans.

  Inputs:
  ddx, ddy, ddtheta

  Outputs/Effects
  Sets the thrust state of the lateral fans to levels that perform the desired acceleration
*/
void set_hovercraft_acceleration(float ddx, float ddy, float ddtheta) {
  // Radius to motor
  float R = 0.1397;
  float sin30 = 0.5;
  float cos30 = 0.866;

  // F1 - left fan
  magnitudes[0] = (ddx / (2 * cos30)) - (ddy / (2 + (2 * sin30))) - (ddtheta / (2 * R * (1 + sin30)));

  // F2 - right fan
  magnitudes[1] = (ddx / (2 * cos30)) + (ddy / (2 + (2 * sin30))) + (ddtheta / (2 * R * (1 + sin30)));

  // F3 - back fan
  magnitudes[2] =  -(ddy / (1 + sin30)) + ((ddtheta * sin30) / (R * (1 + sin30)));

  // set magnitudes of motors
  set_lateral_motor_magnitudes(magnitudes);

}

/**
   Single sensing & control step

   Note: delays are allowed in this function (or any function called by this function)
*/

float Kv = 0.05;
float rotation_rate;
int32_t lift_magnitude = 0;


void control_step()
{
  // Update the state of the IMU
  update_imu();

  // Count the number of times that this function has been called
  static unsigned long counter = 0;

  // assigns rotation rate
  rotation_rate = IMU.gz * 10;
  display_heading_velocity(rotation_rate);

  // Increment the counter (1 per 10 ms)
  counter++;
  //Serial.printf("Counter: %d \n", counter);
  if (counter < 500) {
    // Ramp up the central fan to 100%

    set_lift_motor_magnitude(counter);
    lift_magnitude = counter;


  } else if (counter < 6500) {
    // Implement control law here
    float ddx = 0.0;
    float ddy = 0.0;

    float ddtheta = -Kv * rotation_rate;
    set_hovercraft_acceleration(ddx, ddy, ddtheta);

  } else if (counter < 7000) {
    // Ramp down the central fan to 0%
    for (int i = 0; i < 3; i++)
    {
      magnitudes[i] = 0;
    }
    set_lateral_motor_magnitudes(magnitudes);

    // ramp lift motor down to 0% duty cycle
    set_lift_motor_magnitude(lift_magnitude);
    lift_magnitude -= 1;


  } // Else do nothing
}

/*
  Function Name: loop

  loop through main program, getting the calibrated sensor distance
  and print each distance out

  Inputs:
  none

  Outputs/Effects
  Print out the calculated distance of each sensor
*/


void loop() {

  // Check to see if it is time to execute the control_task
  control_task.step();


} // close void loop
