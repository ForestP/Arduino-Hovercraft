/*
  Project: Project 9
  Group: 11
  Date: 2017-04-24

  Authors: Forest Plasencia, Blake Herren, Jacob Burcham

  Responsible author: Forest Plasencia
*/

// Declare enum state for each sensor as a global variable

#include "project.h"
#include "OpticalFlowCamera.h"
#include "PeriodicAction.h"
#include "ImuUtils.h"


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

const float KLv = -3000.0; // scales the force of correction to move
const float Kp = 1.0; // scales the force of correction by the lateral fans
const float Kv = -0.05; // scales the force of dampening by the lateral fans around the target orientation
const float deadband = 2.5; // tolerance (degrees) of current orientation compared to target orientation
const float saturation = 45; // the plateau of the correction force due to maximizing lateral fan output

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

// Arrays for velocities
float velocity_filtered[3] = {0, 0, 0}; // x_dot, y_dot, theta_dot
float velocity_goal[3] = {0, 0, 0};

//////// **** IMU SETUP CODE **** //////////

// Promise that we will implement this function later
void control_step();
void sensor_step();
void report_step();
void camera_step();
void fsm_step();

// Create a task that will be executed once per 10 ms
PeriodicAction control_task(10, control_step);

// Create a sensor task that executes once per 5ms
PeriodicAction sensor_task(5, sensor_step);

// Create a report task that executes once per 250ms
PeriodicAction report_task(250, report_step);

// Create a camera task that executes once per 5ms
PeriodicAction camera_task(5, camera_step);

// Create a fsm task that executes once per 50ms
PeriodicAction fsm_task(50, fsm_step);

// Sets the goal theta orientation of the craft
float theta_goal = IMU.yaw;

// stores the error in theta
float theta_error = 0;

// Count the number of times that this control step has been called
unsigned long counter = 0;

// store calibrated sensor values
float calibrated_distances[2] = {0, 0}; // FRONT_SENSOR, RIGHT_SENSOR

// tracks number of walls encountered
int wall_num = 1;

// stores the current rotation rate
float rotation_rate;
// sets the lift magnitude
int32_t lift_magnitude = 0;


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
  imu_setup();

  // LED Velocity heading Ports
  // North LED
  PORTB_PCR0 = PORT_PCR_MUX(0x1);
  // East LED
  PORTB_PCR1 = PORT_PCR_MUX(0x1);
  // West LED
  PORTB_PCR11 = PORT_PCR_MUX(0x1);
  // South LED
  PORTB_PCR10 = PORT_PCR_MUX(0x1);

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

  // Camera
  PORTE_PCR24 = PORT_PCR_MUX(0x1);
  PORTE_PCR25 = PORT_PCR_MUX(0x1);
  PORTC_PCR8 = PORT_PCR_MUX(0x1);
  PORTC_PCR9 = PORT_PCR_MUX(0x1);

  // Switch initialization
  PORTE_PCR26 = PORT_PCR_MUX(0x1);
  //GPIOE_PDDR = 0x000;

  // Initialize pins for cameras
  GPIOE_PDDR = 0x3000000;
  GPIOC_PDDR = 0x300;

  for (int i = 0; i < NUM_CAMERAS; i++) {
    int ret = cameras.addCamera(CAMERA_SELECT[i]);
    Serial.print(ret);
  }

  GPIOA_PDDR = 0x1D020;

  // Initialize pins for motors
  GPIOD_PDDR = 0x9F;

  // Initialize pins for led heading
  GPIOB_PDDR = 0xC03;

  // Calibration for magnetometer
  IMU.magbias[0] = 221.655441;
  IMU.magbias[1] = 426.409760;
  IMU.magbias[2] = 327.984680;

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
  Function Name: display_heading

  Light appropriate LED(s) according to heading input, accprding to heading
  input in 10ths of degrees in left hand coordinate system

  Inputs:
   int16_t heading: a heading in tenths of degrees
   with a range of (-1799 - 1800)

  Outputs/Effects
   Light the appropriate LED(s) according to heading

*/

void display_heading(int16_t heading) {
  // pins to set
  int x = 0x00;
  // Branches convert heading deg to cardinal direction
  if (heading >= -225 && heading <= 225) {
    //North Heading
    x = 0x001;
  } else if (heading > 225 && heading < 675) {
    //NE Heading
    x = 0x003;
  } else if (heading >= 675 && heading <= 1125) {
    //E Heading
    x = 0x002;
  } else if (heading > 1125 && heading < 1575) {
    //SE Heading
    x = 0x402;
  } else if (heading >= 1575 || heading <= -1525) {
    //S Heading
    x = 0x400;
  } else if (heading < -1125 && heading > -1575) {
    // SW Heading
    x = 0xC00;
  } else if (heading <= -675 && heading >= -1125) {
    // W Heading
    x = 0x800;
  } else if (heading < -225 && heading > -675) {
    // NW Heading
    x = 0x801;
  }

  GPIOB_PDOR = (GPIOB_PDOR & 0xFFFFF0F0) | (x);
}

/*
  Function Name: set_lift_motor_magnitude

  sets thrust magnitude for lift fan. ensures the magnitude
  is between 0 and 255 which corresponds to 0%-100% PWM duty cycle.

  Inputs:
  magnitude

  Outputs/Effects
  sets the desired duty cycle for the lift fan and clips the magnitude
  value if it lies outside of 0-255.
*/

void set_lift_motor_magnitude(int16_t magnitude) {
  magnitude  =  clip(magnitude, 0, 230); // clips value to acceptable range of 0-230 (90%)
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
  Function Name: set_lateral_direction

  controls behavior of H-bridges to set lateral fans forward or backward individually

  Inputs:
  motor index (0,1,2), direction

  Outputs/Effects:
  controls direction of lateral fan rotation
*/

void set_lateral_direction(int16_t motor, lateralDirection direc) {
  switch (motor) {
    case 0:
      // Left motor
      if (direc == BACKWARD) {
        GPIOD_PDOR = (GPIOD_PDOR & ~0x90) | (0x80);
      } else if (direc == FORWARD) {
        GPIOD_PDOR = (GPIOD_PDOR & ~0x90) | (0x10);
      }
      break;
    case 1:
      // right motor
      if (direc == BACKWARD) {
        GPIOD_PDOR = (GPIOD_PDOR & ~0xC) | (0x04);
      } else if (direc == FORWARD) {
        GPIOD_PDOR = (GPIOD_PDOR & ~0xC) | (0x08);
      }
      break;
    case 2:
      // back motor
      if (direc == BACKWARD) {
        GPIOD_PDOR = (GPIOD_PDOR & ~0x3) | (0x01);
      } else if (direc == FORWARD) {
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
    // Pair 3
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

/*
  Function Name: compute_chassis_motion

  Takes accumulated slip values adx and ady, converts into the X,Y, and Theta motion,
  and fills these values into the motion array

  Inputs:
  Accumulated slip values in x and y = adx and ady, motion is external we're modyfying

  Outputs/Effects:
  Sets motion array equal to the motion in X, Y, and Theta
*/
void compute_chassis_motion(int32_t adx[NUM_CAMERAS], int32_t ady[NUM_CAMERAS], float motion[NUM_CAMERAS]) {

  // motion[0] = X motion
  motion[0] = xCoef[0] + (xCoef[1] * adx[0]) + (xCoef[2] * ady[0]) + (xCoef[3] * adx[1]) + (xCoef[4] * ady[1]) + (xCoef[5] * adx[2]) + (xCoef[6] * ady[2]);

  // motion[1] = Y Motion
  motion[1] = yCoef[0] + (yCoef[1] * adx[0]) + (yCoef[2] * ady[0]) + (yCoef[3] * adx[1]) + (yCoef[4] * ady[1]) + (yCoef[5] * adx[2]) + (yCoef[6] * ady[2]);

  // motion[2] = Theta Motion
  motion[2] = tCoef[0] + (tCoef[1] * adx[0]) + (tCoef[2] * ady[0]) + (tCoef[3] * adx[1]) + (tCoef[4] * ady[1]) + (tCoef[5] * adx[2]) + (tCoef[6] * ady[2]);

};

/*
  Function Name: read_distance

  Calculate the calibraterd distance the sensor is from object
  based on the value received from sensor

  Inputs:
   DistanceSensor side: an enum condition referring to either the
   left or the right sensor.

  Outputs/Effects
   distance: float
   Return the calibrated distance as a float in units of centimeters.
*/


float read_distance(DistanceSensor side) {
  float distance = 0;
  int sensorVal = 0;
  float realDistance = 0;

  if (side == DISTANCE_FRONT) {
    sensorVal = analogRead(leftSensor);
    realDistance = (5636 / (sensorVal - 74.643)); //calculate distance with best fit line
  }
  else if (side == DISTANCE_RIGHT) {
    sensorVal = analogRead(rightSensor);
    realDistance = (5636 / (sensorVal - 74.643)); //calculate distance with best fit line
  } // close if left or right

  if (realDistance > 80 || realDistance < 0) { // calculated distance > 80cm or negative
    distance = 80; // truncate distance to 80cm
  }
  else if (realDistance < 80) {
    distance = realDistance;
  }

  // Return calibrated distance in units of cm
  return distance;

} // close function


//////////////////////// Single sensing & control step /////////////////////////

//////////// Note: delays are allowed in this function (or any function called by this function) ///////////
//


/*
  Function Name: control_step

  Controls the execution and timing of events: ramp up, control law, and ramp down. Control Law
  calls functions to orient the craft and correct for error in heading.

  Outputs/Effects
  Corrects heading and updates IMU. Also ramps up the craft and ramps down.
*/

void control_step()
{
  // Update the state of the IMU
  imu_update();

  rotation_rate = IMU.gz * 10; // set rotation rate
  display_heading_velocity(rotation_rate); // display the heading velocity


  // Increment the counter (1 per 10 ms), counts here to keep in magnitudes of 10ms and used in fsm_step
  counter++;

  // compute ddx and ddy
  float ddx = KLv * (velocity_goal[0] - velocity_filtered[0]);
  float ddy = KLv * (velocity_goal[1] - velocity_filtered[1]);

  theta_error = compute_rotation_error(theta_goal, IMU.yaw); // calculate theta error
  display_heading(theta_error * 10); // set the heading display
  float modified_error = deadband_and_saturation(theta_error, deadband, saturation); // calculate the error with deadband/saturation
  float ddtheta = (Kp * modified_error) + (-Kv * rotation_rate); // set dd theta with both errors taken into account
  set_hovercraft_acceleration(ddx, ddy, ddtheta); // set acceleration of hovercraft

}




/*
  Function Name: sensor_step

  Description:
  Checks to see if user wants to calibrate the IMU by pressing the c key.

  Inputs:
  'c' key on keyboard

  Effects:
  Walks the user through a calibration process via serial monitor print statements
*/

void sensor_step() {
  imu_update();
  // Calls function to calculate front sensor distance
  float frontDistance = read_distance(DISTANCE_FRONT);
  // Calls function to calculate right sensor distance
  float rightDistance = read_distance(DISTANCE_RIGHT);

  // Stores the calibrated distances in a global array
  calibrated_distances[0] = frontDistance;
  calibrated_distances[1] = rightDistance;


  // If user enters char 'c' into serial input, calibrate compass
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 0x63) {
      imu_calibrate_magbias();
    }
  }
}

/*
  Function Name: report_step

  Description:
  prints the value that indicates the craft's orientation "yaw"
*/

void report_step() {
  float yaw = IMU.yaw;
  Serial.printf("yaw: %f\n", yaw);
}

/*
  Function Name: camera_step

  inputs: none

  outputs: sets the velocity filtered array to be used in control_step

  Description:
  Compute the distances traveled in the hovercraft coordinate frame during the last 5ms.
  Compute a low-pass filtered estimate of the hovercraft velocity.
*/

void camera_step() {
  // Initialize values that the three cameras record
  static int8_t dx, dy;
  static uint8_t quality;
  const float dt = 0.005;
  const float tau = 0.05;

  // Query cameras 1,2, and 3 (index 0, 1, and 2) for X and Y slip and image quality
  // Returns -1 = overflow, -2 = no slip, or 0 = slip
  for (int i = 0; i < NUM_CAMERAS; i++) {
    int result = cameras.readSlip(CAMERA_SELECT[i], dx, dy, quality);

    switch (result) {
      case -1 :
        //Serial.printf("Overflow\n");
        break;
      case -2 :
        //Serial.printf("No slip\n");
        break;
      case 0 :
        //Serial.printf("SLIP \n");
        adx[i] = dx; //sets slip array to slip values
        ady[i] = dy;
        qualityAr[i] = quality;

        break;
    }
  }
  compute_chassis_motion(adx, ady, motion);

  for (int i = 0; i < NUM_CAMERAS; i++) {
    velocity_filtered[i] = velocity_filtered[i] * ((1 - (dt / tau)) + motion[i]) / tau;
  }
}

/*
  Function Name: fsm_step

  inputs: none

  outputs: Sets theta_goal, and velocity_goal to be used by control step

  Description:
  Sequence the steps for the FSM.
*/

void fsm_step() {
  static State state = STATE_START; // Initial State

  // manages the states
  switch (state) {
    case STATE_START:
      theta_goal = IMU.yaw;     // stay!
      if (GPIOE_PDIR & 0x4000000) {
        // switched pressed - Go!
        state = STATE_SET_ORIENTATION;
      }

      break;
    case STATE_SET_ORIENTATION:
      theta_goal = IMU.yaw;     // record orientation!
      counter = 0;              // reset counter
      state = STATE_RAMP_LIFT;  // set next state
      break;
    case STATE_RAMP_LIFT:
      // counter is being counted through control step, every 10ms
      if (counter > 500) {
        // 5 Seconds.. Start ramping up
        lift_magnitude = counter - 500; // account for counter starting at 500
        set_lift_motor_magnitude(lift_magnitude); // ramp up lift motor
        if (abs(rotation_rate) > 45) {
          // its hovering!
          velocity_goal[0] = 0.1; // start moving forward
          state = STATE_FORWARD; // next state
        }
      }
      break;
    case STATE_FORWARD:
      if (calibrated_distances[0] < 40) {
        // Stop! at wall
        velocity_goal[0] = 0.0; // stop
        set_lift_motor_magnitude(lift_magnitude - 100); // lower motor

        // determine next state to go to
        switch (wall_num) {
          case 1: //  at first wall
            wall_num += 1; // increment wall num since this was reached
            state = STATE_STOPPING;
            set_lift_motor_magnitude(lift_magnitude); // go back to set magnitude
            break;
          case 2: // at second wall
            velocity_goal[0] = 0; // set velocity goal to zero
            state = STATE_STOPPED;
            break;
        }
      }
      break;
    case STATE_STOPPING:
      // set new theta goal
      theta_goal = theta_goal + 85;
      // correct to keep in bounds
      if (theta_goal <= -180.0) {
        theta_goal += 360;
      } else if (theta_goal >= 180) {
        theta_goal -= 360;
      }
      state = STATE_TURN; // go to next state
      break;
    case STATE_TURN:
      if (abs(theta_error) < 1) {
        // desired rotation reached!
        velocity_goal[0] = 0.1; // start moving forward
        state = STATE_FORWARD; // forward state
      }
      break;
    case STATE_STOPPED:
      // Done!
      theta_goal = IMU.yaw;     // stay!
      set_lift_motor_magnitude(0); // set lift motor to 0
      break;
  }

}


/*
  Function Name: compute_rotation_error

  Description:
  compares orientation to target orientation and calculates the error

  Effects:
  Returns the calculated rotational error
*/

float compute_rotation_error(float theta_goal, float theta) {
  float err = theta - theta_goal;
  if (err < -180) {
    err += 360;
  } else if ( err > 180) {
    err -= 360;
  }
  return err;
}

/*
  Function Name:
  deadband_and_saturation

  Inputs:
  error, deadband, saturation

  Effects:
  modifies the error to allow deadband tolerance and saturation values
*/

float deadband_and_saturation(float error, float deadband, float saturation) {
  int sign = 1;
  // captures sign of eror
  if (error < 0) {
    sign = -1;
    error *= -1;
  }

  // returns error
  if (error < deadband) {
    return 0;
  }
  if (error > saturation) {
    return sign * (saturation - deadband);
  }
  return sign * (error - deadband);

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
  // Check to see if it is time to execute the sensor_task
  sensor_task.step();
  // Check to see if it is time to execute the camera_task
  camera_task.step();
  // Check to see if it is time to execute the fsm_task
  fsm_task.step();
  // Check to see if it is time to execute the control_task
  control_task.step();
  // Check to see if it is time to execute the report_task
  report_task.step();

} // close void loop
