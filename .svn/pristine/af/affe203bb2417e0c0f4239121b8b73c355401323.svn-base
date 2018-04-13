/*
  Project: Project 3
  Group: 11
  Date: 2017-03-02

  Authors: Forest Plasencia, Blake Herren, Jacob Burcham
  
  Responsible author: Blake Herren
*/

// Declare enum state for each sensor as a global variable

#include "project.h"
#include "OpticalFlowCamera.h"

////////////////////////////////////////////////////
// Global constants

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
const float yCoef[7] = {
  -0.1066484176,
  0.0000232153,
  0.0000143951,
  0.0001553162,
  -0.0000371591,
  -0.0002395083,
  0.0001435592
};

// Array of theta coefficients from linear regression of raw movement data
const float tCoef[7] = {
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
  
  PORTE_PCR24 = PORT_PCR_MUX(0x1);
  PORTE_PCR25 = PORT_PCR_MUX(0x1);
  PORTC_PCR8 = PORT_PCR_MUX(0x1);
  PORTC_PCR9 = PORT_PCR_MUX(0x1);

  // Initialize pins for cameras
  GPIOE_PDDR = 0x3000000;
  GPIOC_PDDR = 0x300;

  
   //Initialize each of the cameras
  for (int i = 0; i < NUM_CAMERAS; i++) {
    int ret = cameras.addCamera(CAMERA_SELECT[i]);
    Serial.print(ret);
  }
}


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
  
  if (side == DISTANCE_LEFT) {
    sensorVal = analogRead(leftSensor); 
    realDistance = (5636/(sensorVal - 74.643)); //calculate distance with best fit line
  }  
    else if (side == DISTANCE_RIGHT) {
      sensorVal = analogRead(rightSensor);
      realDistance = (5636/(sensorVal - 74.643)); //calculate distance with best fit line
      } // close if left or right
   
  if (realDistance > 80 || realDistance < 0) { // calculated distance > 80cm or negative
        distance = 80; // truncate distance to 80cm
      }     
      else if (realDistance < 80){
        distance = realDistance; 
      }

  // Return calibrated distance in units of cm
  return distance;
  
} // close function


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
Function Name: loop
   
loop through main program, getting the calibrated sensor distance
and print each distance out

Inputs:
  none

Outputs/Effects
  Print out the calculated distance of each sensor
*/

// Initialize values that the three cameras record
int8_t dx, dy;
uint8_t quality;
// Initialize count value used to print summed values over time
int8_t count = 0;

void loop() {

// If user enters char 'c' into serial input, set all slip values equal to zero
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 0x63) {
      for (int i = 0; i < NUM_CAMERAS; i++) { 
      adx[i] = 0;
      ady[i] = 0;
      qualityAr[i] =0;
      motion[i] = 0;
      }
      Serial.printf("cleared");
      count = 0;
    }

  } else {

    
    // Calls function to calculate left sensor distance
    float leftDistance = read_distance(DISTANCE_LEFT);
    // Calls function to calculate right sensor distance
    float rightDistance = read_distance(DISTANCE_RIGHT);
    // Prints out each sensor distance on the same line
    //Serial.printf("Left: %f   Right: %f \n", leftDistance, rightDistance);

    // Query cameras 1,2, and 3 (index 0, 1, and 2) for X and Y slip and image quality
    // Returns -1 = overflow, -2 = no slip, or 0 = slip
    for (int i = 0; i < NUM_CAMERAS; i++) {
      int result = cameras.readSlip(CAMERA_SELECT[i], dx, dy, quality);
      
      switch(result) {
        case -1 :
          //Serial.printf("Overflow\n");
          break;
        case -2 :
          //Serial.printf("No slip\n");
          break;
        case 0 :
          //Serial.printf("SLIP \n");
          adx[i] += dx; //sets slip array to slip values
          ady[i] += dy;
          qualityAr[i] += quality;
          compute_chassis_motion(adx, ady, motion);
          break;
      }
    } 
  }

  // Adds 1 to count each loop
  count ++;
  // Once count = 10, Print motion in X, Y, and Theta to serial and resets count value
  if (count == 10) {
    Serial.printf("motion-X: %f  motion-Y: %f theta: %f Raw: %d %d %d %d %d %d %d %d %d \n", motion[0], motion[1],motion[2], adx[0], ady[0], qualityAr[0], adx[1], ady[1], qualityAr[1], adx[2],ady[2], qualityAr[2]); 
    
    count = 0;

  }


  delay(100);
} // close void loop
