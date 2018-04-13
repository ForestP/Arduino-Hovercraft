/*
  Project: Project 2
  Group: 11
  Date: 2017-02-20

  Authors: Forest Plasencia, Blake Herren, Jacob Burcham
  
  Responsible author: Jacob Burcham
*/

// Declare enum state for each sensor as a global variable

#include "project.h"

const int16_t leftSensor = 7;
const int16_t rightSensor = 8;

/* 
Function Name: setup
   
Initialize serial port on Teensy

Inputs:
  none

Outputs/Effects
  Initialize inputs and outputs
*/

void setup() {
  // Initialize serial port & send/recieve @ 9600 bit/s
  Serial.begin(9600);
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
Function Name: loop
   
loop through main program, getting the calibrated sensor distance
and print each distance out

Inputs:
  none

Outputs/Effects
  Print out the calculated distance of each sensor
*/

void loop() {
  // Calls function to calculate left sensor distance
  float leftDistance = read_distance(DISTANCE_LEFT);
  // Calls function to calculate right sensor distance
  float rightDistance = read_distance(DISTANCE_RIGHT);
  // Prints out each sensor distance on the same line
  Serial.printf("Left: %f   Right: %f \n", leftDistance, rightDistance);
  delay(100);
} // close void loop
