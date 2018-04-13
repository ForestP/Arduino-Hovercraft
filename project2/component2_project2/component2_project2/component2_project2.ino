/*
  Project: Project 2
  Group: 11
  Date: 2017-02-20

  Authors: Forest Plasencia, Blake Herren, Jacob Burcham
  
  Responsible author: Jacob Burcham
  
*/

// Initialize analog port as a global variable
const int analog_1 = 7; // Left
const int analog_2 = 8; // Right

// Declare enum state for each sensor as a global variable
typedef enum {
    // Set DISTANCE_LEFT to port 7
   DISTANCE_LEFT = 7,
   // Set DISTANCE_LEFT to port 8
   DISTANCE_RIGHT = 8
}DistanceSensor;

/* 
Function Name: setup
   
Initialize serial port on Teensy

Inputs:
  none

Outputs/Effects
  Initialize inputs and outputs
  
*/

void setup() {

  // Initialize serial port
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
  if (side == DISTANCE_LEFT) {
    int sensorVal = analogRead(side);
    // calculate distance
    distance = (4947.6)/(sensorVal - 40.72);
    // set distance to 80 if sensor reads higher
    if (distance > 80 || distance < 0) {
      distance = 80;
    }
  } else if (side == DISTANCE_RIGHT) {
    int sensorVal = analogRead(side);
    // calculate distance
    distance = (4624.6)/(sensorVal - 31.694);
    // set distance to 80 if sensor reads higher
    if (distance > 80) {
      distance = 80;
    }
  }

  // Return calibrated distance in units of cm
  return distance;
  
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

  // Calls function to calculate left sensor distance
  float leftDistance = read_distance(DISTANCE_LEFT);
  // Calls function to calculate right sensor distance
  float rightDistance = read_distance(DISTANCE_RIGHT);

  // Prints out each sensor distance on the same line
  Serial.print("Left: ");
  Serial.print(leftDistance);
  Serial.print(" Right: ");
  Serial.println(rightDistance);
  
  delay(100);

}
