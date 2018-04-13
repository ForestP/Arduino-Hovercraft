#include "OpticalFlowCamera.h"

/*
 * ADNS3080 Interface Class
 * 
 * Supports multiple cameras, all wired in parallel to an SPI interface
 * 
 * @author Adapted by Andrew H. Fagg
 * 
 ***************************************
 * Wiring: 
 * 
 * This code uses the SPI0 interface on the Teensy (all cameras are connected to these pins)
 * - MISO -> MISO0 PIN 12
 * - MOSI -> MOSI0 PIN 11
 * - SCLK -> SCK0 PIN 13
 * 
 * In addition:
 * - Each camera requires GND and +5V
 * - Each camera receives a unique select signal: NCS -> choose your pin(s)
 * - Each camera receives a common Reset control pin: RST -> choose your pin 
 * 
 * 
 * 
 * ***************************************
 *  References:
 *  http://ardupilot.org/copter/docs/common-mouse-based-optical-flow-sensor-adns3080.html
 *  https://github.com/Lauszus/ADNS3080
 *  https://www.arduino.cc/en/Reference/SPI
 *  https://www.pjrc.com/teensy/pinout.html
 *  https://www.pjrc.com/teensy/td_libs_SPI.html
 *  
 *****************************************
 *  This code was adapted from the work of Kristian Sloth Lauszus and Mustafa Ghazi
 *  
 *  The original license (Lauszus) is as follows:
 *  
 *  Copyright (C) 2015 Kristian Sloth Lauszus. All rights reserved.
 *  Based on the code by Randy Mackay. DIYDrones.com
 *  This code is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  This code is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with this code.  If not, see <http://www.gnu.org/licenses/>.
 *  Contact information
 *   -------------------
 *  Kristian Sloth Lauszus
 *  Web      :  http://www.lauszus.com
 *  e-mail   :  lauszus@gmail.com
 */


 /**
  * Constructor
  * 
  * Initializes the SPI interface and resets all cameras connected
  * to the specified RST pin
  * 
  * @param resetPin is the Arduino pin number used as a common reset signal across
  * all cameras (RST)
  * 
  * 
  */
OpticalFlowCamera::OpticalFlowCamera(uint8_t resetPin)
{
  // Remember the reset pin index
  this->resetPin = resetPin;

  // Configure the SPI communication settings
  this->spiSettings = SPISettings(2e6, MSBFIRST, SPI_MODE3); // 2 MHz, mode 3

  // Initialize the SPI interface
  SPI.begin();

  // Configure the reset pin as an output
  pinMode(this->resetPin, OUTPUT);

  // Reset all of the cameras on the bus
  reset();
}

/**
 * Reset all of the cameras
 * 
 * TODO: should this be public?
 * 
 */
void OpticalFlowCamera::reset(void)
{
  digitalWrite(this->resetPin, HIGH); // Set high
  delayMicroseconds(10);
  digitalWrite(this->resetPin, LOW); // Set low
  delayMicroseconds(500); // Wait for sensor to get ready
}

/**
 * Add a camera to the set of cameras using the SPI interface.
 * 
 * In particular, the camera's select line pin is configured.
 * 
 * @param selectPin Arduino pin index used to select the new camera
 * 
 * @return 0 if camera has been successfully initialized and detected; and
 *         -1 if a camera has not been detected
 */
int OpticalFlowCamera::addCamera(uint8_t selectPin)
{
  // Configure the select pin as an output & set it to HIGH
  pinMode(selectPin, OUTPUT);
  digitalWrite(selectPin, HIGH);

  // Query the camera
  uint8_t id = spiRead(selectPin, ADNS3080_PRODUCT_ID);

  // Did we find a camera?
  if (id == ADNS3080_PRODUCT_ID_VALUE) 
  {
    // Camera recognized: all okay
    return(0);
  }
  else
  {
    // Error
    return(-1); 
  }
}

/**
 * Read the slip signal from the specified camera
 * 
 * @param selectPin The Arduino pin index for the camera's select pin 
 * @param &dx Number of pixels the image has moved by in the X dimension since 
 * the last call to this function (filled in by this function) 
 * @param &dy Number of pixels the image has moved by in the Y dimension
 * @param &surfaceQuality A quality score for the surface (0 = poor)
 * 
 * @return 0 if no problems and dx/dy/surfaceQuality are valid and can be used;
 *         -1 if an overflow has occurred (the change is larger than can be 
 *         represented in an int8_t; your program should be calling this function quicker);
 *         -2 if an error has occurred, for example, if the surface quality is too low
 */
int OpticalFlowCamera::readSlip(uint8_t selectPin, int8_t &dx, int8_t &dy, uint8_t &surfaceQuality)
{
  // Read sensor
  uint8_t buf[4];
  spiRead(selectPin, ADNS3080_MOTION_BURST, buf, 4);

  // First byte is the status byte
  uint8_t motion = buf[0];
  
  if(motion & 0x10) {
    // Overflow
    return(-1);
  }else if(motion & 0x80) {
    // Good data
    dx = buf[1];
    dy = buf[2];
    surfaceQuality = buf[3];
    return(0);
  }else {
    // Not a slip packet or an error
    return(-2);
  }
  
}

/////////////////////////////////////////////////
// Private functions

/**
 * Write a single byte of data to a camera.
 * 
 * @param selectPin The Arduino pin index for the camera
 * @param reg Command byte
 * @param data Byte to send
 * 
 */
void OpticalFlowCamera::spiWrite(uint8_t selectPin, uint8_t reg, uint8_t data) {
  spiWrite(selectPin, reg, &data, 1);
}

/**
 * Write a sequence of bytes to a camera
 * 
 * @param selectPin The Arduino pin index for the camera
 * @param reg Command byte
 * @param *data Array of bytes to send
 * @param length Number of bytes to send
 */
void OpticalFlowCamera::spiWrite(uint8_t selectPin, uint8_t reg, uint8_t *data, uint8_t length) {
  // Start the SPI transaction
  SPI.beginTransaction(spiSettings);

  // Select the specified camera
  digitalWrite(selectPin, LOW);

  // Send the command byte
  SPI.transfer(reg | 0x80); // Indicate write operation
  delayMicroseconds(75); // Wait minimum 75 us in case writing to Motion or Motion_Burst registers
  SPI.transfer(data, length); // Write data

  // Deselect the camera
  digitalWrite(selectPin, HIGH);

  // Complete the transaction
  SPI.endTransaction();
}

uint8_t OpticalFlowCamera::spiRead(uint8_t selectPin, uint8_t reg) {
  uint8_t buf;
  spiRead(selectPin, reg, &buf, 1);
  return buf;
}

/**
 * Send a command byte to a camera and then read the specified number of bytes
 * 
 * @param selectPin The Arduino pin index for the camera
 * @param reg Command byte
 * @param *data Array of bytes to read
 * @param length Number of bytes to read
 */
void OpticalFlowCamera::spiRead(uint8_t selectPin, uint8_t reg, uint8_t *data, uint8_t length) {
  // Start the SPI transaction
  SPI.beginTransaction(spiSettings);
  // Select the camera
  digitalWrite(selectPin, LOW);

  SPI.transfer(reg); // Send register address
  delayMicroseconds(75); // Wait minimum 75 us in case writing to Motion or Motion_Burst registers
  memset(data, 0, length); // Make sure data buffer is 0
  SPI.transfer(data, length); // Write data

  // Deselect the camera
  digitalWrite(selectPin, HIGH);

  // Complete the transaction
  SPI.endTransaction();
}
