#ifndef OPTICAL_FLOW_CAMERA_H
#define OPTICAL_FLOW_CAMERA_H
#include <SPI.h>

/*
 * ADNS3080 Interface Class
 * 
 * Supports multiple cameras, all wired in parallel to an SPI interface
 * 
 * @author Adapted by Andrew H. Fagg
 */
 
class OpticalFlowCamera 
{
  public:
    OpticalFlowCamera(uint8_t resetPin);
    void reset(void);
    int addCamera(uint8_t selectPin);
    int readSlip(uint8_t selectPin, int8_t &dx, int8_t &dy, uint8_t &surfaceQuality);
    
  private:
    SPISettings spiSettings;
    int resetPin;
    void spiWrite(uint8_t selectPin, uint8_t reg, uint8_t data);
    void spiWrite(uint8_t selectPin, uint8_t reg, uint8_t *data, uint8_t length);
    uint8_t spiRead(uint8_t selectPin, uint8_t reg);
    void spiRead(uint8_t selectPin, uint8_t reg, uint8_t *data, uint8_t length);

    // Register Map for the ADNS3080 Optical OpticalFlow Sensor
    static const uint8_t ADNS3080_PRODUCT_ID =          0x00;
    static const uint8_t ADNS3080_MOTION =              0x02;
    static const uint8_t ADNS3080_DELTA_X =             0x03;
    static const uint8_t ADNS3080_DELTA_Y =             0x04;
    static const uint8_t ADNS3080_SQUAL =               0x05;
    static const uint8_t ADNS3080_CONFIGURATION_BITS =  0x0A;
    static const uint8_t ADNS3080_MOTION_CLEAR =        0x12;
    static const uint8_t ADNS3080_FRAME_CAPTURE =       0x13;
    static const uint8_t ADNS3080_MOTION_BURST =        0x50;

    // ADNS3080 hardware config
    static const uint8_t ADNS3080_PIXELS_X =            30;
    static const uint8_t ADNS3080_PIXELS_Y =            30;

    // Id returned by ADNS3080_PRODUCT_ID register
    static const uint8_t ADNS3080_PRODUCT_ID_VALUE =    0x17;
    
};

#endif
