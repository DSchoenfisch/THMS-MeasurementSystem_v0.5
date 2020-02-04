/**************************************************************************/
/*!
    @file     Adafruit_ADS1015_DS.h
    @author   K. Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    This is a library for the Adafruit ADS1015 breakout board
    ----> https://www.adafruit.com/products/???

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0  - First release
    v1.1  - Added ADS1115 support - W. Earl
*/
/**************************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

/*=========================================================================
    I2C ADDRESS/BITS (Ist einstellbar über change_I2C_address() )
    -----------------------------------------------------------------------*/
    #define ADS1015_ADDRESS                 (0x49)    //100 1001 (ADDR = GND)
    #define ADS1015_ADDRESS_GND             (0x48)    //100 1000 
    #define ADS1015_ADDRESS_VDD             (0x49)    //100 1001
    #define ADS1015_ADDRESS_SDA             (0x50)    //100 1010
    #define ADS1015_ADDRESS_SCL             (0x51)    //100 1011
/*=========================================================================*/

/*========D. Schoenfisch===================================================
    NUMBER OF ALLERT PIN
    -----------------------------------------------------------------------*/
    #define ADS1015_ALLERT_PIN              (4)
/*=========================================================================*/

/*=========================================================================
    CONVERSION DELAY (in mS)
    -----------------------------------------------------------------------*/
    #define ADS1015_CONVERSIONDELAY         (1)
    #define ADS1115_CONVERSIONDELAY         (8)
/*=========================================================================*/

/*=========================================================================
    POINTER REGISTER
    -----------------------------------------------------------------------*/
    #define ADS1015_REG_POINTER_MASK        (0x03)
    #define ADS1015_REG_POINTER_CONVERT     (0x00)
    #define ADS1015_REG_POINTER_CONFIG      (0x01)
    #define ADS1015_REG_POINTER_LOWTHRESH   (0x02)
    #define ADS1015_REG_POINTER_HITHRESH    (0x03)
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER
    -----------------------------------------------------------------------*/
    #define ADS1015_REG_CONFIG_OS_MASK      (0x8000)
    #define ADS1015_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion
    #define ADS1015_REG_CONFIG_OS_BUSY      (0x0000)  // Read: Bit = 0 when conversion is in progress
    #define ADS1015_REG_CONFIG_OS_NOTBUSY   (0x8000)  // Read: Bit = 1 when device is not performing a conversion

    #define ADS1015_REG_CONFIG_MUX_MASK     (0x7000)
    #define ADS1015_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)
    #define ADS1015_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3
    #define ADS1015_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3
    #define ADS1015_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3
    #define ADS1015_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
    #define ADS1015_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
    #define ADS1015_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
    #define ADS1015_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3

    #define ADS1015_REG_CONFIG_PGA_MASK     (0x0E00)
    #define ADS1015_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3
    #define ADS1015_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1
    #define ADS1015_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2 (default)
    #define ADS1015_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range = Gain 4
    #define ADS1015_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range = Gain 8
    #define ADS1015_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range = Gain 16

    #define ADS1015_REG_CONFIG_MODE_MASK    (0x0100)
    #define ADS1015_REG_CONFIG_MODE_CONTIN  (0x0000)  // Continuous conversion mode
    #define ADS1015_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)

    #define ADS1015_REG_CONFIG_DR_MASK      (0x00E0)  
    #define ADS1015_REG_CONFIG_DR_128SPS    (0x0000)  // ADS1015: 128  / ADS1115: 8 samples per second
    #define ADS1015_REG_CONFIG_DR_250SPS    (0x0020)  // ADS1015: 250  / ADS1115: 16 samples per second
    #define ADS1015_REG_CONFIG_DR_490SPS    (0x0040)  // ADS1015: 490  / ADS1115: 32 samples per second
    #define ADS1015_REG_CONFIG_DR_920SPS    (0x0060)  // ADS1015: 920  / ADS1115: 64 samples per second
    #define ADS1015_REG_CONFIG_DR_1600SPS   (0x0080)  // ADS1015: 1600 / ADS1115: 128 samples per second (default)
    #define ADS1015_REG_CONFIG_DR_2400SPS   (0x00A0)  // ADS1015: 2400 / ADS1115: 250 samples per second
    #define ADS1015_REG_CONFIG_DR_3300SPS   (0x00C0)  // ADS1015: 3300 / ADS1115: 475 samples per second
    #define ADS1015_REG_CONFIG_DR_MAX_SPS   (0x00E0)  // ADS1015: 3300 / ADS1115: 860 samples per second

    #define ADS1015_REG_CONFIG_CMODE_MASK   (0x0010)
    #define ADS1015_REG_CONFIG_CMODE_TRAD   (0x0000)  // Traditional comparator with hysteresis (default)
    #define ADS1015_REG_CONFIG_CMODE_WINDOW (0x0010)  // Window comparator

    #define ADS1015_REG_CONFIG_CPOL_MASK    (0x0008)
    #define ADS1015_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)
    #define ADS1015_REG_CONFIG_CPOL_ACTVHI  (0x0008)  // ALERT/RDY pin is high when active

    #define ADS1015_REG_CONFIG_CLAT_MASK    (0x0004)  // Determines if ALERT/RDY pin latches once asserted
    #define ADS1015_REG_CONFIG_CLAT_NONLAT  (0x0000)  // Non-latching comparator (default)
    #define ADS1015_REG_CONFIG_CLAT_LATCH   (0x0004)  // Latching comparator

    #define ADS1015_REG_CONFIG_CQUE_MASK    (0x0003)
    #define ADS1015_REG_CONFIG_CQUE_1CONV   (0x0000)  // Assert ALERT/RDY after one conversions
    #define ADS1015_REG_CONFIG_CQUE_2CONV   (0x0001)  // Assert ALERT/RDY after two conversions
    #define ADS1015_REG_CONFIG_CQUE_4CONV   (0x0002)  // Assert ALERT/RDY after four conversions
    #define ADS1015_REG_CONFIG_CQUE_NONE    (0x0003)  // Disable the comparator and put ALERT/RDY in high state (default)
/*=========================================================================*/

typedef enum
{//                                                                             ADS1015  ADS1115
  GAIN_TWOTHIRDS    = ADS1015_REG_CONFIG_PGA_6_144V,    //  +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  GAIN_ONE          = ADS1015_REG_CONFIG_PGA_4_096V,    //  +/- 4.096V  1 bit = 2mV      0.125mV
  GAIN_TWO          = ADS1015_REG_CONFIG_PGA_2_048V,    //  +/- 2.048V  1 bit = 1mV      0.0625mV
  GAIN_FOUR         = ADS1015_REG_CONFIG_PGA_1_024V,    //  +/- 1.024V  1 bit = 0.5mV    0.03125mV
  GAIN_EIGHT        = ADS1015_REG_CONFIG_PGA_0_512V,    //  +/- 0.512V  1 bit = 0.25mV   0.015625mV
  GAIN_SIXTEEN      = ADS1015_REG_CONFIG_PGA_0_256V     //  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
} adsGain_t;

typedef enum      // !!!!!!!!!!!!! MUSS FÜR ADS1115 ANGEPASST WERDEN
{
  DR_1         = ADS1015_REG_CONFIG_DR_128SPS,  // ADS1015: 128  / ADS1115: 8 samples per second
  DR_2         = ADS1015_REG_CONFIG_DR_250SPS,  // ADS1015: 250  / ADS1115: 16 samples per second
  DR_3         = ADS1015_REG_CONFIG_DR_490SPS,  // ADS1015: 490  / ADS1115: 32 samples per second
  DR_4         = ADS1015_REG_CONFIG_DR_920SPS,  // ADS1015: 920  / ADS1115: 64 samples per second
  DR_5         = ADS1015_REG_CONFIG_DR_1600SPS, // ADS1015: 1600 / ADS1115: 128 samples per second (default)
  DR_6         = ADS1015_REG_CONFIG_DR_2400SPS, // ADS1015: 2400 / ADS1115: 250 samples per second
  DR_7         = ADS1015_REG_CONFIG_DR_3300SPS,  // ADS1015: 3300 / ADS1115: 475 samples per second
  DR_8         = ADS1015_REG_CONFIG_DR_MAX_SPS  // ADS1015: 3300 / ADS1115: 860 samples per second
} adsDR_t;

typedef enum    //Author: D.Schoenfisch
{
  ADDR_TO_GND       = ADS1015_ADDRESS_GND, 
  ADDR_TO_VDD       = ADS1015_ADDRESS_VDD, 
  ADDR_TO_SDA       = ADS1015_ADDRESS_SDA, 
  ADDR_TO_SCL       = ADS1015_ADDRESS_SCL
} adsAddr_t;

class Adafruit_ADS1015_DS
{
protected:
   // Instance-specific properties
   uint8_t   m_i2cAddress;
   uint8_t   m_conversionDelay;
   uint8_t   m_bitShift;
   adsDR_t   m_dataRate; //Author: D.Schoenfisch
   adsGain_t m_gain;
   int       m_alertPin; //Author: D.Schoenfisch

public:
  Adafruit_ADS1015_DS(uint8_t i2cAddress = ADS1015_ADDRESS, int alertPin = ADS1015_ALLERT_PIN);
  void      begin(void);
  void      reset_all_ADS(void);
  void      change_I2C_address(uint8_t i2cAddress);
  uint16_t  readADC_SingleEnded(uint8_t channel);
  int16_t   readADC_Differential_0_1(void);
  int16_t   readADC_Differential_2_3(void);
  void      startComparator_SingleEnded(uint8_t channel, int16_t threshold);
  int16_t   getLastConversionResults();
  void      setGain(adsGain_t gain);
  adsGain_t getGain(void);
  void      setDataRate(adsDR_t dataRate); //Author: D.Schoenfisch
  uint8_t   startContinuous_Differential_0_1(); // Author: D. Schoenfisch
  int16_t   readMeasVal_Contiuous(); // Author: D. Schoenfisch
  void      powerDown(); // Author: D. Schoenfisch

 private:
};

// Derive from ADS1105 & override construction to set properties
class Adafruit_ADS1115_DS : public Adafruit_ADS1015_DS
{
 public:
  Adafruit_ADS1115_DS(uint8_t i2cAddress = ADS1015_ADDRESS, int alertPin = ADS1015_ALLERT_PIN);

 private:
};
