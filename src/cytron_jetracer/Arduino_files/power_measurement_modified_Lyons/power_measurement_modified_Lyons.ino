/*!
LTC2992: Dual Wide Range Power Monitor

@verbatim

Setting the Thresholds:
    1. Select the Threshold Configuration from the main menu.
    2. Select the desired Threshold to be changed. Then enter the minimum and maximum
       values.
    3. If any reading exceeds the set threshold, a fault will be generated that can be viewed in
     the Read/Clear Faults Menu


Reading and Clearing a Fault:
    1. Select the Read/Clear Fault option from the main menu.
    2. To read all the faults, select Read Faults option. This will display all the faults that have occured.
  3. To clear all faults, go back to the Read/Clear Faults menu and select Clear Faults option.

NOTE: Due to memory limitations of the Atmega328 processor this sketch shows limited functionality of the LTC2992. Please
      check the datasheet for a picture of the full functionality of the LTC2992.

NOTES
 Setup:
 Set the terminal baud rate to 115200 and select the newline terminator.
 Requires a power supply.
 Refer to demo manual DC2561A.

USER INPUT DATA FORMAT:
 decimal : 1024
 hex     : 0x400
 octal   : 02000  (leGPIOg 0 "zero")
 binary  : B10000000000
 float   : 1024.0

@endverbatim
http://www.linear.com/product/LTC2992

http://www.linear.com/product/LTC2992#demoboards

Copyright 2018(c) Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
 - Neither the name of Analog Devices, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.
 - The use of this software may or may not infringe the patent rights
   of one or more patent holders.  This license does not release you
   from the requirement that you obtain separate licenses from these
   patent holders to use this software.
 - Use of the software either in source or binary form, must be run
   on or directly connected to an Analog Devices Inc. component.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*! @file
    @ingroup LTC2992
*/

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_I2C.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "QuikEval_EEPROM.h"
#include "LTC2992.h"
#include <Wire.h>
#include <SPI.h>

#define LTC2992_I2C_ADDRESS 0x6F


int8_t menu_1_continuous_mode(int8_t CTRLA_mode,  int8_t bit_resolution, float scale);


/* void store_alert_settings();        // Store the alert settings to the EEPROM
int8_t restore_alert_settings();    // Read the alert settings from EEPROM */

#define CONTINUOUS_MODE_DISPLAY_DELAY 50                  //!< The delay between readings in continious mode

// LSB Weights
const float LTC2992_GPIO_8bit_lsb = 8.000E-3;                //!< Typical GPIO lsb weight for 8-bit mode in volts
const float LTC2992_GPIO_12bit_lsb = 0.500E-3;                //!< Typical GPIO lsb weight for 12-bit mode in volts
const float LTC2992_DELTA_SENSE_8bit_lsb = 200.00E-6;        //!< Typical Delta lsb weight for 8-bit mode in volts
const float LTC2992_DELTA_SENSE_12bit_lsb = 12.50E-6;        //!< Typical Delta lsb weight for 12-bit mode in volts
const float LTC2992_SENSE_8bit_lsb = 400.00E-3;                //!< Typical SENSE lsb weight for 8-bit mode in volts
const float LTC2992_SENSE_12bit_lsb = 25.00E-3;                //!< Typical SENSE lsb weight for 12-bit mode in volts
const float LTC2992_Power_8bit_lsb = LTC2992_DELTA_SENSE_8bit_lsb*LTC2992_SENSE_8bit_lsb;     //!< Typical POWER lsb weight for 8-bit mode in V^2
const float LTC2992_Power_12bit_lsb = LTC2992_DELTA_SENSE_12bit_lsb*LTC2992_SENSE_12bit_lsb;     //!< Typical POWER lsb weight for 12-bit mode in V^2

const float resistor = .02;         //!< resistor value on demo board

// Error string
const char ack_error[] = "Error: No Acknowledge. Check I2C Address."; //!< Error message

// Global variables
static int8_t demo_board_connected;        //!< Set to 1 if the board is connected


//! Initialize Linduino
void setup()
{
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  
  char demo_name[] = "DC2561";      // Demo Board Name stored in QuikEval EEPROM

  quikeval_I2C_init();              //! Configure the EEPROM I2C port for 100kHz
  Serial.begin(115200);             //! Initialize the serial port to the PC
  //print_title();
  demo_board_connected = true;
  if (!demo_board_connected)
  {
  //  Serial.println(F("Demo board not detected, will attempt to proceed"));
    demo_board_connected = true;
  }
  if (demo_board_connected)
  {
   // restore_alert_settings();
   // print_prompt();
  }
}

//! Repeats Linduino loop
void loop()
{                              //! I2C acknowledge indicator
  static int8_t CTRLA_mode = 0x00;              //! CTRLA Register Setting Default.
  static int8_t bit_resolution = 1;             //! Variable to select ADC Resolution. 1 = 12-bit, 0 = 8-bit. Settable in Settings.
  static float scale = (150/3);                 //! Stores division ratio for resistive divider on GPIO pin.  Configured inside "Settings" menu.

  if (demo_board_connected)                     //! Do nothing if the demo board is not connected
  {
      menu_1_continuous_mode(CTRLA_mode, bit_resolution, scale);  //! Continuous Mode
  }
}







//! BATTERY SENSOR Continuous Mode.
int8_t menu_1_continuous_mode(int8_t CTRLA_mode, //!< Set Continious Mode
                              int8_t bit_resolution, //!< Set ADC Resolution
                              float scale) //!< Stores division ratio for resistive divider on GPIO pin.  Configured inside "Settings" menu.
//! @return Returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  int8_t LTC2992_mode;
  int8_t ack = 0;
  CTRLA_mode = ((CTRLA_mode & LTC2992_CTRLA_MEASUREMENT_MODE_MASK) | (LTC2992_MODE_CONTINUOUS & ~LTC2992_CTRLA_MEASUREMENT_MODE_MASK));
  Serial.println();
  ack |= LTC2992_write(LTC2992_I2C_ADDRESS, LTC2992_CTRLA_REG, LTC2992_mode); //! Sets the LTC2992 to continuous mode
  do
  {

      uint32_t power1_code, power2_code, max_power1_code, max_power2_code, min_power1_code, min_power2_code;

      uint16_t current1_code, current2_code, max_current1_code, max_current2_code, min_current1_code, min_current2_code;
      ack |= LTC2992_read_12_bits(LTC2992_I2C_ADDRESS, LTC2992_DELTA_SENSE1_MSB_REG, & current1_code);
      ack |= LTC2992_read_12_bits(LTC2992_I2C_ADDRESS, LTC2992_MAX_DELTA1_SENSE_MSB_REG, & max_current1_code);
      ack |= LTC2992_read_12_bits(LTC2992_I2C_ADDRESS, LTC2992_MIN_DELTA1_SENSE_MSB_REG, & min_current1_code);

      ack |= LTC2992_read_12_bits(LTC2992_I2C_ADDRESS, LTC2992_DELTA_SENSE2_MSB_REG, & current2_code);
      ack |= LTC2992_read_12_bits(LTC2992_I2C_ADDRESS, LTC2992_MAX_DELTA2_SENSE_MSB_REG, & max_current2_code);
      ack |= LTC2992_read_12_bits(LTC2992_I2C_ADDRESS, LTC2992_MIN_DELTA2_SENSE_MSB_REG, & min_current2_code);

      float current1, current2, max_current1, max_current2, min_current1, min_current2;
      current1 = LTC2992_code_to_current(current1_code, resistor, LTC2992_DELTA_SENSE_12bit_lsb);
      max_current1 = LTC2992_code_to_current(max_current1_code, resistor, LTC2992_DELTA_SENSE_12bit_lsb);
      min_current1 = LTC2992_code_to_current(min_current1_code, resistor, LTC2992_DELTA_SENSE_12bit_lsb);



      //Serial.print(F("\n    Current 1: "));
      Serial.print(current1, 4);
      //Serial.print(F(" A"));


      uint16_t SENSE1_code, max_SENSE1_code, min_SENSE1_code;
      ack |= LTC2992_read_12_bits(LTC2992_I2C_ADDRESS, LTC2992_SENSE1_MSB_REG, & SENSE1_code);
      ack |= LTC2992_read_12_bits(LTC2992_I2C_ADDRESS, LTC2992_MAX_SENSE1_MSB_REG, & max_SENSE1_code);
      ack |= LTC2992_read_12_bits(LTC2992_I2C_ADDRESS, LTC2992_MIN_SENSE1_MSB_REG, & min_SENSE1_code);

      float SENSE1, max_SENSE1, min_SENSE1;
      SENSE1 = LTC2992_SENSE_code_to_voltage(SENSE1_code, LTC2992_SENSE_12bit_lsb);
      max_SENSE1 = LTC2992_SENSE_code_to_voltage(max_SENSE1_code, LTC2992_SENSE_12bit_lsb);
      min_SENSE1 = LTC2992_SENSE_code_to_voltage(min_SENSE1_code, LTC2992_SENSE_12bit_lsb);


      //Serial.print(F("\n      SENSE 1: "));
      Serial.print(SENSE1, 4);
      //Serial.print(F(" V"));


    Serial.print(F("\n"));
    Serial.flush();
    delay(CONTINUOUS_MODE_DISPLAY_DELAY);
  }
  while (Serial.available() == false);
  read_int(); // clears the Serial.available
  //return (ack);
}
