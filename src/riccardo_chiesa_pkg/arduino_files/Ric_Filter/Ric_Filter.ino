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

// define frequency of data output
double dt = 0.075;
unsigned long time_now = 0; // updated later inside loop


#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
//MPU6050 accelgyro(0x68, &Wire1); // <-- use for AD0 low, but 2nd Wire (TWI/I2C) object

int16_t ax, ay, az;
int16_t gx, gy, gz;
float ax_float, ay_float, gz_float;
float IMU_acc_conversion = 9.81 / 16384 ; // comes out in meters per second
float IMU_gyro_conversion = 0.007633; //taken from datasheet(1 / 131) comes out in derees per second














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
#include <BasicLinearAlgebra.h>
#define LTC2992_I2C_ADDRESS 0x6F

using namespace BLA;


char menu_1_continuous_mode(int8_t CTRLA_mode,  int8_t bit_resolution, float scale);


/* void store_alert_settings();        // Store the alert settings to the EEPROM
int8_t restore_alert_settings();    // Read the alert settings from EEPROM */

//#define CONTINUOUS_MODE_DISPLAY_DELAY 100                  //!< frequency of output

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




//velocity reading related
// Changing the constant below determines the time between velocity measurements

// Constant below determines how much % of new velocity is used to update the filtered old velocity
// High value (100) means the new velocity measurement is the filtered velocity, thus no filtering is done
// Low value (1) means it takes a long time for new measurements to change the filtered velocity
unsigned long filter_weight = 50;

// Creating global variables for the detection pins, these need to be volatile
volatile unsigned long i_IR = 0; 
volatile unsigned long IR_LastTimeWeMeasured = micros();
volatile unsigned long IR_SumPeriods = 0;

// Variables for storing the incremented detection values, so they are not changed during calculations
unsigned long IR_store = 0;
unsigned long IR_SumPeriods_store = 0;

// Variables for timing the loop period
unsigned long period = dt * pow(10,6);       // conversion from seconds to microseconds


// Variables for converting detections to velocities
//double conversion = 0.041291804;   // Conversion from gear and from wheel rotation to wheel motion
//unsigned long IR_conversion_period = pow(10,9) / 20;   // For integer division storage multiplied by 10^9
double conversion_n_to_m = 0.002063;   // Conversion from gear and from wheel rotation to wheel motion -> 2pi/(ndetections_per_revolution(20))*Tau_differential(11/39)*R_wheel(0.0233)

// Variables for storing the velocities
double IR_vel = 0;
//unsigned long period_vel = 0;
//unsigned long filter_vel = 0;

//double detections_2_velocity_factor = 2 * 3.141593 / 20 * dt * conversion; // this is 2pi/n_partitions on wheel * gear ratio * Radius of wheel [m] so it converts detections to m/s
double detections_2_velocity_factor = conversion_n_to_m / dt;

// Variables to initialize the filter
volatile float theta_pre_pre = 0 ;
volatile float theta_pre = 0 ;
volatile float time_pre_pre = 0 ;
volatile float time_pre = 0 ;
int N = 2 ; // degree of the fitting polynomial
float theta_i ;
float time_i ;
float vel ;
BLA::Matrix <3> abc ;
BLA::Matrix<3, 3> A;
BLA::Matrix<3> theta;

// string publishing related global variables
String current_str;
String voltage_str;
String ax_str;
String ay_str;
String gz_str;
String vel_str;

// variables need to set a fixed frequency
 



//! Initialize sensors
void setup()
{
  // setting the pins----------
  // IMU power supply
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  // IR sensor
  pinMode(7, OUTPUT);     //for power
  digitalWrite(7, HIGH);
  attachInterrupt(digitalPinToInterrupt(2), Pulse_Event_IR, CHANGE); // setting interrupt pin to perform operations really quick on trigger event
  //----------------------------

  //IMU related ----------------
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize IMU 
    accelgyro.initialize();
   //---------------------------

  // Battery state related------
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
  //----------------------------


  

  // v.Fill(0);
  // v(2,0) = 0
  
  
}

//! Repeats Linduino loop
void loop()
{ 
  //start timer
  time_now = micros();
  
  //IMU sensor-----------------------------------------------------------
  // read raw accel/gyro measurements from IMU device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // convert to m/s^2 and degrees per second respectively
  ax_float = float(ax) * IMU_acc_conversion;
  ay_float = float(ay) * IMU_acc_conversion;
  gz_float = float(gz) * IMU_gyro_conversion;

  
  ax_str = String(ax_float);
  ay_str = String(ay_float);
  gz_str = String(gz_float);

  // ------------------------------------------------------------------------
  
  
  // IR velocity measure ---------------------------------------------------

  // Convert incremented detections to velocities
  IR_vel = detections_2_velocity_factor * float(i_IR);
  // Conver to string in order to send the velocities over serial connection

  vel = 2*abc(0)*time_i + abc(1) ;
  
    
  //vel_str = String(IR_vel);
  vel_str = String(vel);
  
  // reset counter to 0 (it is incremented by the interrupt pin)
  i_IR = 0;
  
  
  //char IR_detections_char[8];
  //dtostrf(i_IR, 6, 0, IR_detections_char); // Leave room for too large numbers!

 
  //-------------------------------------------------------------------------
  
  
  
  
  
  
  //Battery state related ---------------------------------------------------
  //! I2C acknowledge indicator
  static int8_t CTRLA_mode = 0x00;              //! CTRLA Register Setting Default.
  static int8_t bit_resolution = 1;             //! Variable to select ADC Resolution. 1 = 12-bit, 0 = 8-bit. Settable in Settings.
  static float scale = (150/3);                 //! Stores division ratio for resistive divider on GPIO pin.  Configured inside "Settings" menu.
  
  //char out_str = "abc"; 
  //out_str = menu_1_continuous_mode(CTRLA_mode, bit_resolution, scale);  //! Continuous Mode


  //Battery sensor (used to be in a function)
  int8_t LTC2992_mode;
  int8_t ack = 0;
  CTRLA_mode = ((CTRLA_mode & LTC2992_CTRLA_MEASUREMENT_MODE_MASK) | (LTC2992_MODE_CONTINUOUS & ~LTC2992_CTRLA_MEASUREMENT_MODE_MASK));
  ack |= LTC2992_write(LTC2992_I2C_ADDRESS, LTC2992_CTRLA_REG, LTC2992_mode); //! Sets the LTC2992 to continuous mode

  uint32_t power1_code, power2_code, max_power1_code, max_power2_code, min_power1_code, min_power2_code;

  uint16_t current1_code, current2_code, max_current1_code, max_current2_code, min_current1_code, min_current2_code;
  ack |= LTC2992_read_12_bits(LTC2992_I2C_ADDRESS, LTC2992_DELTA_SENSE1_MSB_REG, & current1_code);


  float current1, current2, max_current1, max_current2, min_current1, min_current2;
  current1 = LTC2992_code_to_current(current1_code, resistor, LTC2992_DELTA_SENSE_12bit_lsb);


  uint16_t SENSE1_code, max_SENSE1_code, min_SENSE1_code;
  ack |= LTC2992_read_12_bits(LTC2992_I2C_ADDRESS, LTC2992_SENSE1_MSB_REG, & SENSE1_code);


  float SENSE1, max_SENSE1, min_SENSE1;
  SENSE1 = LTC2992_SENSE_code_to_voltage(SENSE1_code, LTC2992_SENSE_12bit_lsb);
  //strings to sendo over serial
  current_str = String(current1);
  voltage_str = String(SENSE1);
  //-------------------------------------------------------------------------




  //send out string through serial ------------------------------------------
  Serial.println("Cur" + current_str + "Vol" + voltage_str + "Acc_x" + ax_str + "Acc_y"+ ay_str + "Gyr_z" + gz_str + "Vel" + vel_str);
  //-------------------------------------------------------------------------

  // wait to keep steady output frequency
  //delay(100); // setting it to 50 seems like the loop doesn't close for the battery state readings amd they don't get updated
  
  // Run while loop for remaining time
  while( micros() < time_now + period)
  {
    // Wait until the sampling period 
  }
}


 
// Interrupt pins
// Run this function when pin 2 (IR sensor) is interrupted
void Pulse_Event_IR()
{  
  i_IR++; //so just increment the number of detections
  
  theta_i = i_IR*3.141593/20 ;
  
  time_i = micros() / pow(10,6) ;

  theta.Fill(0) ;
  theta(0,0) = theta_pre_pre ;
  theta(1,0) = theta_pre ;
  theta(2,0) = theta_i ;
  
  A = { pow(time_pre_pre,2), time_pre_pre, 1,
        pow(time_pre,2),     time_pre,     1,
        pow(time_i,2),       time_i,       1 } ;
        
  BLA::Matrix <3, 3> inv_A = A ;
  bool is_nonsingular = Invert(inv_A) ;
  abc = inv_A * theta ;
  
  theta_pre_pre = theta_pre;
  theta_pre = theta_i;
  time_pre_pre = time_pre;
  time_pre = time_i;
  
  
  
}
