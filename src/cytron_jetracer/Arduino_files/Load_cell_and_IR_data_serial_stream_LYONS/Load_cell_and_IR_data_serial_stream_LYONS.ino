/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------
*/

/*
   Settling time (number of samples) and data filtering can be adjusted in the config.h file
   For calibration and storing the calibration value in eeprom, see example file "Calibration.ino"

   The update() function checks for new data and starts the next conversion. In order to acheive maximum effective
   sample rate, update() should be called at least as often as the HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS.
   If you have other time consuming code running (i.e. a graphical LCD), consider calling update() from an interrupt routine,
   see example file "Read_1x_load_cell_interrupt_driven.ino".

   This is an example sketch on how to use this library
*/

#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

//load cell related pins:
const int HX711_dout = 5; //mcu > HX711 dout pin
const int HX711_sck = 4; //mcu > HX711 sck pin

//velocity measurement related
const int IR_sensor_power_pin = 3;
const int IR_sensor_interrupt_pin = 2;


//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;


//velocity reading related
// Changing the constant below determines the time between velocity measurements
double dt = 0.05;
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
unsigned long time_now = 0;

// Variables for converting detections to velocities
double conversion = 0.041291804;   // Conversion from gear and from wheel rotation to wheel motion
unsigned long IR_conversion_period = pow(10,9) / 20;   // For integer division storage multiplied by 10^9

// Variables for storing the velocities
double IR_vel = 0;
unsigned long period_vel = 0;
unsigned long filter_vel = 0;

double detections_2_velocity_factor = 2 * 3.141593 / 20 * dt * conversion; // this is 2pi/n_partitions on wheel * gear ratio * Radius of wheel [m] so it converts detections to m/s









void setup() {
  Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Starting...");

  LoadCell.begin();
  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  float calibrationValue; // calibration value (see example file "Calibration.ino")
  calibrationValue = 696.0; // uncomment this if you want to set the calibration value in the sketch
#if defined(ESP8266)|| defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266/ESP32 and want to fetch the calibration value from eeprom
#endif
  //EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom

  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }

  //Velocity measurement related
  //pinMode(IR_sensor_interrupt_pin, INPUT_PULLUP);
  pinMode(IR_sensor_power_pin, OUTPUT);
  digitalWrite(IR_sensor_power_pin, HIGH);
  // Attach an interrupt function to pin 2 for the IR sensor, call this function when the state changes on the pin
  attachInterrupt(digitalPinToInterrupt(IR_sensor_interrupt_pin), Pulse_Event_IR, CHANGE);


  
}

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      //Serial.print("Load_cell output val: ");
      //Serial.println(i);
      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  // check if last tare operation is complete:
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }

  // Velocity reading related
  // Reset the timer for the sampling time
  time_now = micros();

  // Convert incremented detections to velocities
  IR_vel = detections_2_velocity_factor * float(i_IR);  // Velocity is multiplied by 1000 (inside conversion factor)
  
  // reset counter to 0
  char IR_detections_char[8];
  dtostrf(i_IR, 6, 0, IR_detections_char); // Leave room for too large numbers!
  i_IR = 0;
  
  // Send the velocities over serial connection
  float load = LoadCell.getData();
  char load_char[8]; // Buffer big enough for 7-character float
  char IR_vel_char[8]; // Buffer big enough for 7-character float
  dtostrf(load, 6, 2, load_char); // Leave room for too large numbers!
  dtostrf(IR_vel, 6, 2, IR_vel_char); // Leave room for too large numbers!
  
  char out_str[100];
  strcpy(out_str, "Load");
  strcat(out_str, load_char);
  strcat(out_str, "Detections");
  strcat(out_str, IR_detections_char);


  
  Serial.println(out_str);
  //Serial.print("Velocity output val: ");
  //Serial.println(IR_vel);
  //Serial.flush();

  // Run while loop for remaining time
  while( micros() < time_now + period)
  {
    // Wait until the sampling period 
  }
  


}



///////////////////////////////////
// Functions for the interrupt pins

// Run this function when pin 2 (IR sensor) is interrupted
void Pulse_Event_IR()
{  
  i_IR++; //so just increment the number of detections
}
