/* 	Voldemort the Mobile Drawing Robot (Morti) v1.0
		A Capstone Project for the Electromechanical Systems Design course 
		Carnegie Mellon University
		December 2018

		Code written by Mecca Parker
		Team Members: Kenny Sladick, Christopher Bright, Rory Hubbard, Kam Undieh

		Voldemort is a mobile drawing robot. Other drawing mechanisms are limited 
		to a single, maximum canvas size. Morti is not.

    Read more: (https://github.com/meccaparker/voldemort) 
*/

#ifndef voldemort.h
#define voldemort.h

#include "Arduino.h"

#include <HardwareSerial.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Encoder.h>
#include <Servo.h> 
#include <SD.h>
#include <math.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <RPLidar.h>
#include <utility/imumaths.h>
#include <MegunoLink.h>
#include <Adafruit_NeoPixel.h>
#include <FastLED.h>
// #include <Reactduino.h>

#ifdef __AVR__
  #include <avr/power.h>
#endif

#include "report.h"
#include "dc.h"
#include "tool.h"
#include "parser.h"
#include "serial.h"
#include "lidar.h"
#include "led.h"

#define CS_PIN 53                     // chipselect for SD card reader

// Success codes
#define SUCCESS_DC_INIT 10            // motors initialized
#define SUCCESS_DC_DISABLE 11         // motors disabled
#define SUCCESS_DC_SET_TARGET 12      // set setpoints for motors
#define SUCCESS_DC_MOVE 13            // dc motors have moved to setpoints

#define SUCCESS_TOOL_ENGAGED 20       // tool down
#define SUCCESS_TOOL_DISENGAGED 21    // tool up
#define SUCCESS_TOOL_INIT 22          // tool initialized

#define SUCCESS_TERMINAL_MODE 30      // terminal mode activated - input commands via serial port
#define SUCCESS_FILE_MODE 31          // file mode activated - reading commands from file "gcode.txt"
#define SUCCESS_FILE_COMPLETE 32      // file with commands successfully read

#define SUCCESS_COM_RECEIVED 41       // GCode command received 
#define SUCCESS_GCODE_PARSE 42        // GCode commmand parsed

#define SUCCESS_VMORT_INIT 50         // Voldemort has initialized himself

// Error codes
#define ERROR_FILE_MODE 30            // unable to read file
#define ERROR_TERMINAL_MODE 31        // serial unreachable for terminal mode
#define ERROR_SD_READ 32              // unable to read SD card

#define ERROR_UNRECOGNIZED_GCODE 40   // uncrecognized GCode received

#define DATA_PIN 11                   // led data pin
#define NUM_LEDS 8   

#endif voldemort.h
