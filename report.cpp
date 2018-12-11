/* 	Voldemort the Mobile Drawing Robot (Morti) v1.0
		A Capstone Project for the Electromechanical Systems Design course 
		Carnegie Mellon University
		December 2018

		Code written by Mecca Parker
		Team Members: Kenny Sladick, Christopher Bright, Rory Hubbard, Kam Undieh

		Voldemort is a mobile drawing robot. Other drawing mechanisms are limited 
		to a single, maximum canvas size. Morti is not. 

		Read more: (https://github.com/meccaparker/voldemort)

    Handles all printing for successful and failed function execution.
*/

#include "voldemort.h"

void pprint(char msg[]) {
  Serial.println(msg);
}

void report_success(int code) {
  Serial.print("[OK]: ");
  switch (code) {
    case SUCCESS_DC_INIT:
      pprint("Motors initialized.");
      break;
    case SUCCESS_DC_DISABLE:
      pprint("Motors disabled.");
      break;
    case SUCCESS_DC_SET_TARGET:
      pprint("Motor target set.");
      break;
    case SUCCESS_DC_MOVE: 
      pprint("Motors are in motion.");
      break;
    case SUCCESS_TOOL_ENGAGED: 
      pprint("Tool engaged.");
      break;
    case SUCCESS_TOOL_DISENGAGED: 
      pprint("Tool disengaged.");
      break;
    case SUCCESS_TOOL_INIT: 
      pprint("Tool initialized.");
      break;
    case SUCCESS_COM_RECEIVED:
      Serial.print("Command Received: ");
      break;
    case SUCCESS_TERMINAL_MODE:
      pprint("Terminal Mode activated. Send Voldemort a Gcode command via the serial port.");
      break;
    case SUCCESS_FILE_MODE:
      pprint("SD card read successfully... splendid. ");
      break;
    case SUCCESS_VMORT_INIT:
      pprint("Voldemort is ready to do your bidding...");
      break;
    case SUCCESS_GCODE_PARSE:
      pprint("GCode parsed and executed.");
      break;
    case SUCCESS_FILE_COMPLETE:
      pprint("GCode file complete.");
    default:
      pprint("Unrecognized success code.");
      break;
  }
}

void report_error(int code) {
  Serial.print("[ERROR]: ");
  switch (code) {
    case ERROR_SD_READ:
      pprint("Something sinister happened. Initialization of SD card reader failed.");
      break;
    case ERROR_UNRECOGNIZED_GCODE:
      pprint("GCode Command unrecognized.");
      break;
    
    default:
      pprint("Unknown error occured.");
      break;
  }

}