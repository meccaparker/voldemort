/* 	Voldemort the Mobile Drawing Robot (Morti) v1.0
		A Capstone Project for the Electromechanical Systems Design course 
		Carnegie Mellon University
		December 2018

		Code written by Mecca Parker
		Team Members: Kenny Sladick, Christopher Bright, Rory Hubbard, Kam Undieh

		Voldemort is a mobile drawing robot. Other drawing mechanisms are limited 
		to a single, maximum canvas size. Morti is not. 

		Read more: (https://github.com/meccaparker/voldemort)

		Arduino G-code Interpreter for Voldemort the Mobile Drawing Robot
		Adapted from https://github.com/alx/reprap-arduino-firmware/tree/master/GCode_Interpreter
			v1.0 by Mike Ellery - initial software (mellery@gmail.com)
			v1.1 by Zach Hoeken - cleaned up and did lots of tweaks (hoeken@gmail.com)
			v1.2 by Chris Meighan - cleanup / G2&G3 support (cmeighan@gmail.com)
			v1.3 by Zach Hoeken - added thermocouple support and multi-sample temp readings (hoeken@gmail.com)
			v1.4 by Mecca Parker - adapted for DC motor capability and integration with Voldemort the Mobile Drawing Robot (meccaparker@gmail.com)

		Handles all serial port communcation for the retrieval of GCode commands.
		Provides support for reading GCode files and receiving GCode input by the
		user from the serial port (File Mode and Terminal Mode).
*/

#include "voldemort.h"

#define FILE_MODE 1 					// read GCode text file for commands
#define TERMINAL_MODE 2				// listen to serial port for commands from user

#define COMMAND_SIZE 128			// maximum size of command received from serial port

char f_path[] = "gcode.txt";
byte serial_mode = 1; 				// File or Terminal Mode

char w[COMMAND_SIZE]; 				// command string

byte serial_count;
int no_data = 0;

File f;

void serial_init() {
	Serial.println("[In Progress]: Initializing serial port to receive commands.");
	serial_clear();
	switch (serial_mode) {
		case FILE_MODE:
				if (!SD.begin(CS_PIN)) {
				report_error(ERROR_SD_READ);
				serial_switch_mode(TERMINAL_MODE);
				return;
			} else {
				Serial.print("[In Progress]: Attempting to read from "); Serial.println(f_path);
				f = SD.open(f_path); // open file at designated path
				if (!f) report_error(ERROR_FILE_MODE); // report an error if unable to read file
				else report_success(SUCCESS_FILE_MODE);
				return;
			}
			break;
		
		case TERMINAL_MODE:
			report_success(SUCCESS_TERMINAL_MODE);
			break;
		
		default:
			Serial.println("[In Progress]: No mode specified. Defaulting to File Mode.");
			serial_switch_mode(FILE_MODE);
			serial_init();
			break;
	}
}

void serial_start() {
	// morti_is_available(false);
	switch (serial_mode) {
		case FILE_MODE:
			serial_file_mode();
			break;
		
		case TERMINAL_MODE:
			serial_terminal_mode();
			break;
		
		default:
			if (!SD.begin(CS_PIN)) {
				report_error(ERROR_SD_READ);
				serial_mode = TERMINAL_MODE; // switch to Terminal Mode if we can't reach the SD card
				report_success(SUCCESS_TERMINAL_MODE);
				return;
			} else {
				Serial.print("Reading from "); Serial.println(f_path);
				serial_mode = FILE_MODE;
				f = SD.open(f_path);
				if (f) report_success(SUCCESS_FILE_MODE);
				else report_error(ERROR_FILE_MODE);
				return;
			}
			break;
	}
}

void serial_clear() {
	// clear serial command string
	for (int i=0; i < COMMAND_SIZE; i++) {
		w[i] = 0;
	serial_count = 0;
	}
}

void serial_switch_mode(byte mode) {
	Serial.println("[In Progress]: Switching serial mode.");
	serial_mode = mode;
	if (mode == FILE_MODE){
		report_success(SUCCESS_FILE_MODE);
	} else {
		report_success(SUCCESS_TERMINAL_MODE);
	}
	return;
}

void serial_file_mode() {
	char c;

	if (f) {
		// Read new commands from file if available
		while (f.available()) { 
			c = f.read();
			no_data = 0;

			// Newlines are ends of commands
			if (c != '\n') { 
				w[serial_count] = c;
				serial_count++;
			}
			else { // if there are no characters, increment counter and delay.
				no_data++;
				delayMicroseconds(100);
			}
			if (serial_count && (c == '\n' || no_data > 100)) { // if there's a pause or we got a real command, do it
				parser_process_string(w, serial_count); // process our command
				serial_clear(); // clear command
			}

		}
		f.close();
		report_success(SUCCESS_FILE_COMPLETE);
	} else {
		// if the file didn't open, report the error and switch to Terminal Mode
		Serial.print("[ERROR]: Error opening ");
		Serial.print(f_path);
		Serial.println(". Listening on serial port for user commands instead.");
		serial_mode = TERMINAL_MODE;
		return;
	}
}

void serial_terminal_mode() {
	char c;

	// Read serial port if we have characters
	if (Serial.available() > 0) {
		c = Serial.read();
		no_data = 0;
		
		// Newlines are ends of commands
		if (c != '\n' && c != '\r') {
			w[serial_count] = c;
			serial_count++;
		}
	}
	
	// If there are no characters, increment counter and delay.
	else {
		no_data++;
		delayMicroseconds(100);
	}

	// If there's a pause or we got a real command, do it
	if (serial_count && (c == '\n' || c == '\r' || no_data > 1000)) {
		parser_process_string(w, serial_count); // process command
		serial_clear(); // clear command
	}

}