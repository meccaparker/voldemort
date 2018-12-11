/* 	Voldemort the Mobile Drawing Robot (Morti) v1.0
		A Capstone Project for the Electromechanical Systems Design course 
		Carnegie Mellon University
		December 2018

		Code written by Mecca Parker
		Team Members: Kenny Sladick, Christopher Bright, Rory Hubbard, Kam Undieh

		Voldemort is a mobile drawing robot. Other drawing mechanisms are limited 
		to a single, maximum canvas size. Morti is not. 

		Read more: (https://github.com/meccaparker/voldemort)

		Main Arduino file that initializes Morti.
*/

#include "voldemort.h"

void setup() {
	Serial.begin(9600);
	
	digitalWrite(CS_PIN, HIGH); 

	init();
	
	report_success(SUCCESS_VMORT_INIT);
}	

void loop() {
	led_pulse();  // control status led

	serial_start(); // Read serial port for commands during each loop
}

void init() {
	led_init(); delay(3000); // set up led strip

	serial_init(); delay(500); // open serial port

	tool_init(); delay(500); // jog tool

	dc_init(); delay(500); // prepare dc motors
}
 

