/* 	Voldemort the Mobile Drawing Robot (Morti) v1.0
		A Capstone Project for the Electromechanical Systems Design course 
		Carnegie Mellon University
		December 2018

		Code written by Mecca Parker
		Team Members: Kenny Sladick, Christopher Bright, Rory Hubbard, Kam Undieh

		Voldemort is a mobile drawing robot. Other drawing mechanisms are limited 
		to a single, maximum canvas size. Morti is not. 

    Read more: (https://github.com/meccaparker/voldemort)

    Handles tool actuation for Morti.
*/

#include "voldemort.h"

Servo servo;

byte engaged_angle = 75;
byte disengaged_angle = 170;
byte servo_pin = 10;
bool engaged = 0; // engaged boolean

// Lower tool
void tool_engage() {
  if (!engaged) {
    engaged = true;
    servo.write(engaged_angle);
    delay(500);
    report_success(SUCCESS_TOOL_ENGAGED);
  }
}

// Raise tool
void tool_disengage() {
  if (engaged) {
    engaged = false;
    servo.write(disengaged_angle); 
    delay(500);
    report_success(SUCCESS_TOOL_DISENGAGED);
  }
}

// Initialize Morti
void tool_init() {
  servo.attach(servo_pin);
  tool_disengage(); delay(250);
  tool_engage(); delay(250);
  tool_disengage();
  report_success(SUCCESS_TOOL_INIT);
}