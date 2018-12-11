/* 	Voldemort the Mobile Drawing Robot (Morti) v1.0
		A Capstone Project for the Electromechanical Systems Design course 
		Carnegie Mellon University
		December 2018

		Code written by Mecca Parker
		Team Members: Kenny Sladick, Christopher Bright, Rory Hubbard, Kam Undieh

		Voldemort is a mobile drawing robot. Other drawing mechanisms are limited 
		to a single, maximum canvas size. Morti is not. 

		Read more: (https://github.com/meccaparker/voldemort)

		Adapted from FastLED library by Daniel Garcia (https://github.com/FastLED).
		
		Lights up the LEDs in a continuous, pulsing fashion.
*/

#include "voldemort.h"

CRGB leds[NUM_LEDS]; // Define the array of leds
byte freq = 35;

void led_init() {
	LEDS.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);
	LEDS.setBrightness(55);
}

void fadeall() { for(int i = 0; i < NUM_LEDS; i++) { leds[i].nscale8(200); } }

void led_pulse() {
	static uint8_t hue = 0;

	// First slide the led in one direction
	for(int i = 0; i < NUM_LEDS; i++) {
		// Set the i'th led to red 
		leds[i] = CHSV(hue++, 255, 255);
		// Show the leds
		FastLED.show(); 
		// now that we've shown the leds, reset the i'th led to black
		// leds[i] = CRGB::Black;
		fadeall();
		// Wait a little bit before we loop around and do it again
		delay(freq);
	}

	// Now go in the other direction.  
	for(int i = (NUM_LEDS)-1; i >= 0; i--) {
		// Set the i'th led to red 
		leds[i] = CHSV(hue++, 255, 255);
		// Show the leds
		FastLED.show();
		// now that we've shown the leds, reset the i'th led to black
		// leds[i] = CRGB::Black;
		fadeall();
		// Wait a little bit before we loop around and do it again
		delay(freq);
	}
}

