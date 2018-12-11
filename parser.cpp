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
			v1.3 by Zach Hoeken - added thermocouple support and multi-sample temp readings. (hoeken@gmail.com)
			v1.4 by Mecca Parker - adapted for DC motor capability and integration with Voldemort the Mobile Drawing Robot (meccaparker@gmail.com)

		Parses GCode commands received into executable dc motor setpoints and more.
 */

#include "voldemort.h"

// Define the parameters of our machine.
#define X_STEPS_PER_INCH 416.772354
#define X_STEPS_PER_MM   16.4083604
#define X_MOTOR_STEPS    400

#define Y_STEPS_PER_INCH 416.772354
#define Y_STEPS_PER_MM   16.4083604
#define Y_MOTOR_STEPS    400

#define Z_STEPS_PER_INCH 16256.0
#define Z_STEPS_PER_MM   640.0
#define Z_MOTOR_STEPS    400

#define FAST_XY_FEEDRATE 1000.0
#define FAST_Z_FEEDRATE  50.0

// Units in curve section
#define CURVE_SECTION_INCHES 0.019685
#define CURVE_SECTION_MM 0.5

struct FloatPosition {
	float x;
	float y;
 	float z;
};

FloatPosition current_pos = { 0, 0 , 0}; 		// tracks current position
FloatPosition target_pos = { 0, 0 , 0};			// tracks target position
FloatPosition delta_pos = { 0, 0 , 0};

// default to millimeters for units
float x_units = X_STEPS_PER_MM;
float y_units = Y_STEPS_PER_MM;
float z_units = Z_STEPS_PER_MM;
float curve_section = CURVE_SECTION_MM;

// direction vars
byte x_direction = 1;
byte y_direction = 1;
byte z_direction = 1;

byte abs_mode = 0;   //0 = incremental positioning; 1 = absolute positioning

// feedrate variables.
float feedrate = 0.0;
long feedrate_micros = 0;

// Look for the command if it exists.
int has_command(char key, char instruction[], int string_size) {
	for (int i=0; i<string_size; i++) {
		if (instruction[i] == key) return 1;
	}
	return 0;
}

// Look for the number that appears after the char key and return it
double search_string(char key, char instruction[], int string_size) {
	char temp[10] = "";

	for (int i=0; i<string_size; i++){
		if (instruction[i] == key) {
			i++;      
			int k = 0;
			while (i < string_size && k < 10) {
				if (instruction[i] == 0 || instruction[i] == ' ') break;
				temp[k] = instruction[i];
				i++;
				k++;
			}
			return strtod(temp, NULL);
		}
	}
	
	return 0;
}

// Print GCode instruction receieved
void print_instruction(char instruction[], int size) {
	report_success(SUCCESS_COM_RECEIVED);
	for (int i = 0; i < size; i++) {
    if (instruction[i] != '\n') Serial.print(instruction[i]);
	}
	Serial.println();
}

// Set current position variables from outside of parser.cpp file
void parser_set_current_pos(float x, float y) {
	current_pos.x = x;
	current_pos.y = y;
}

// Read the string and execute instructions
void parser_process_string(char instruction[], int size) {

	FloatPosition fp;
	fp.x = 0.0;
	fp.y = 0.0;
	fp.z = 0.0;

	int code = 0;

	print_instruction(instruction, size);
	
	// The character / means delete block... used for comments
	if (instruction[0] == '/') {
		Serial.println("[OK]: Delete block detected.");
		return;
	}
	
	// Engage tool shortcut
	if (instruction[0] == '!') {
		Serial.println("[IN PROGRESS]: Engaging tool.");
		tool_engage();
		return;
	}

	// Disengage tool shortcut
	if (instruction[0] == '@') {
		Serial.println("[IN PROGRESS]: Disengaging tool.");
		tool_disengage();
		return;
	}

	// Change serial mode shortcut
	if (instruction[0] == '#') {
		Serial.println("[IN PROGRESS]: Changing mode.");
		byte mode = (int)instruction[1];
		serial_switch_mode(mode);
		return;
	}
	
	// Check for command
	if (
		has_command('G', instruction, size) ||
		has_command('X', instruction, size) ||
		has_command('Y', instruction, size) ||
		has_command('Z', instruction, size)
	) {

		// Identify command code 
		code = (int)search_string('G', instruction, size);
		
		// Get coordinates if required by the code type given. Note that this code 
		// structure causes codes 0 through 3 to fall into the same code block.
		switch (code) {
			case 0: 
			case 1:
			case 2: // CCW Arc command (G3)
			case 3:
			// Check command for X, Y, and Z coordinates
			if (has_command('X', instruction, size)) {
				fp.x = search_string('X', instruction, size);
			} else fp.x = current_pos.x;
		
			if (has_command('Y', instruction, size)) {
				fp.y = search_string('Y', instruction, size);
			} else fp.y = current_pos.y;
		
			if (has_command('Z', instruction, size)) {
				fp.z = search_string('Z', instruction, size);
			} else fp.z = current_pos.z;
			break;
		}

		// Execute GCode command
		switch (code) {
			// G0 & G1 are basically the same thing
			// G0 - Rapid Movement
			case 0:
			// G1 - Coordinated Movement

			case 1:
				// Set our target
				dc_set_target(fp.x, fp.y, fp.z);

				// Feedrate Control (feedrate control not implemented)
				if (has_command('G', instruction, size)) {
					// Adjust if we have a specific feedrate (only for G1 command)
					if (code == 1) {
						// Pull feedrate from instruction
						feedrate = search_string('F', instruction, size);
						if (feedrate > 0) feedrate_micros = 42; //calculate_feedrate_delay(feedrate);
						// No feedrate
						else feedrate_micros = 42; //getMaxSpeed();
					} else feedrate_micros = 42; //getMaxSpeed();
				} else { // if not a G command, we just have coordinates
					if (feedrate > 0)
						feedrate_micros = 42; //calculate_feedrate_delay(feedrate);
					// No feedrate, set to max speed
					else
						feedrate_micros = 42; //getMaxSpeed();
				}

				// Position and speed set - complete move
				dc_move(feedrate);
			break;
			
			// G2 - Clockwise arc
			case 2:

			// G3 - CCW arc
			case 3:
				FloatPosition cent; // Center of arc 

				// Center coordinates are always relative
				cent.x = search_string('I', instruction, size) + current_pos.x;
				cent.y = search_string('J', instruction, size) + current_pos.y;
				
				float angleA, angleB, angle, radius, length, aX, aY, bX, bY;

				aX = (current_pos.x - cent.x);
				aY = (current_pos.y - cent.y);
				bX = (fp.x - cent.x);
				bY = (fp.y - cent.y);
				
				if (code == 2) { // Clockwise
					angleA = atan2(bY, bX);
					angleB = atan2(aY, aX);
				} else { // Counterclockwise
					angleA = atan2(aY, aX);
					angleB = atan2(bY, bX);
				}

				// Make sure angleB is always greater than angleA
				// and if not add 2PI so that it is (this also takes
				// care of the special case of angleA == angleB,
				// ie we want a complete circle)
				if (angleB <= angleA) angleB += 2 * M_PI;
				angle = angleB - angleA;

				radius = sqrt(aX * aX + aY * aY);
				length = radius * angle;
				int steps, s, step;
				steps = (int) ceil(length / curve_section);

				// Break arc into small segments to be executed individually
				FloatPosition newPoint;
			
				for (s = 1; s <= steps; s++) {
					step = (code == 3) ? s : steps - s; // Work backwards for CW
					newPoint.x = cent.x + radius * cos(angleA + angle * ((float) step / steps));
					newPoint.y = cent.y + radius * sin(angleA + angle * ((float) step / steps));
					dc_set_target(newPoint.x, newPoint.y, fp.z);

					// Need to calculate rate for each section of curve
					if (feedrate > 0) feedrate_micros = 42; //calculate_feedrate_delay(feedrate);
					else feedrate_micros = 42; //getMaxSpeed();

					// Make the move
					dc_move(feedrate_micros);
				}
				break;

			// G4 - Dwell
			case 4:
				delay((int)search_string('P', instruction, size));
				break;

			// G20 - Use inches as units
			case 20:
				x_units = X_STEPS_PER_INCH;
				y_units = Y_STEPS_PER_INCH;
				z_units = Z_STEPS_PER_INCH;
				curve_section = CURVE_SECTION_INCHES;
				// calculate_deltas();
				break;

			// G21 - Use millimeters as units
			case 21:
				x_units = X_STEPS_PER_MM;
				y_units = Y_STEPS_PER_MM;
				z_units = Z_STEPS_PER_MM;
				curve_section = CURVE_SECTION_MM;
				// calculate_deltas();
				break;

			// G28 - Homing. Return to machine 0 with tool up.
			case 28:
				dc_set_target(0.0, 0.0, 1.0);
				dc_move(dc_get_max_speed());
				break;

			// G30 - Homing via intermediate point (may be buggy - not tested).
			case 30:
				fp.x = search_string('X', instruction, size);
				fp.y = search_string('Y', instruction, size);
				fp.z = search_string('Z', instruction, size);

				// Set our target
				if(abs_mode) {
					if (!has_command('X', instruction, size)) fp.x = current_pos.x;
					if (!has_command('Y', instruction, size)) fp.y = current_pos.y;
					if (!has_command('Z', instruction, size)) fp.z = current_pos.z;
					dc_set_target(fp.x, fp.y, fp.z);
				} else {
					dc_set_target(current_pos.x + fp.x, current_pos.y + fp.y, fp.z);
					dc_move(dc_get_max_speed());
				}

				//go home.
				dc_set_target(0.0, 0.0, 1);
				dc_move(dc_get_max_speed());
				break;

			// G90 - Absolute FloatPositioning
			case 90:
				abs_mode = 1;
				break;

			// G91 - Incremental FloatPositioning
			case 91:
				abs_mode = 0;
				break;
			
			// G92 - Set current position as home
			case 92:
					current_pos.x = 0;
					current_pos.y = 0;
					current_pos.z = 0;
					break;

/*
			//Inverse Time Feed Mode
			case 93:

			break;  //TODO: add this

			//Feed per Minute Mode
			case 94:

			break;  //TODO: add this
*/

			default:
				report_error(ERROR_UNRECOGNIZED_GCODE);
		}
	}
	
	// Find an M code
	if (has_command('M', instruction, size)) {
		code = search_string('M', instruction, size);
		switch (code) {
			//TODO: this is a bug because search_string returns 0.
			case 0:
				1;
				break;
	/*
			case 0:
				//todo: stop program
			break;

			case 1:
				//todo: optional stop
			break;

			case 2:
				//todo: program end
			break;
	*/		
			//set max extruder speed, 0-255 PWM
			case 100:
				// extruder_speed = (int)(search_string('P', instruction, size));
				break;

			//turn extruder on, forward
			case 101:
				// extruder_set_direction(1);
				// extruder_set_speed(extruder_speed);
				break;

			//turn extruder on, reverse
			case 102:
				// extruder_set_direction(0);
				// extruder_set_speed(extruder_speed);
				break;

			//turn extruder off
			case 103:
				// extruder_set_speed(0);
				break;

			//custom code for temperature control
			case 104:
				// extruder_set_temperature((int)search_string('P', instruction, size));

				//warmup if we're too cold.
				// while (extruder_get_temperature() < extruder_target_celsius)
				// {
				// 	extruder_manage_temperature();
				// 	Serial.print("T:");
				// 	Serial.println(extruder_get_temperature());
				// 	delay(1000);	
				// }
				
				break;

			//custom code for temperature reading
			case 105:
				// Serial.print("T:");
				// Serial.println(extruder_get_temperature());
				break;
			
			//turn fan on
			case 106:
				// extruder_set_cooler(255);
				break;

			//turn fan off
			case 107:
				// extruder_set_cooler(0);
				break;
			
			default:
				break;
		}		
	}
	report_success(SUCCESS_GCODE_PARSE);
}