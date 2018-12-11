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

#ifndef dc.h
#define dc.h

void dc_init();

float dc_get_max_speed();

void dc_set_target(float sp_x, float sp_y, float sp_z);

void dc_move(float fr);

void dc_disable();

#endif 