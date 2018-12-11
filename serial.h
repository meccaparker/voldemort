#ifndef serial.h
#define serial.h

void serial_init();

void serial_clear();

void serial_switch_mode(byte mode);

void serial_start();

void serial_terminal_mode();

void serial_file_mode();

#endif