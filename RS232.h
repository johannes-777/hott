#ifndef RS232_H_INCLUDED
#define RS232_H_INCLUDED

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);

#endif // RS232_H_INCLUDED
