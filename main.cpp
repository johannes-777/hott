/****************************************************************************************************
* Author: Johannes Schabauer
* Date: 5th of May 2014
* Copyright:
*	GPL V2
* Description:
*  	This little program is a demonstration of how to decode a Graupner SUMD protocol 
*  	It was deveoloped using a Graupner GR 12SC receiver and a Raspberry PI with
* 	Raspian Wheezy.  
*	
* Connecting the Hardware:
* Please take a look at this website for the connection:
* 	http://elinux.org/RPi_Low-level_peripherals#GPIO_hardware_hacking
*  	You will need to disable the Linux Console on the RPI before using the UART:
*		/boot/cmdline: dwc_otg.lpm_enable=0 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline rootwait
*		/etc/inittab: 
*			#Spawn a getty on Raspberry Pi serial line
*			#T0:23:respawn:/sbin/getty -L ttyAMA0 115200 vt100
*   Restart and Check if the uart is free now:	
*		cat /proc/cmdline
*		ps aux | grep ttyAMA0
* 	Then connect to Channel 6 of the GR 12SH to the RPI GPIO Connector:
*		Pin 4: +5V
*		Pin 6: GND
*		[Pin 8: TXD]  (No need to send anything to the Receiver)
*		Pin 10: RXD
*	
* Testing:
* Set the serial Port to 115200 8N1 (e.g. with minicom) and check with hexdump if it works:
* 	cat /dev/ttyAMA0 | hexdump -v -e '21/1 "%-x\t" "\n"' > hott.txt
* If you turn on the sender, you should receive something like this:
*	a8	0	8	1c	20	2e	e0	2e	e0	2e	e8	2e	e0	22	60	2e	e0	35	20	0	11
*	a8	0	8	1c	20	2e	e0	2e	e0	2e	e8	2e	e0	22	60	2e	e0	35	20	0	11
*
* Have fun.
****************************************************************************************************/

#include <iostream>
#include <features.h>
#include "RS232.h"
#include <string.h>
#include <unistd.h>
#include <iomanip>

#define SIZE_TELEGRAM 21
#define HOTT_CHAR_0 0xA8				//This is the Start Character
#define HOTT_CHAR_1 0x00				//This may indicate some error code (don't know, since I didn't sign the NDA)
#define HOTT_CHAR_2 0x08				//This is the Number of Channels.  Currently only 8 Channels are supported

using namespace std;

const char *portname = "/dev/ttyAMA0";			//This may also be ttyUSB1 or tty1.  But be sure to take care of the 3.3Volt UART levels!
static int error= 0;

const float S_RAW_MIN= 7172;
const float S_MIN= -150;
const float S_RAW_MAX= 16735;
const float S_MAX= 150;


/**************************************************************************/
/* LOW LEVEL RS232 HANDLING
/**************************************************************************/
int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // ignore break signal
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}

/**************************************************************************/
/* INIT AND TELEGRAM RETRIEVAL
/**************************************************************************/
/**
* Initialize serial Port for HOTT
* Returns the FileDevice 
*/
int hott_init()
{
	int fd= 0;
    fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf ("error %d opening %s: %s", errno, portname, strerror (errno));
        return -1;
    }

    int rc = set_interface_attribs (fd, B115200, 0);  // set speed to 57,600 bps, 8n1 (no parity)
    set_blocking (fd, false);  // set no blocking
    return fd;
}

/**
* Read one Telegram from RS232
*/
int getTel(int fd, char* buf, int len)
{
    int n;
    char start;

    while(true)
    {
        //Warte auf das Startzeichen
        memset(buf, 0, len);
        n = read (fd, &start, 1);  // read up to 100 characters if ready to read
        if (n != 1)
        {
            cout << "Kein Empfang" << endl;
            continue;
        }
        if ( start != HOTT_CHAR_0)
        {
            cout << "Unerwartetes Zeichen: " << start << endl;
            continue;
        }
		
        //Das erste Zeichen ist richtig
        buf[0]= start;
        n= read(fd, &buf[1], 2);
        if (n != 2)
        {
            cout << "Kein Empfang" << endl;
            continue;
        }
		
        //Überprüfe die nächsten 2 Startzeichen
        if ((buf[1] != HOTT_CHAR_1) || (buf[2] != HOTT_CHAR_2))
        {
            cout << "Unerwartete StartZeichen: " << buf << endl;
            continue;
        }

        //Startzeichenfolge richtig, Befülle das Telegramm
        for (int i= 3; i< SIZE_TELEGRAM; i++)
        {
            n= read(fd, &buf[i], 1);
            if (n != 1)
            {
                cout << "Kein Empfang" << endl;
                continue;
            }
        }
        //Jetzt ist das Telegramm vollständig eingelesen
        return 0;
    }
}

/**************************************************************************/
/* HOTT SUMD TELEGRAM DECODING
/**************************************************************************/
/**
* decode Telegram and calc the values
*/
void get_channels(char* tel, int s_raw[], float s_final[], int n_of_channels)
{
    int offset= 3;  //Values start at Position 3

    for (int i= 1; i<= n_of_channels; i++)
    {
        //Get Raw Values
        s_raw[i]=  tel[offset]*255 + tel[offset+1];
        offset+= 2;  //Prepare Offset to read next value;

        //Calculate final Values from -150% to +150%
        s_final[i]= ((S_MAX-S_MIN)/(S_RAW_MAX - S_RAW_MIN)) * (s_raw[i] - S_RAW_MIN) + S_MIN;
        std::cout << std::fixed << std::setw( 5 ) << std::setprecision( 1 ) << std::setfill( ' ' ) << s_final[i] << "\t";  //Can't help it: I just prefer the good old printf ...
    }
    cout << "\r";
}

/**
 * Return true, if the Checksum matches.
 * If not, print the erorrnous telegram and return false
 */
bool get_checksum(unsigned char* tel)
{
    int i;
    unsigned char c= 0;
    unsigned char check= tel[SIZE_TELEGRAM-1];


    c= 0x00;
    for (i= 0; i< (SIZE_TELEGRAM-1); i++)
    {
        c= c+ tel[i];
    }
    if (c != check)
    {
		if (check == 0x0A)			//This is somewhat wired, maybe it is a bug on the Graupner GR 12SC receiver, which I was using here
		{
		  return true;
		  //Schon OK 
		}
        printf("Checksum Failure: checksum is 0x%x, calculated 0x%x!!\r\n", check, c);
        printf("%d: ", error++);
        for (int i= 0; i< SIZE_TELEGRAM; i++)
        {
           printf("0x%x\t", tel[i]);
        }
        printf("\r\n");
        return false;
    }
    return true;
}

/**************************************************************************/
/* MAIN
/**************************************************************************/
int main(int argc, char* argv[])
{
	int fd= 0;
    char buf [SIZE_TELEGRAM+10];
    int s_raw[9];
    float s_final[9];
    int rc;

    cout << "Starting to read from HOTT receiver at port " << portname << endl;
    fd= hott_init();
    cout << "Init returned the filedevice" << fd << endl;

    while(true)
    {
        rc= getTel(fd, buf, sizeof(buf));
        if (get_checksum((unsigned char*) buf))
        {
            get_channels(buf, s_raw, s_final, 8);
        }
        else{
            //cout << "Checksum Failure" << endl;
        };
    }
    return 0;
}
