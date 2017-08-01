/*
 * create.c
 *
 *  Created on: 2017. 7. 27.
 *      Author: youngkwang
 */


/*
 * main.cpp
 *
 *  Created on: 2017. 7. 27.
 *      Author: youngkwang
 */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <stdlib.h>
#include "MW-AHRSv1.h"
#include <errno.h>

#define BAUDRATE B115200
#define MODEDEVICE "/dev/ttyUSB0"
#define STX 0x02
#define Length 13
#define DeviceID 1
#define ETX 0x03

enum MW_AHRS_AXIS {
	X_AXIS = 1, Y_AXIS, Z_AXIS
};


void pkt_init (unsigned char* msg);

// Convert number to char array
void int_to_msg (unsigned char* data, int val);
void float_to_msg (unsigned char* data, float val);

// Create request packet
void object_write_request(unsigned char* msg, unsigned char idx, unsigned char* value);
void object_read_request(unsigned char* msg, unsigned char idx, unsigned char axis);

// Calculate checksum bit
void checksum (unsigned char* msg);

int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);

int main()
{
	unsigned char pkt_send[Length];
	unsigned char pkt_recv[Length];
	unsigned char data[4];
	memset(pkt_send, 0, sizeof(pkt_send));
	memset(pkt_recv, 0, sizeof(pkt_recv));
	memset(data, 0, sizeof(data));

//	unsigned char id[Length] = "id\n";

	pkt_init(pkt_send);

	object_read_request(pkt_send, TEMPERATURE, 1);

	checksum(pkt_send);

	/*
	for(int i = 0;i < Length; i++)
	{
		printf("%02X", pkt_send[i]);
	}
	printf("\n");
	*/


	int fd;
	fd=open(MODEDEVICE, O_RDWR | O_NOCTTY | O_SYNC);

	// Defend code
	if (fd < 0)
	{
		printf("Failed to connect IMU\n");
		exit(-1);
	}

	set_interface_attribs (fd, BAUDRATE, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking

	write (fd, "hello!\n", 7);           // send 7 character greeting

	usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
	                                     // receive 25:  approx 100 uS per char transmit
	char buf [100];
	int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read

	/*
	struct termios newtio;
	// newtio <-- serial port setting.
	memset(&newtio, 0, sizeof(struct termios));
	newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
	//B9600 : 시리얼 통신 baurate 설정
	//CS8   : 데이터 BIT설정 CS5, CS6, CS7, CS8 중 선택
	//CREAD : 데이터를 전송 뿐만 아니라 읽기도 가능하게
	//CLOCAL: 내부 통신 포트를 사용
	newtio.c_iflag = IGNPAR;
	//Parity bit를 사용하지 않음
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME]   = 0;
	newtio.c_cc[VMIN]    = 0;

	tcflush(fd, TCIFLUSH);
	//    tcflush() discards data written to the object referred to by fd but not transmitted
	tcsetattr(fd, TCSANOW, &newtio);
	//    tcsetattr()  sets  the  parameters associated with the terminal


	int write_byte = 0;

	while(write_byte < 13)
	{
		write_byte += write(fd, pkt_send, Length);
		printf("write_byte = %d", write_byte);
	}

	usleep((Length + 25) * 100);


	int read_byte = 1;

	while(read_byte > 0)
	{
		read_byte = read(fd, pkt_recv, strlen(pkt_recv));

		printf("Read Byte = %d\n", read_byte);

		for(int i = 0;i < Length; i++)
		{
			printf("%02X", pkt_recv[i]);
		}
		printf("\n");
	}
	*/

	close(fd);

	return 0;
}

void pkt_init (unsigned char* msg)
{
	msg[0] = STX;
	msg[1] = Length;
	msg[2] = DeviceID;
	msg[12] = ETX;

	return;
}

void object_read_request(unsigned char* msg, unsigned char idx, unsigned char axis)
{
	unsigned char cmd = AC_OBJECT_READ_REQ;
	cmd = cmd | OT_FLOAT;

	msg[3] = cmd;
	msg[5] = idx;
	msg[6] = axis;
}

void object_write_request(unsigned char* msg, unsigned char idx, unsigned char* value)
{
	unsigned char cmd = AC_OBJECT_WRITE_REQ;
	msg[5] = idx;
	for(int i = 7; i < 11; i++)
	{
		msg[i] = value[i - 7];
	}
}

void int_to_msg (unsigned char* data, int val)
{
	/*
	 * using byte pointer
	 *
	for (size_t i = 0; i < sizeof(val); ++i) {
	  // Convert to unsigned unsigned char* because a unsigned char is 1 byte in size.
	  // That is guaranteed by the standard.
	  // Note that is it NOT required to be 8 bits in size.
	  unsigned char byte = *((unsigned char *)&val + i);
	  data[sizeof(val) - i] = byte;
	  printf("Byte %d = %x\n", i, (unsigned)byte);
	  printf("%d = %02x\n", i, data[4 - i]);
	}
	 */

	for(int i = 3; i >= 0; i--)
	{
		data[i] = val;
		val >>= 8;
	}

	/*
	 * print in Hexadecimal
	 *
	for (int i = 0; i < 4; i++)
	{
		printf("%02X", data[i]);
	}
	printf("\n");
	 */

}

void float_to_msg (unsigned char* data, float val)
{
	for(int i = 3; i >= 0; i--)
	{
		data[i] = val;
		//val >>= 8;
	}
}

void checksum (unsigned char *msg)
{
	unsigned char cs = 0;

	for(int i = 2; i <= 10; i++)
		cs += msg[i];

	msg[11] = cs;
}


int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
//                error_message ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
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
        	printf("Error from tcsetattr");
//                error_message ("error %d from tcsetattr", errno);
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
        	printf("Error from tggetattr");
//                error_message ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0) printf("error from setting term attributes");
//                error_message ("error %d setting term attributes", errno);
}
