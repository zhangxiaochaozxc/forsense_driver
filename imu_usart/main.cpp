#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <stdint.h>
#include "serial_port.h"

#define REFRESH_INTERVAL 1000 


int main(int argc, char *argv[])
{
	int baud_rate = 115200;
	uint8_t recv_buf[2048];
	char log[512];
	int len;

	if (argc < 2)
	{
		fprintf(stderr, "Usage: %s <serial_port> [baud_rate]\n", argv[0]);
		return 1;
	}

	char *port_name = argv[1];

	if (argc > 2)
		baud_rate = atoi(argv[2]);

	int fd = serial_port_open(port_name);
	if (fd < 0)
		return 1;

	if (serial_port_configure(fd, baud_rate) < 0)
	{
		fprintf(stderr, "Cannot open %s\n", port_name);
		serial_port_close(fd);
		return 1;
	}

	printf("* %s successfully open with %d.\n", port_name, baud_rate);

    // 发送 AHRS 指令
	uint8_t cmd[34] = {0x55,0xaa,0x03,0x00,0x18,0x00,0x00,0x00,0x80,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x52,0xd8,0x8e,0xe8};
	for(int i = 0; i < 10; i++){
	write(fd, cmd, sizeof(cmd));
	}
		while (1)
	   {
			len = read(fd, recv_buf, sizeof(recv_buf));
			if(len>=sizeof(recv_buf)) len=sizeof(recv_buf);
			for (int i = 0; i < len; i++)
			imu_rx(recv_buf[i]);
			usleep(REFRESH_INTERVAL);
		}



	return 0;
}

