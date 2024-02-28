#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/time.h>
#include <stdint.h>


#define IMU_PARSE_STATE_SYNC1_ID      0xAA
#define IMU_PARSE_STATE_SYNC2_ID      0x55

enum 
{
     UART_STREAM_MODE_SEND_AHRS=1,
     UART_COMMAND_MODE=100,
};

typedef enum {
    IMU_PARSE_STATE_WAIT_SYNC1=0,
    IMU_PARSE_STATE_WAIT_SYNC2,
    IMU_PARSE_STATE_WAIT_ID1,
    IMU_PARSE_STATE_WAIT_ID,
    IMU_PARSE_STATE_WAIT_LENGTH1,
    IMU_PARSE_STATE_WAIT_LENGTH2,
    IMU_PARSE_STATE_PAYLOAD,
    IMU_PARSE_STATE_CHECK1,
    IMU_PARSE_STATE_CHECK2,
    IMU_PARSE_STATE_CHECK3,
    IMU_PARSE_STATE_CHECK4
} imu_parse_state_t; 

typedef struct {
    volatile unsigned char state;
    volatile unsigned int count;
    volatile unsigned int id;
    volatile unsigned int length;
    volatile uint32_t check;
    volatile unsigned char id_temp;
    volatile unsigned char length_temp;
    volatile unsigned char check_temp1;
    volatile unsigned char check_temp2;
    volatile unsigned char check_temp3;
    volatile unsigned char check_temp4;
} ParseStruct;

#pragma pack(push) //保存对齐状态
#pragma pack(1)    //设定为1字节对齐

/**
 * 接收AHRS帧负载
 */
struct AHRS_PAYLOAD_9axies
{ 
  uint32_t time_us;
  float pitch;
  float roll;
  float yaw;
  float imu[7];
  float mag[3];
};

struct AHRS_PAYLOAD
{ 
  uint32_t time_us;
  float pitch;
  float roll;
  float yaw;
  float imu[7];
};

#pragma pack(pop)//恢复对齐状态
typedef union 
{ 
  struct AHRS_PAYLOAD_9axies AHRS_DATA_9axis;
  struct AHRS_PAYLOAD AHRS_DATA;
  unsigned char payload[1000]; 
} ParseUnion;

 void imu_rx(unsigned char data);
















int serial_port_open(const char *portname);
int serial_port_configure(int fd, int baud_rate);
int serial_send_then_recv(int fd, const char *send_str, const char *expected, char *recv_buf, size_t recv_buf_size, int timeout_ms);
void serial_port_close(int fd);
int serial_port_write(int fd, char *buffer, int size);
int serial_port_read(int fd, const char *buffer, int size);

#endif // SERIAL_PORT_H