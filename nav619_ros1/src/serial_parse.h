#ifndef __SERIAL_PARSE_H_
#define	__SERIAL_PARSE_H_

#include <stdint.h>
#include <string.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <stdio.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <signal.h>

#include "forsense_ins/nav619Data.h"


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
 *  forsense_ins数据帧
 */
struct  NAV_DATA
{
    //uint8_t header1; //0xAA 帧头
    //uint8_t header2; //0x55 帧头
    //uint16_t id;//帧ID 0x166
    //uint16_t length; //帧长
    uint32_t itow;//GPS周内毫秒
    uint16_t week_num;//GPS周计数
    int32_t lat;//纬度  纬度的值是lat除以10的7次方 double lat=((double)nav_struct.lat)/10000000.0;
    int32_t lon;//经度  经度的值是lon除以10的7次方 double lon=((double)nav_struct.lon)/10000000.0;
    int32_t hgt;//高度  高度的值是hgt除以10的3次方 double hgt=((double)nav_struct.hgt)/1000.0;
    float vn;//北向速度
    float ve;//东向速度
    float vd;//地向速度
    float roll;//横滚
    float pitch;//俯仰
    float yaw;//航向
    float rtk_yaw;//双天线航向
    float wheel_angle;//预留
    float imu[7];// imu[0]加速度x，imu[1]加速度y,imu[2]加速度z,imu[3]角速度x,imu[4]角速度y,imu[5]角速度z,imu[6]温度
    uint8_t fix_type;//定位状态
    uint8_t sv_num;//星数
    uint8_t diff_age;//差分延时
    uint8_t heading_type;//定向状态
    uint16_t pos_acc;//位置精度因子（cm）
    uint16_t status;//状态位
    uint32_t rev[2];//rev[0]预留1,rev[1]预留2
    //    uint32_t check_crc;//crc校验位
}__attribute__((packed));


void imu_rx(unsigned char data);
uint32_t crc_crc32(uint32_t crc, const uint8_t *buf, uint32_t size);
extern ros::Publisher forsense_insdata_pub;


#endif

