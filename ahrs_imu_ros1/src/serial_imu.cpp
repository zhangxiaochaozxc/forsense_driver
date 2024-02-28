//serial_imu.cpp
#include "serial_parse.h"

#define IMU_SERIAL   "/dev/ttyUSB0"
#define BAUD         (115200)
#define BUF_SIZE     1024

ros::Publisher IMU_pub;

static int frame_rate;
static int frame_count;

static uint8_t buf[2048];

void timer(int sig)
{
	if(SIGALRM == sig)
	{
		frame_rate = frame_count;
		frame_count = 0;
		alarm(1);
	}
}

int main(int argc, char** argv)
{
	int rev = 0;
	ros::init(argc, argv, "serial_imu");
	ros::NodeHandle n;

	IMU_pub = n.advertise<sensor_msgs::Imu>("/IMU_data", 20);

	serial::Serial sp;

	serial::Timeout to = serial::Timeout::simpleTimeout(100);

	sp.setPort(IMU_SERIAL);

	sp.setBaudrate(BAUD);

	sp.setTimeout(to);
	
	signal(SIGALRM,timer);

	try
	{
		sp.open();
	}
	catch(serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port.");
		return -1;
	}
    
	if(sp.isOpen())
	{
		ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
	}
	else
	{
		return -1;
	}
	
	alarm(1);
	
	ros::Rate loop_rate(500);

//AHRS
	uint8_t cmd[34]={0x55,0xaa,0x03,0x00,0x18,0x00,0x00,0x00,0x80,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x52,0xd8,0x8e,0xe8};

	for(int i=0;i<10;i++)
	sp.write(cmd, sizeof(cmd));

	while(ros::ok())
	{
		size_t num = sp.available();
		if(num!=0)
		{
			uint8_t buffer[BUF_SIZE]; 
	
			if(num > BUF_SIZE)
				num = BUF_SIZE;
			
			num = sp.read(buffer, num);
			if(num > 0)
			{

				for (int i = 0; i < num; i++)
				{
					imu_rx(buffer[i]);
				}
			}
		}
		loop_rate.sleep();
	}
    
	sp.close();
 
	return 0;
}
