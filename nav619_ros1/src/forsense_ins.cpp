//forsense_ins.cpp
#include "serial_parse.h"

#define IMU_SERIAL   "/dev/ttyUSB0"
#define BAUD         (460800)
#define BUF_SIZE     1024


static int frame_rate;
static int frame_count;
extern  MULTI_LONG_CMD_STRUCT data_cmd_long;
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
	ros::init(argc, argv, "forsense_ins");
	ros::NodeHandle n;

    forsense_insdata_pub = n.advertise<forsense_ins::nav619Data>("nav619Data", 1000);

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
	Send_CMD_LONG(3,8,0,0,0,0,0);
	for (int i = 0; i < 10; i++)
	sp.write((uint8_t *)&data_cmd_long,sizeof(data_cmd_long));

	ros::Rate loop_rate(500);


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
