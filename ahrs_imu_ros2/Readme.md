# ROS2串口例程

本文档介绍如何在ROS2下来读取IMU的数据，并提供了c++语言例程代码，通过执行ROS2命令，运行相应的节点，就可以看到打印到终端上的信息。

* 测试环境：Ubuntu22

* ROS版本：ROS2 humble

* 测试设备：

## 安装USB-UART驱动

Ubuntu 认不需要安装串口驱动。将调试版连接到电脑上时，会自动识别设备。识别成功后，会在dev目录下出现一个对应的设备:ttyUSBx

检查USB-UART设备是否被Ubantu识别：

1. 打开终端，输入`ls /dev`,先查看已经存在的串口设备。
2. 查看是否已经存在  ttyUSBx 这个设备文件，便于确认对应的端口号。
4. 接下来插入USB线，连接调试板，然后再次执行`ls /dev`。 dev目录下多了一个设备`ttyUSB0`：

```shell
linux@ubuntu:~$ ls /dev
.....
hpet             net           tty11     tty4   ttyS0      ttyUSB0    vhost-vsock
hugepages        null          tty12     tty40  ttyS1      udmabuf  vmci
......
```

4.打开USB设备的可执行权限：

```shell
   $ sudo chmod 777 /dev/ttyUSB0
```

## 编译imu_ros2工作空间

1. 打开终端进入/forsense_driver/ahrs_imu_ros2目录
2. 执行`colcon build`命令，编译成功后出现如下信息。

```shell
linux@ubuntu20:~/forsense_driver/ahrs_imu_ros2$ colcon build
Starting >>> imu_ros2
Finished <<< imu_ros2 [0.24s]                  

Summary: 1 package finished [0.35s]
linux@ubuntu20:~/serial_imu_ws$ 
```

## 3. 修改串口波特率和设备号

1. 在Ubuntu环境中，支持的波特率为115200, 460800, 921600。本例程使用的默认波特率是460800，默认打开的串口名称是/dev/ttyUSB0。	

2. 如果您需要更高的输出频率，请修改`config/hipnuc_config.yaml`文件中的配置参数。	

```c
IMU_publisher:
    ros__parameters:
        serial_port: "/dev/ttyUSB0"
        baud_rate: 460800
        frame_id: "base_link"
        imu_topic: "/IMU_data"
```

注意修改后需要回到serial_imu_ws目录下，重新执行`colcon build`命令

## 显示数据
本例程提供了一种查看数据方式：

​	1、输出ROS 定义的sensor_msgs::Imu。

###  输出ROS标准 Imu.msg

1. 打开终端，执行：

```shell
linux@ubuntu20:~$ ros2 launch imu_ros2 imu_spec_msg.launch.py
```

​	2.如果执行失败，提示找不到相应的launch文件，则需要配置环境，在当前终端执行：

```shell
linux@ubuntu:~$source ./install/setup.bash
```

​	3.执行成功后，就可以看到所有的信息：

```c
[listener-2] ---
[listener-2] header:
[listener-2] 	stamp:
[listener-2] 	  secs:1639099575
[listener-2] 	  nanosecs:538349240
[listener-2] 	frame_id:base_link
[listener-2] orientation:
[listener-2] 	x: -0.000000000000000000
[listener-2] 	y: -0.000000000000000000
[listener-2] 	z: 0.000000000000000000
[listener-2] 	w: 0.000000000000000000
[listener-2] orientation_covariance: [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
[listener-2] angular_velocity: 
[listener-2] 	x: -0.000815955184543841
[listener-2] 	y: -0.001057390143056437
[listener-2] 	z: 0.001062464062371403
[listener-2] angular_velocity_covariance: [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
[listener-2] linear_acceleration:
[listener-2] 	x: 8.110355603694916482
[listener-2] 	y: -2.125157430768013000
[listener-2] 	z: 5.013053989410400924
[listener-2] linear_acceleration_covariance: [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
[listener-2] ---
```



