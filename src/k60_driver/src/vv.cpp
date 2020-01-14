#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <serial/serial.h>
#include <iostream>
#include <math.h>
#include <string.h>
#include "k60_driver.hpp"

using namespace std;

Driver::Driver(){
	data[0] = 0xAA;
	data[6] = 0x55;
	serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort("/dev/car");
    sp.setBaudrate(115200);
    sp.setTimeout(to);
    try
    {
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
    }
    
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/car is opened.");
    }
    else
    {
        ROS_INFO("Unknow Error");
    }
	status = odom_err = speed_err = 0;
	cmd_sub = n_.subscribe("/car/cmd_vel",1,&Driver::cmdCB,this); //订阅控制消息
    odom_sub = n_.subscribe("/odometry/filtered", 1, &Driver::odomCB,this); //订阅里程计
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &Driver::goalCB,this); //订阅设定的目标
    laser_sub = n_.subscribe("/scan", 1, &Driver::laserCB,this); //订阅雷达扫描信息

    speed_pub = n_.advertise<geometry_msgs::Vector3>("car/speed", 1);
    pose_pub = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    timer1 = n_.createTimer(ros::Duration(0.05), &Driver::recoveryLoop, this); //20Hz
    timer2 = n_.createTimer(ros::Duration(0.025), &Driver::checkLoop, this); //40Hz
}

void Driver::send_cmd(uint16_t motor_pwm,uint16_t servo_pwm){
	data[1] = (motor_pwm & 0xFF00) >> 8;
	data[2] = (motor_pwm & 0x00FF);
	data[3] = (servo_pwm & 0xFF00) >> 8;
	data[4] = (servo_pwm & 0x00FF);
	data[5] = (data[1] + data[2] + data[3]+ data[4]) & 0xFF;
	sp.write(data,7);
	read_speed();
}
void Driver::read_speed()
{
	ros::Rate loop_rate(500); //500hz
    int cnt = 0;
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        cnt +=1;
        size_t n = sp.available();
        if(n >= 8)
        {
            uint8_t buffer[100];
            //读出数据
            n = sp.read(buffer, n);
            if(buffer[0] == 0x5C){
            	if((buffer[1]>>4) != 0xF){ //返回确认信息为错误
            		ROS_ERROR_STREAM("cmd ack error");
            		cout << buffer[1] << endl;
            	}
            	if(((buffer[5] + buffer[2] + buffer[3]+ buffer[4]) & 0xFF) != buffer[6]){ //校验收到的速度
            		ROS_ERROR_STREAM("recv speed error");
            		return;
            	}
            	speed.x = (buffer[2]<<8) + buffer[3];
            	speed.x = int16_t(speed.x);
            	speed.y = (buffer[4]<<8) + buffer[5];
            	speed.y = int16_t(speed.y);
            	speed.z = ((buffer[1] & 0xF) == 0xF?1:0) ;
            	for(int i=99; i>0 ;i--)
            		speeds[i] = speeds[i-1];
            	speeds[0] = (speed.x+speed.y)/2;
            	if(speeds[1] - speeds[0] > 800){ //改
            		speed_err += 1;
            		ROS_INFO("speed drop");
            	}else{
            		speed_err = 0;
            	}
            	speed_pub.publish(speed);
            	ROS_INFO("l = %d,r = %d", (int)speed.x,(int)speed.y);
            }
            else
            	ROS_ERROR_STREAM("cmd error");
            return;
        }
        if(cnt >= 5)
            return;
        loop_rate.sleep();
    }
}

void Driver::laserCB(const sensor_msgs::LaserScan::ConstPtr& laserMsg)
{
	memset(area,0,12*2*sizeof(int));
	int min_len = (int)(360/12);
	laser = *laserMsg;
	char in = 0,start = 0,sta = 0;
	int cnt;
	max_dis = 0;
	for(int i = 150;i<=210;i++){
		if(laser.ranges[i-1] < 6 && laser.ranges[i] > 100){
			sta = 1; //天线
			continue;
		}
		if(sta == 1 && laser.ranges[i] > 100)
			continue;
		sta = 0;
		if(laser.ranges[i] > max_dis){
			max_dis = laser.ranges[i];
			max_dir = (i-180); //距离最远的方向，范围：-30 -- 30
		}
	}
	int j;
	for(int i = 0;i<180;i++){
		if(i <= 89)
			j = i + 270;
		else
			j = i - 90;
		if(in ==0 && laser.ranges[j] < 0.2){
			in = 1;
			start = i;
			continue;
		}
		if(in = 0 && i > 0 && laser.ranges[j-1] > 0.5 && laser.ranges[j] > 100){
			in = 1;
			start = i;
			continue;
		}
		if(in = 1 && (laser.ranges[j] < 0.3 || laser.ranges[j] > 100))
			continue;
		if(in ==1 && i-start > 20){
			in = 0;
			area[cnt][0] = start;
			area[cnt][1] = i;
			cout<<cnt<<endl;
			cnt ++;
		}
	}
}

void Driver::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
	last_odom = cur_odom;
	cur_odom = *odomMsg; //应当是准确的
	float len = pow(sqrt(cur_odom.pose.pose.position.x-last_odom.pose.pose.position.x)+sqrt(cur_odom.pose.pose.position.y-last_odom.pose.pose.position.y),0.5);
	if(len > lengths[0]+lengths[1]){ //这里是异常判断
		odom_err += 1;
		ROS_INFO("Odom error");
		return;
	}
	odom = cur_odom;
	odom_err = 0;
	for(int i=99;i>0;i--)
		lengths[i] = lengths[i-1];
	lengths[0] = len;
}
void Driver::cmdCB(const geometry_msgs::Twist& twist)
{
    double angle;
    angle = 2500.0 - twist.angular.z * 2000 / 180.0;
    if(status == 0)
    	send_cmd(uint16_t(twist.linear.x),uint16_t(angle));
}
void Driver::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
	goal = *goalMsg;
}
void Driver::relocate(){
	initialpose.pose.pose = odom.pose.pose;
	for(int i;i<36;i++)
		initialpose.pose.covariance[i] = 0;
	initialpose.pose.covariance[0] = 0.25;
	initialpose.pose.covariance[7] = 0.25;
	initialpose.pose.covariance[35] = 0.06853891945200942;
	initialpose.header.seq = 1;
	initialpose.header.frame_id = "map";

}
void Driver::checkLoop(const ros::TimerEvent&){
	int cnt = 0;
	int angle;
	if((odom_err > 0 || speed_err > 0) && status == 0){
		if(area[0][1] == 0){
			ROS_INFO("LaserScan well"); //雷达具有最终决定权，选择更改
			return;
		}
		//status = 1;
		for(int i = 0;i < 12;i++){
			if(area[i][0] == 0 && i > 0)
				break;
			angle += (area[i][0]+area[i][1])/2;
			cnt ++;
			//ROS_INFO("Area: %d",cnt);
		}
	}else if(status > 0){
		distance += speeds[0]/1000*0.05;
		if(distance < -0.8) //倒车
			odom_err = speed_err = status = 0;
	}
	else
		return;
}
void Driver::recoveryLoop(const ros::TimerEvent&){
	if(status == 1)
	{
		cout << "dir :"<<max_dir <<endl;
	    int angle = 2500.0 - max_dir * 2000 / 50.0;
	    send_cmd(1420,angle);
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "k60_driver");

    Driver driver;
  	ros::AsyncSpinner spinner(4); // Use 4 threads
  	spinner.start();
 	ros::waitForShutdown();
    return 0;
}
