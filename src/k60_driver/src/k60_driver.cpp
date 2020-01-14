#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <serial/serial.h>
#include <iostream>

using namespace std;
//创建一个serial类
serial::Serial sp;
geometry_msgs::Vector3 speed;
ros::Publisher pub;
ros::Subscriber sub;
char tx_en = 0;
uint8_t data[7] = {0xAA,0x00,0x00,0x00,0x00,0x00,0x55};

void send_cmd(uint16_t motor_pwm,uint16_t servo_pwm){
	data[1] = (motor_pwm & 0xFF00) >> 8;
	data[2] = (motor_pwm & 0x00FF);
	data[3] = (servo_pwm & 0xFF00) >> 8;
	data[4] = (servo_pwm & 0x00FF);
	data[5] = (data[1] + data[2] + data[3]+ data[4]) & 0xFF;
	tx_en = 1;
	// sp.write(data,7);
}
void read_(const ros::TimerEvent&){
	ros::Rate loop_rate(500); //250hz
    int cnt = 0,sync = 0;
    while(ros::ok())
    {
        cnt +=1;
        size_t n = sp.available(); //获取缓冲区内的字节数
        if(n >= 8)
        {
            uint8_t buffer[100];
            //读出数据
            n = sp.read(buffer, n);
            if(buffer[0] == 0x5c){
            	if((buffer[1]>>4) != 0xf){ //返回确认信息为错误
            		ROS_INFO("cmd ack error");
            	}
            	uint8_t v_hex = ((buffer[5] + buffer[2] + buffer[3]+ buffer[4]) & 0xff);
            	if(v_hex != buffer[6]){ //校验收到的速度
            		// cout<<hex<<int(v_hex)<<endl;
            		for(int i=0;i<n;i++)
            			cout<<hex<<int(buffer[i])<<endl;
            		ROS_INFO("speed error");
            		return;
            	}
            	speed.x = (buffer[2]<<8) + buffer[3];
				speed.x = int16_t(speed.x);
            	speed.y = (buffer[4]<<8) + buffer[5];
				speed.y = int16_t(speed.y);
            	speed.z = ((buffer[1] & 0xF) == 0xF?1:0) ;
            	pub.publish(speed);
            	if(tx_en == 1){
            		tx_en = 0;
            		sp.write(data,7);
            	}
            	ROS_INFO("l = %d,r = %d", int16_t(speed.x),int16_t(speed.y));
            }
            else
            	ROS_ERROR_STREAM("cmd error");
            return;
        }
        if(cnt >= 10){
        	ROS_INFO("time out");
            return;
		}
        loop_rate.sleep();
    }
}
void TwistCallback(const geometry_msgs::Twist& twist)
{
    double angle;
    //ROS_INFO("z= %f", twist.angular.z);
    angle = 2600.0 - twist.angular.z * 2000.0 / 180.0;
    if(angle>2500)
        angle = 2500;
    ROS_INFO("angle= %d",uint16_t(angle));
    send_cmd(uint16_t(twist.linear.x),uint16_t(angle));
}

int main(int argc, char** argv)
{
    //创建timeout
    ros::init(argc, argv, "k60_driver");
    ros::NodeHandle n;
    ros::Timer timer;
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/car");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
    	send_cmd(0,0); //单片机开始发送
    	sp.write(data,7);
        ROS_INFO_STREAM("/dev/car is opened.");
    }
    else
    {
        return -1;
    }
    timer = n.createTimer(ros::Duration(0.05),&read_); //20Hz
    sub = n.subscribe("/car/cmd_vel",1,TwistCallback);
    pub = n.advertise<geometry_msgs::Vector3>("car/speed", 1);
    ros::spin();
    sp.close();
    return 0;
}
