#include "ros/ros.h"
#include <iostream>
#include <cstdlib>
#include <ctime>
#include "PID.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

using namespace std;

class Test
{
    public:
        Test();
        void TestCB(const ros::TimerEvent&);
        void SpeedCB(const geometry_msgs::Vector3Ptr speedMsg);
        void OdomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void getSetSpeedCB(const ros::TimerEvent&);
    private:
        PID pid_speed;
        ros::NodeHandle nd;
        ros::Publisher pub, pub_twist;
        ros::Subscriber sub_flag, sub_odom;
        ros::Timer timer1,timer2;
        geometry_msgs::Twist cmd_vel, twist_speed;
        nav_msgs::Odometry odom;
        int start_flag;
        double speed_array[3];
        double P,I,D;
        double speedChange_freq,controller_freq;
        double maxDutySpeed;
};

Test::Test()
{
    ros::NodeHandle pn("~");
    pn.param("P",P,3.0);
    pn.param("I",I,2.0);
    pn.param("D",D,0.0);
    pn.param("controller_freq",controller_freq,20.0);
    pn.param("speedChange_freq",speedChange_freq,20.0);
    pn.param("maxDutySpeed",maxDutySpeed,1660.0);
    pub = nd.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);//发布占空比速度
    pub_twist = nd.advertise<geometry_msgs::Twist>("car/twist_speed", 1); //发布线速度 m/s
    sub_flag = nd.subscribe("/car/speed", 1, &Test::SpeedCB, this); //获取电调标志
    sub_odom = nd.subscribe("/odometry/filtered", 1, &Test::OdomCB, this);
    timer1 = nd.createTimer(ros::Duration((1.0)/controller_freq), &Test::TestCB, this);
    timer2 = nd.createTimer(ros::Duration((1.0)/speedChange_freq), &Test::getSetSpeedCB, this);
    double temp_array[3] = {2.0, 3.0, 3.0};
    cmd_vel.linear.x = 1565;
    odom.twist.twist.linear.x = 0;
    start_flag = 0;
    speed_array[0] = temp_array[0];
    speed_array[1] = temp_array[1];
    speed_array[2] = temp_array[2];
    PIDInit(&pid_speed);
}

void Test::SpeedCB(const geometry_msgs::Vector3Ptr speedMsg)
{
    start_flag = (*speedMsg).z;
}
void Test::OdomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    odom = *odomMsg;
}
void Test::getSetSpeedCB(const ros::TimerEvent&)
{
    srand(time(NULL));
    int index = rand() % 2;
    twist_speed.linear.x = speed_array[index];
}

void Test::TestCB(const ros::TimerEvent&)
{
    if(!start_flag)
    {
        cmd_vel.linear.x = 1545;
        cmd_vel.angular.z = 90;
        twist_speed.linear.x = 0;
        pub.publish(cmd_vel);
        // pub_twist.publish(twist_speed);
    }
    else 
    {
        double u = twist_speed.linear.x - odom.twist.twist.linear.x;
        cmd_vel.linear.x = 1500 + PIDCal(&pid_speed, u, cmd_vel.linear.x);
        cmd_vel.angular.z = 90;
        cmd_vel.linear.x = cmd_vel.linear.x > maxDutySpeed ? maxDutySpeed : cmd_vel.linear.x;
        /*---------------openLoop------------*/
        if(u < -0.3)
        {
           cmd_vel.linear.x = 1300;
        }
        if(u > 0.5)
        {
            cmd_vel.linear.x = maxDutySpeed;
        }
        twist_speed.linear.y = odom.twist.twist.linear.x;
        ROS_INFO("gas = %.2f\tsetSpeed = %.2f", cmd_vel.linear.x, twist_speed.linear.x);
        pub.publish(cmd_vel);
        pub_twist.publish(twist_speed);

    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PIDTest");
    Test test;
    ros::spin();
    return 0;    
}