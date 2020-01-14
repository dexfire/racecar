#include <iostream>
#include "ros/ros.h"
#include "ADRC.h"
#include <geometry_msgs/Vector3.h> 
#include <cstdlib>
#include <ctime>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

using namespace std;


class Test
{
public:
    Test();
    void TestCB(const ros::TimerEvent&);
    void getSetSpeedCB(const ros::TimerEvent&);
    void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void flagCB(const geometry_msgs::Vector3::ConstPtr& flagMsg);
private:
    geometry_msgs::Vector3 test;
    nav_msgs::Odometry odom;
    geometry_msgs::Twist speed;
    ros::Publisher pub_,pub;
    ros::Subscriber odom_sub, flag_sub;
    ros::NodeHandle n;
    ros::Timer timer1,timer2;
    ADRC adrc;
    double v0;
    int count;
    double start_flag;
};

Test::Test()
{

    pub_ = n.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);
    pub = n.advertise< geometry_msgs::Vector3>("car/TD", 1);
    flag_sub = n.subscribe("/car/speed", 1, &Test::flagCB, this);
    odom_sub = n.subscribe("/odometry/filtered", 1, &Test::odomCB, this);
    timer1 = n.createTimer(ros::Duration((1.0)/20), &Test::TestCB, this);
    //timer2 = n.createTimer(ros::Duration((1.0)/2), &Test::getSetSpeedCB, this);
    v0 = 0;
    count = 0;
    start_flag =false;
}
void Test::flagCB(const geometry_msgs::Vector3::ConstPtr& flagMsg)
{
    start_flag = (*flagMsg).z;
}
void Test::TestCB(const ros::TimerEvent&)
{
    double y = odom.twist.twist.linear.x;
        if(start_flag)
    {  
    //     if(count++ < 40)
    // {
    //     speed.angular.z = 90;
    //     v0 = 3.2;
    // }
    // else if(count++ < 100)
    // {
    //     speed.angular.z = 90;
    //     v0 = 2.0;
    // }
    // // else if(count++ < 140)
    // // {
    // //     speed.angular.z = 90;
    // //     v0 = 3.2;
    // // }
    // // else if(count++ < 200)
    // // {
    // //     speed.angular.z = 180;
    // //     v0 = 2.0;
    // // }
    // else
    //     count = 0;
    speed.angular.z = 90;
    v0 = 2.0;
    speed.linear.x = 1500 + adrc.Calculate(y,v0);
    cout<< "gas" << speed.linear.x<<endl;    
    test.x = v0;
    test.y = adrc.getV1();
    test.z = adrc.getZ1();
    cout << " v0 = "<<v0<<" v1 = "<< test.y << " y = "<< y<<" z1 = "<<test.z<<endl;

    pub_.publish(speed);
    pub.publish(test);
    }

    
}
void Test::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    odom = *odomMsg;
}
void Test::getSetSpeedCB(const ros::TimerEvent&)
{
    srand(time(NULL));
    int index = rand() % 4;
    v0 = index;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ADRCTest");
    Test test;
    ros::spin();
    return 0;    
}