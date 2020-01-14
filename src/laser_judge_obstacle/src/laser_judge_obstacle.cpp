#include <ros/ros.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>    
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <iostream>
#include <map>
#include <time.h>
#include <cmath>
#include <geometry_msgs/Point.h>

using namespace cv;
using namespace std;
sensor_msgs::LaserScan laser;
geometry_msgs::Point  obstacle;
Mat my_map,area;
int cnt=0; 
ros::Subscriber laser_sub,odom_sub;
ros::Publisher pub;
double towards,at_x,at_y;
double obstacle_x[15],obstacle_y[15];
double obstacle_distance[20];//当前障碍与之前的每个障碍的距离
int obstacle_num = 0;
int num =0;
bool have_middle_obstacle = false;
bool have_obstacle = false;

double getYaw(const geometry_msgs::Pose& carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp,yaw;
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp,tmp, yaw);

    return yaw;
}
void odomCB(const nav_msgs::Odometry odomMsg)
{
    towards = getYaw(odomMsg.pose.pose);
    //if(towards>-1.57&&towards<1.57)
    //{towards = 0; }
    //else {towards = 3.14;}
    //at_x = odomMsg.pose.pose.position.x;
    //at_y = odomMsg.pose.pose.position.y;
}


bool ifInArea(double angle, double distance)
{
    int x,y,y_min = -1000,y_max = 1000;
    //cout<<"angle: "<<angle<<" dis:"<<distance<<endl;
    angle = angle/180.0*3.1415926 + towards;
    x = distance*20.0*sin(angle)+at_x+40; //\u56de\u7a0bangle\u5e94\u8be5\u4e3a\u8d1f(angle\u53d6\u503c-180\u5230180)
    y = -distance*20.0*cos(angle)+at_y+100;
    //cout<<"Car at x:"<<at_x<<" y:"<<at_y<<" towards :"<<towards<<endl;
    //cout<<"Point at x:"<<x<<" y:"<<y<<" angle :"<<angle<<endl;
    if(my_map.at<uchar>(y, x) < 250)
    	return false;
    for(int i=0;i<33;i++){ //\u5de6\u53f31.5m
        if(my_map.at<uchar>(y-i, x) < 10 && y-i > y_min)
            y_min = y-i;
        if(my_map.at<uchar>(y+i, x) < 10 && y+i < y_max)
            y_max = y+i;
    }
    if(y_min == -1000 || y_max == 1000)
    	return false;
    //cout<<"ymin: "<<y_min<<" ymax: "<<y_max<<endl;	
    if(y_max - y < 5 && y - y_min < 5)
        return false;
    return true;
}

void laserCB(const sensor_msgs::LaserScan::ConstPtr& laserMsg)
{
    map<int,int>  points;
    laser = *laserMsg;
    cnt = 50;
    
    int in = 0,start = 0,obstacle_point = 0;
    int j,hei,px=50,k;
    double xl,yl,angle;
    laser.ranges[180] = laser.ranges[90];
    have_middle_obstacle == false;
    for(int i = 140;i >=40;i--){
        if(i <= 89)
            j = i + 270;
        else
            j = i - 90;
        laser.ranges[i] = laser.ranges[j];
	    angle = i/180.0*3.1415926 + towards;
        xl = laser.ranges[i]*cos(angle);
        yl = abs(laser.ranges[i]*sin(angle));
        if(yl > 6 || yl < 0 || xl > 2 || xl < -2)
            continue;
        if(laser.ranges[j] < 2.5 && laser.ranges[i+1] - laser.ranges[i] > 0.4){
            in = 1;
            start = i;
            continue;
        }
	    k=j-1;
        if(k<0){
            k=359;}
        if(in ==1 && laser.ranges[j] < 2.5 && laser.ranges[k] - laser.ranges[j] > 0.4){ //j-1不能小于0
            in = 0;
            if(fabs(i - start) > (0.15*360/(2*laser.ranges[i]*3.1415926))+2)
                continue;
            if(!((ifInArea(i,laser.ranges[i])) && ifInArea(start,laser.ranges[start])))
                continue;
                points[start] = laser.ranges[j];
                cout<<"start: "<<start<<" end: "<<i<<" len: "<<laser.ranges[j]<<endl;
                obstacle_point = (i+start)/2;
                angle = obstacle_point/180.0*3.1415926 + towards;
                obstacle.x = laser.ranges[obstacle_point]*sin(angle)+at_x; 
                obstacle.y = -(laser.ranges[obstacle_point]*cos(angle))+at_y;
                num = 0;
                for (k = obstacle_num; k>=0; k--){
                    obstacle_x[0]=0;
                    obstacle_y[0]=0;
                    obstacle_distance[k] = fabs((obstacle.x - obstacle_x[k])*(obstacle.x - obstacle_x[k])+
                    (obstacle.y - obstacle_y[k])*(obstacle.y - obstacle_y[k])); 
                     //cout<<"bstacle_distance: "<< obstacle_distance[k]  <<endl;
                    if(obstacle_distance[k] > 1.0){
                        num++;
                    }
                }
                    if (num > obstacle_num){
                        obstacle_num++;
                        //num = 0;
                        if(obstacle_num>=13){
                            obstacle_num =13;
                        } 
                        obstacle_x [obstacle_num] = obstacle.x;
                        obstacle_y [obstacle_num] = obstacle.y; 
                    }    
        }
        //cout<<"num: "<< num <<endl;
        cout<<"obstacle_num: "<< obstacle_num <<endl;
        cout<<"at_x: "<<at_x<<endl;
        cout<<"at_y: "<<at_y<<endl;
        cout<<"towards: "<<towards<<endl;
        cout<<"angle: "<<angle<<endl;
        for (k = obstacle_num; k>=0; k--){
        cout<<"obstacle_x: "<< obstacle_x[k]<<endl;
        cout<<"obstacle_y: "<< obstacle_y[k]<<endl;
        }
        for (k = obstacle_num; k>=0; k--){
            if(obstacle_y[k]>-0.14&&obstacle_y[k]<0.14&&obstacle_x[k]<7.5&&obstacle_x[k]>2){
                have_middle_obstacle = true;
                obstacle.x = obstacle_x [k];
                obstacle.y = obstacle_y [k];
                for (k = obstacle_num; k>0; k--){
                    if ((obstacle_x[k]-obstacle.x)<0){
                         have_middle_obstacle = false;
                         //break;
                   }
                }
            }
        }
    }  
    if(have_middle_obstacle == false){
    obstacle.x = 0.0; 
    obstacle.y = 0.0;
    }

    pub.publish(obstacle);
    //cout<<"obstacle_num: "<<obstacle_num<<endl;
}
void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
     tf::StampedTransform transform;
    try
    {
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
   at_x = transform.getOrigin().x();
   at_y = transform.getOrigin().y();
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_judge_obstacle");
  ros::NodeHandle nh;
  ros::Timer timer;
  my_map = imread("/home/sz/racecar/src/art_racecar/map/test.pgm");
  my_map = my_map(Rect(1960, 1900, 600, 200));
  imwrite("/home/sz/aaa.png",my_map);
  cvtColor(my_map,my_map,COLOR_BGR2GRAY);
  laser_sub = nh.subscribe("/scan", 1, laserCB);
  odom_sub = nh.subscribe("/odometry/filtered", 1,odomCB);
  pub = nh.advertise<geometry_msgs::Point>("/obstacle", 1);
  //timer = nh.createTimer(ros::Duration(0.05),&cam); //20Hz
  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  timer = nh.createTimer(ros::Duration(0.1), boost::bind(&transformPoint, boost::ref(listener)));

  cout<<"Done init, running"<<endl;
  ros::spin();
}
