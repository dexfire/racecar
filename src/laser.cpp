#include <ros/ros.h>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>
#include <iostream>
#include <map>
#include <time.h>
#include <cmath>
#include <geometry_msgs/Point.h>

using namespace cv;
using namespace std;
sensor_msgs::LaserScan laser;
VideoCapture cap;
int cnt=0,at_x,at_y;
Mat img,my_map,area;
VideoWriter out; 
ros::Subscriber laser_sub,odom_sub;
ros::Publisher pub;
double towards;

template<typename T> string toString(const T& t){
    ostringstream oss;  //创建一个格式化输出流
    oss<<t;             //把值传递入流中
    return oss.str();  
}

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
    at_x = odomMsg.pose.pose.position.x*20;
    at_y = odomMsg.pose.pose.position.y*20;
}

void transpose(Mat src, Mat &dst)
{
	int row = src.rows;
	int col = src.cols;
	dst.create(src.cols, src.rows, src.type());
	for (int i = 0; i < src.rows; i++){
		for (int j = 0; j < src.cols; j++){
			dst.at<Vec3b>(j, i) = src.at<Vec3b>(i,j);
		}
	}
}

bool ifInArea(double angle, double distance)
{
    int x,y,y_min = -1000,y_max = 1000;
    //cout<<"angle: "<<angle<<" dis:"<<distance<<endl;
    angle = angle/180.0*3.1415926 + towards;
    x = distance*20.0*sin(angle)+at_x+40; //回程angle应该为负(angle取值-180到180)
    y = distance*20.0*cos(angle)+at_y+100;
    //cout<<"Car at x:"<<at_x<<" y:"<<at_y<<" towards :"<<towards<<endl;
    //cout<<"Point at x:"<<x<<" y:"<<y<<" angle :"<<angle<<endl;
    if(my_map.at<uchar>(y, x) < 250)
    	return false;
    for(int i=0;i<30;i++){ //左右1.5m
        if(my_map.at<uchar>(y-i, x) < 10 && y-i > y_min)
            y_min = y-i;
        if(my_map.at<uchar>(y+i, x) < 10 && y+i < y_max)
            y_max = y+i;
    }
    if(y_min == -1000 || y_max == 1000)
    	return false;
    cout<<"ymin: "<<y_min<<" ymax: "<<y_max<<endl;
    area = my_map(Rect(x-30,y-30 ,60,60 ));
  	cvtColor(area,area,COLOR_GRAY2BGR);
  	Mat patch;
  	transpose(area,patch);
  	patch.copyTo(img(Rect(610,600,60,60)));
    if(y_max - y < 5 && y - y_min < 5)
        return false;
    return true;
}

void laserCB(const sensor_msgs::LaserScan::ConstPtr& laserMsg)
{
    map<int,int>  points;
    laser = *laserMsg;
    cap >> img;
    cnt = 50;
    img = img(Rect(0, 0, 1280, 720));
    int in = 0,start = 0;
    int j,hei,px=50;
    double xl,yl,angle;
    int pppx,pppy;
    laser.ranges[180] = laser.ranges[90];
    for(int i = 120;i >=60;i--){
        if(i <= 89)
            j = i + 270;
        else
            j = i - 90;
        laser.ranges[i] = laser.ranges[j];
        hei = 650-laser.ranges[i]/8.0*550.0;
        if(hei<100)
          hei = 100;
        circle(img,Point(px,hei),3,Scalar(255,255,0),-1);
        px += 6;

        angle = i/180.0*3.1415926 + towards;
        xl = laser.ranges[i]*cos(angle);
        yl = abs(laser.ranges[i]*sin(angle));
        pppx = 600 + (xl+2.0)/4.0*600.0;
        pppy = 600 - yl/10.0*500.0;

        circle(img,Point(pppx,pppy),3,Scalar(255,0,255),-1);
        if(yl > 6 || yl < 0 || xl > 2 || xl < -2)
            continue;
        if(laser.ranges[j] < 3.5 && laser.ranges[i+1] - laser.ranges[i] > 0.4){
            in = 1;
            start = i;
            continue;
        }
        if(in ==1 && laser.ranges[j] < 3.5 && laser.ranges[j-1] - laser.ranges[j] > 0.4){ 
            in = 0;
            if(i - start > (0.15*360/(2*laser.ranges[i]*3.1415926))+2)
                continue;
            if(!((ifInArea(i,laser.ranges[i])) && ifInArea(start,laser.ranges[start])))
                continue;
            points[start] = laser.ranges[j];
            //cout<<"start: "<<start<<" end: "<<i<<" len: "<<laser.ranges[j]<<endl;
            cnt += 80;
            putText(img,toString(start),Point(100,cnt),FONT_HERSHEY_COMPLEX,2,Scalar(255, 0, 0),3,8,0);
            putText(img,toString(i),Point(300,cnt),FONT_HERSHEY_COMPLEX,2,Scalar(0, 0, 255),3,8,0);
            angle = start/180.0*3.1415926 + towards;
            xl = laser.ranges[start]*cos(angle);
            yl = abs(laser.ranges[start]*sin(angle));
            pppx = 600 + (xl+2.0)/4.0*600.0;
            pppy = 600 - yl/10.0*500.0;

            angle = i/180.0*3.1415926 + towards;

            circle(img,Point(pppx,pppy),3,Scalar(255,0,0),-1);
            xl = laser.ranges[i]*cos(angle);
            yl = abs(laser.ranges[i]*sin(angle));
            pppx = 600 + (xl+2.0)/4.0*600.0;
            pppy = 600 - yl/10.0*500.0;

            circle(img,Point(pppx,pppy),3,Scalar(255,0,0),-1);
        }

    }
    out<<img;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cam");
  ros::NodeHandle nh;
  ros::Timer timer;
  my_map = imread("/home/sz/racecar/src/art_racecar/map/test.pgm");
  my_map = my_map(Rect(1960, 1900, 600, 200));
  imwrite("/home/sz/aaa.png",my_map);
  cvtColor(my_map,my_map,COLOR_BGR2GRAY);
  laser_sub = nh.subscribe("/scan", 1, laserCB);
  odom_sub = nh.subscribe("/odometry/filtered", 1,odomCB);

  //timer = nh.createTimer(ros::Duration(0.05),&cam); //20Hz
  cap.open(0);
  cap >> img;
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 2560); 
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720); 
  cap >> img;
  cout<<"Done init, running"<<endl;
  out.open("/home/sz/out1.avi", CV_FOURCC('X', 'V', 'I', 'D'),10,Size(1280,720));
  if (!cap.isOpened())
  { 
    cout<<"closed"<<endl;
    return -1;
  }
  ros::spin();
  cap.release();
  out.release();
}