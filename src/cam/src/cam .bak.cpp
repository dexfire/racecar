#include <ros/ros.h>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <map>
#include <time.h>
#include <cmath>
#include <geometry_msgs/Vector3.h>


using namespace cv;
using namespace std;

VideoCapture cap;
double t,fps,dis,temp,angle;
time_t start_time, cur_time;
int x,y,ww,rpy_r=0,rpy_p=0,rpy_y=0,cnt=0;
Mat img;

ros::Subscriber imu_sub;

template<typename T> string toString(const T& t){
    ostringstream oss;  //创建一个格式化输出流
    oss<<t;             //把值传递入流中
    return oss.str();  
}

void imu_fix(const geometry_msgs::Vector3& rpy){
  rpy_r = rpy.x;
  rpy_p = rpy.y;
  rpy_y = rpy.z;
  // cout<<"r:"<<rpy_r<<" p:"<<rpy_p<<" y:"<<rpy_y<<endl;
  cout<<"r:"<<rpy_r<<" p:"<<rpy_p<<" y:"<<rpy_y<<" add: "<<add<<endl;
}
void cam(const ros::TimerEvent&){
	// cnt ++;
	// if(cnt ==100 ){
 //    // time(&start_time);
 //    // do { time(&cur_time);
 //    // } while((cur_time - start_time) < 10);
 //    cap >> img;
 //    putText(img,toString(rpy_p),Point(640,100),FONT_HERSHEY_COMPLEX,2,Scalar(0, 0, 255),3,8,0);
 //    putText(img,toString(rpy_y),Point(640,200),FONT_HERSHEY_COMPLEX,2,Scalar(0, 255, 255),3,8,0);
 //    imwrite("/home/sz/poc2.jpg",img(Rect(0, 0, 1280, 720)));
 //    cout<<"ok"<<endl;
	// }
  Mat hsv,mask1,mask,dst,kernel;
  // cout<<fps<<endl;
  cap >> img;
  img = img(Rect(0, 0, 1280, 720));
  dst = Mat::zeros(img.size(), img.type());

  //颜色阈值
  cvtColor(img, hsv, CV_BGR2HSV);
  inRange(hsv, Scalar(0, 90, 50), Scalar(10, 250, 250), mask);
  inRange(hsv, Scalar(170, 90, 50), Scalar(180, 250, 250), mask1);
  mask = mask + mask1;
  for (int r = 0; r < img.rows; r++)
  {
    for (int c = 0; c < img.cols; c++)
    {
      if (mask.at<uchar>(r, c) == 255)
      {
        dst.at<Vec3b>(r, c) = img.at<Vec3b>(r, c);
      }
    }
  }
  //转换成灰度图
  cvtColor(dst,dst,COLOR_BGR2GRAY);

  //图像开闭操作,去噪
  kernel = getStructuringElement(MORPH_RECT, Size(7, 7));
  morphologyEx(dst,dst,MORPH_OPEN,kernel);
  kernel = getStructuringElement(MORPH_RECT, Size(9, 9));
  morphologyEx(dst,dst,MORPH_CLOSE,kernel);

  //二值化
  threshold(dst,dst,0,180,0);

  //轮廓提取
  vector<vector <Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(dst, contours, hierarchy,RETR_EXTERNAL, CHAIN_APPROX_NONE);

  //找到下轮廓
  map<int,int> points; 
  int value;
  for(int i=0;i<contours.size();i++)  
  {
    if(contours[i].size()<200){
      continue;
    }
    drawContours(img, contours, i,Scalar(255,255,0),1);
    for(int j=0;j<contours[i].size();j++){
      value = contours[i][j].y;
      if (points.count(contours[i][j].x) > 0)
        value = max(points[contours[i][j].x],contours[i][j].y);
      points[contours[i][j].x] = value;
    }
  }

  /***对下轮廓完成投影变换***/
  map<int, int>::iterator iter;
  int add = 1.55*(rpy_p+18)-38; 
  // Mat test(1280,1720,16); 测试用，绘制变换后的点
  Point2f srcPoint[4] = {
    cv::Point(387,374+add),
    cv::Point(882,376+add),
    cv::Point(113,529+add),
    cv::Point(1181,528+add),
  };
  Point2f dstPoint[4] = {
    cv::Point(x,y),
    cv::Point(x+ww,y),
    cv::Point(x,y+ww),
    cv::Point(x+ww,y+ww),
  };
  Mat M = cv::getPerspectiveTransform(srcPoint,dstPoint); 
  Mat Mx(3,1,CV_64FC1);
  dis = 1000;
  int min_x = 1280,min_y = 1720; //记录距离最近点
  for(iter = points.begin(); iter != points.end(); iter++) {
    Mx.at<double>(0, 0) = iter->first;
    Mx.at<double>(0, 1) = iter->second;
    Mx.at<double>(0, 2) = 1;
    Mx = cv::getPerspectiveTransform(srcPoint,dstPoint)*Mx;
    int p_x = Mx.at<double>(0,0)/Mx.at<double>(0,2) - 640;
    int p_y = 1870 - Mx.at<double>(0,1)/Mx.at<double>(0,2);
    temp = powf(pow(p_y,2)+pow(p_x,2),0.5)/300*80;
    angle = atan2(p_y,p_x)/3.1415926*180;
    if(dis > temp){
      dis = temp; 
      min_x = iter->first;
      min_y = iter->second;
    }
  }
  Point p(min_x,min_y);
  putText(img,toString(dis), p,FONT_HERSHEY_COMPLEX,2,Scalar(0, 255, 255),3,8,0);
  putText(img,toString(rpy_p),Point(640,100),FONT_HERSHEY_COMPLEX,2,Scalar(0, 0, 255),3,8,0);
  putText(img,toString(rpy_y),Point(640,200),FONT_HERSHEY_COMPLEX,2,Scalar(0, 255, 255),3,8,0);
  cout<<"Min dis:"<<dis<<endl;
  imshow("2", img);
  // imshow("1", test);

  waitKey(10);
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "cam");
  ros::NodeHandle nh;
  ros::Timer timer;

  imu_sub = nh.subscribe("/imu/rpy",1,imu_fix); //订阅imu

  x = 490;
  y = 1270;
  ww = 300;

  timer = nh.createTimer(ros::Duration(0.05),&cam); //20Hz
  cap.open(0);
  cap >> img;
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 2560); 
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720); 
  cap >> img;

  if (!cap.isOpened())
  { 
    cout<<"closed"<<endl;
    return -1;
  }
  ros::spin();
  cap.release();
}