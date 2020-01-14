/*
Copyright (c) 2017, ChanYuan KUO, YoRu LU,
latest editor: HaoChih, LIN
All rights reserved. (Hypha ROS Workshop)

This file is part of hypha_racecar package.

hypha_racecar is free software: you can redistribute it and/or modify
it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published
by the Free Software Foundation, either version 3 of the License, or
any later version.

hypha_racecar is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU LESSER GENERAL PUBLIC LICENSE for more details.

You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
along with hypha_racecar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "art_car_controller.hpp"
#include "PID.h"
#include "ADRC.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#define PI 3.14159265358979

int start_loop_flag = 0;
int start_speed = 1560;
PID  pid_speed;
int straight_count = 0, turn_count = 0, U_count = 0, speed_count = 0;
int loop = 1;
int num = 0;
bool flag = false;
using namespace std;

L1Controller::L1Controller()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Car parameter
    pn.param("L", L, 0.26);
    pn.param("Lrv", Lrv, 10.0);
    pn.param("Vcmd", Vcmd, 1.0);
    pn.param("lfw", lfw, 0.13);
    pn.param("lrv", lrv, 10.0);

    //Controller parameter
    pn.param("controller_freq", controller_freq, 20);
    pn.param("AngleGain", Angle_gain, -1.0);
    pn.param("GasGain", Gas_gain, 1.0);
    pn.param("baseSpeed", baseSpeed, 1575);
    pn.param("baseAngle", baseAngle, 90.0);
    pn.param("setTurnAngle", setTurnAngle, 20.0);
    pn.param("turnSpeed", turnSpeed, 1600);
    pn.param("maxDutySpeed", maxDutySpeed, 1660.0);
    pn.param("minDutySpeed", minDutySpeed, 1200.0);
    pn.param("coe", coe, 5.0);
    pn.param("speed_U", speed_U, 1.8);
    pn.param("speed_90", speed_90, 1.8);
    pn.param("speed_120", speed_120, 3.2);
    pn.param("speed_turn", speed_turn, 2.5);
    pn.param("speed_straight", speed_straight, 3.2);

    //Publishers and Subscribers
    odom_sub = n_.subscribe("/odometry/filtered", 1, &L1Controller::odomCB, this);
    path_sub = n_.subscribe("/move_base/GlobalPlanner/plan", 1, &L1Controller::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);
    speed_sub = n_.subscribe("/car/speed", 1, &L1Controller::speedCB, this);
    obstacle_sub = n_.subscribe("/obstacle", 1, &L1Controller::obstacleCB, this);
    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 10);
    pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);

    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &L1Controller::controlLoopCB, this); // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.5)/controller_freq), &L1Controller::goalReachingCB, this); // Duration(0.05) -> 20Hz

    //Init variables
    Lfw =  getL1Distance(Vcmd);
    goalRadius = 1.5;
    last_eta = 0, eta = 0;
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;
    isU = false;
    isTurn_90 = false;
    isTurn_120 = false;
    isStraight = true;
    turn_left = turn_right = false;
    start_flag=0;//电调是否打开的标志
    cmd_vel.linear.x = 1500; // 1500 for stop
    cmd_vel.angular.z = baseAngle;
    pathLock = false;
    path_temp = 1;
    loop_count = 0;
    SteeringAngle = 0;

    //Show info
    ROS_INFO("[param] baseSpeed: %d", baseSpeed);
    ROS_INFO("[param] baseAngle: %f", baseAngle);
    ROS_INFO("[param] AngleGain: %f", Angle_gain);
    ROS_INFO("[param] Vcmd: %f", Vcmd);
    ROS_INFO("[param] Lfw: %f", Lfw);
    ROS_INFO("[param] turnSpeed: %d", turnSpeed);
    ROS_INFO("[param] setTurnAngle: %f", setTurnAngle);

    //Visualization Marker Settings
    initMarker();
    PIDInit(&pid_speed);
   // ADRC adrc;
    car_stop = 0;
    speedSum = 0;
}

void L1Controller::obstacleCB(const geometry_msgs::Point::ConstPtr& obstacleMsg)
{
	obs_point = *obstacleMsg;
}
void L1Controller::initMarker()
{
    start_point.header.frame_id = points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
    start_point.ns = points.ns = line_strip.ns = goal_circle.ns = "Markers";
    start_point.action = points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    start_point.pose.orientation.w = points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;
    start_point.id = 3;

    points.type = visualization_msgs::Marker::POINTS;
    start_point.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    start_point.scale.x = 0.2;
    start_point.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goalRadius*2;
    goal_circle.scale.y = goalRadius*2;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.r = 1.0;
    points.color.a = 1.0;

    start_point.color.b = 1.0;
    start_point.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}


void L1Controller::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    odom = *odomMsg;
}


void L1Controller::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
	path = *pathMsg;
}

void L1Controller::speedCB(const geometry_msgs::Vector3::ConstPtr& speedMsg)
{
    start_flag = (*speedMsg).z;
}
void L1Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    try
    {
        geometry_msgs::PoseStamped odom_goal;
        tf_listener.transformPose("odom", ros::Time(0) , *goalMsg, "map" ,odom_goal);//将map上的goal转化到odom
        odom_goal_pos = odom_goal.pose.position;
        goal_received = true;
        goal_reached = false;

        /*Draw Goal on RVIZ*/
        goal_circle.pose = odom_goal.pose;
        marker_pub.publish(goal_circle);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

double L1Controller::getYawFromPose(const geometry_msgs::Pose& carPose)
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

// bool L1Controller::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
// {
//     float car2wayPt_x = wayPt.x - carPose.position.x;
//     float car2wayPt_y = wayPt.y - carPose.position.y;
//     double car_theta = getYawFromPose(carPose);

//     float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;
//     float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;

//     if(car_car2wayPt_x >0) /*is Forward WayPt*/
//         return true;
//     else
//         return false;
// }
bool L1Controller::isForwardPath(const geometry_msgs::Point& car_pos)
{
    double fwdPt_pose_yaw = getYawFromPose(fwdPt_pose);//获取yaw
    geometry_msgs::Point fwdPt2car;
    fwdPt2car.x = cos(fwdPt_pose_yaw)*(car_pos.x - fwdPt_pose.position.x) + sin(fwdPt_pose_yaw)*(car_pos.y - fwdPt_pose.position.y);
    fwdPt2car.y = -sin(fwdPt_pose_yaw)*(car_pos.x - fwdPt_pose.position.x) + cos(fwdPt_pose_yaw)*(car_pos.y - fwdPt_pose.position.y);
    if(fwdPt2car.x < 0) //向前的路径
        return true;
    else
        return false;
}


bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < Lfw)
        return false;
    else if(dist >= Lfw)
        return true;
}

geometry_msgs::Point L1Controller::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;//当前坐标
    double carPose_yaw = getYawFromPose(carPose);//获取yaw
    geometry_msgs::Point forwardPt, endpoint;
    geometry_msgs::Point odom_car2WayPtVec;
    geometry_msgs::Pose thisPose, lastPose, startPose;
    double twoPoseAngle;
    foundForwardPt = false; 
    AvgCurvature = 0;
    double sumCurvature = 0; 
    geometry_msgs::Point startpoint;
    if(!goal_reached)
    {
	 //    if(fwdPt_pose_map.position.x > 4 && fwdPt_pose_map.position.x < 6)	
		// 	pathLock = true;
		// else
		// 	pathLock = false;
		// if(!pathLock)
		// {
		// 	lock_path = path;
		// }
  //       else
  //           cout<<"path is lock"<<endl;
        map_path = path;
        for(int i = 0; i< map_path.poses.size(); i++)
        {
            try
            {  
                geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];//路径上的点(map)
                geometry_msgs::PoseStamped odom_path_pose, pose_3m;//路径上的点(odom) 
                tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                lastPose = thisPose;
                thisPose = odom_path_pose.pose;
                if(i == 20)
                {
                    startPose = odom_path_pose.pose;
                    startpoint = odom_path_wayPt;
                }
                if(i > 20)
                {
                    double dx = thisPose.position.x-lastPose.position.x;
                    double dy = thisPose.position.y-lastPose.position.y;
                    double ds = sqrt(dx * dx + dy * dy);
                    twoPoseAngle = getTwoPoseAngle(lastPose, thisPose);
                    sumCurvature += getCurvature(twoPoseAngle, ds);
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);//判断路径上的点是不是在lfw距离外
                     if(_isWayPtAwayFromLfwDist) //找到前瞻点
                    {
                    	fwdPt_pose = odom_path_pose.pose;//possible wrong
                        bool _isForwardPath = isForwardPath(carPose_pos);
                        if(!_isForwardPath)
                            continue;
                        forwardPt = odom_path_wayPt;
                        foundForwardPt = true;
                        AvgCurvature = sumCurvature / (i - 20);
                        fwdPtPose2startPoseAngle = getfwdPtPose2startPoseAngle(startPose);
                        fwdPt_pose_map = map_path.poses[i].pose;
                        if(i + 30 < map_path.poses.size())
                        {
                            tf_listener.transformPose("odom", ros::Time(0) , map_path.poses[i + 30], "map" ,pose_3m);
                            endpoint = pose_3m.pose.position;
                            angle_3m = fabs(getYawFromPose(pose_3m.pose));
                        }
                        else
                            angle_3m = PI;
                        cout <<"x = "<< fwdPt_pose_map.position.x<< " y = "<< fwdPt_pose_map.position.y<<endl;
                        break;
                    }
                }

            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }
        
    }
    else if(goal_reached)
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
        //ROS_INFO("goal REACHED!");
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    start_point.points.clear();
    line_strip.points.clear();
    
    if(foundForwardPt && !goal_reached)
    {
        start_point.points.push_back(startpoint);
        start_point.points.push_back(endpoint);
        points.points.push_back(carPose_pos);
        points.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }
    
    marker_pub.publish(points);
    marker_pub.publish(start_point);
    marker_pub.publish(line_strip);
    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    return odom_car2WayPtVec;
}


double L1Controller::getEta(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);

    double eta = atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
    return eta;
}


double L1Controller::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = odom.pose.pose.position;//车子当前位置
    double car2goal_x = odom_goal_pos.x - car_pose.x;//目标位置－当前位置＝离目标的距离
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);

    return dist2goal;
}

double L1Controller::getL1Distance(const double& _Vcmd)
{
    double L1 = 0;
    if(_Vcmd < 1.34)
        L1 = 3 / 3.0;
    else if(_Vcmd > 1.34 && _Vcmd < 5.36)
        L1 = _Vcmd*2.24 / 3.0;
    else
        L1 = 12 / 3.0;
    return L1;
}

double L1Controller::getSteeringAngle(double eta)
{
    double steeringAnge = -atan2((L*sin(eta)),(Lfw/2+lfw*cos(eta)))*(180.0/PI);
    //ROS_INFO("Steering Angle = %.2f", steeringAnge);
    return steeringAnge;
}

double L1Controller::getGasInput(const double& setSpeed, const float& current_v)
{
    double u = (setSpeed - current_v)*Gas_gain;
    //ROS_INFO("velocity = %.2f\tu = %.2f",current_v, u);
    return u;
}
void L1Controller::goalReachingCB(const ros::TimerEvent&)
{

    if(goal_received)
    {
        double car2goal_dist = getCar2GoalDist();
        if(car2goal_dist < goalRadius)
        {
            goal_reached = true;
            goal_received = false;
            ROS_INFO("Goal Reached !");
            car_stop = 100;
        }
    }
}

// double L1Controller::getfwdPtPose2carPoseAngle(const geometry_msgs::Pose& carPose)
// {
//     double fwdPt_pose_yaw = getYawFromPose(fwdPt_pose);//fwdPt_pose前瞻点位姿
//     double carPose_yaw = getYawFromPose(carPose);
//     double fwdPtPose2carPoseAngle = fabs(fwdPt_pose_yaw - carPose_yaw);
//     if(fwdPtPose2carPoseAngle > PI)
//     {
//     	fwdPtPose2carPoseAngle = 2 * PI - fwdPtPose2carPoseAngle;
//     }
//     return fwdPtPose2carPoseAngle;   //[0,PI]
// }

double L1Controller::getfwdPtPose2startPoseAngle(const geometry_msgs::Pose& startPose)
{
    double fwdPt_pose_yaw = getYawFromPose(fwdPt_pose);//fwdPt_pose前瞻点位姿
    double startPose_yaw = getYawFromPose(startPose);
    double fwdPtPose2startPoseAngle = fabs(fwdPt_pose_yaw - startPose_yaw);
    if(fwdPtPose2startPoseAngle > PI)
    {
    	fwdPtPose2startPoseAngle = 2 * PI - fwdPtPose2startPoseAngle;
    }
    return fwdPtPose2startPoseAngle;   //[0,PI]
}
double L1Controller::getSetSpeed(const geometry_msgs::Pose& carPose)
{
    double setSpeed;
    double fwdPt_pose_yaw = fabs(getYawFromPose(fwdPt_pose));
    double x = fwdPt_pose_map.position.x;
    if(U_count)
    {
        fwdPt_pose_yaw = PI - fwdPt_pose_yaw;
        angle_3m = PI - angle_3m;
    }
    if((fwdPt_pose_yaw >= PI * 5 / 9 && fwdPt_pose_yaw < PI) || angle_3m >= PI * 5 / 9 && angle_3m < PI)
    {
        isStraight = false;
        isU = true;
        isTurn = false;
        straight_count = 0;
        turn_count = 0;
    }
    else if(fwdPt_pose_yaw < PI * 1 / 18 && !isU)
    {
        if(++straight_count > 4)
        {
            isStraight = true;
            isU = false;
            isTurn_90 = isTurn = isTurn_120 = false;
            turn_count = 0;
        }
            
    }
    else if(fwdPt_pose_yaw >= PI * 1 / 18 && fwdPt_pose_yaw < PI * 5 / 9 && !isU)
    {
        // if(++turn_count > 4)
        // {
            isStraight = false;
            isTurn = true;
            isU = false;
            isTurn_120 = isTurn_90 = false;
            straight_count = 0;
        // }
    }
    if(U_count == 0)
    {
        if(x > 7 && x < 11) // 8~11
        {
            isTurn_90 = true;
            isStraight = isTurn = isU = isTurn_120 = false;
        }
    }
    else
    {
        if(x > 6.5 && x < 10) // 8~11
        {
            isTurn_90 = true;
            isStraight = isTurn = isU = isTurn_120 = false;
        }
    }
    if(x > 14.5 && x < 22 && !isU) // 15~22
     //if(x > 8.7 && x < 16 && !isU)
    {
    	isTurn_120 = true;
    	isStraight = isTurn = isU = isTurn_90 = false;
    }

    if(isU)
    {
        if(fabs(eta) < PI / 12 && fwdPt_pose_yaw > PI * 7 / 9 && fwdPt_pose_yaw < PI )
        {
            isU = false;
            U_count++;
        }
        setSpeed = speed_U;
    }
    if(isTurn_90)
    {
    	if(U_count == 0)
    	{
    		if(x < 9)
	    	{
	    	 	setSpeed = speed_90;
	    	}
	    	else
	    	{
	    	 	setSpeed = speed_90;	
	    	}
    	}
    	else
    	{
    		if(x > 9)
	    	{
	    	 	setSpeed = speed_90;
	    	}
	    	else
	    	{
	    	 	setSpeed = speed_90;	
	    	}
    	}

    }
    if(isTurn_120)
    {
        setSpeed = speed_120;
    }
    if(isTurn)
    {
        setSpeed = speed_turn;
    	 
    }
    if(isStraight)
    {
        if(fabs(eta) > PI / 12)
            setSpeed = 2.5;
        else
            setSpeed = speed_straight;
    }
    Lfw = getL1Distance(setSpeed);
    ROS_INFO("fwdPt_pose_yaw = %.2f\tLfw = %.2f", fwdPt_pose_yaw * 180 / PI, Lfw);     
    return setSpeed;
}
double L1Controller::getTwoPoseAngle(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2)
{
    double pose1_yaw = getYawFromPose(pose1);//fwdPt_pose前瞻点位姿
    double pose2_yaw = getYawFromPose(pose2);
    double twoPoseAngle = fabs(pose1_yaw - pose2_yaw);
    if(twoPoseAngle > PI)
    {
    	twoPoseAngle = 2 * PI - twoPoseAngle;    
    }
    return twoPoseAngle;    //[0,PI]
}
double L1Controller::getCurvature(double d_angle,double ds)
{
    double _curvature;
    _curvature = d_angle / ds;
    return _curvature;
}
double L1Controller::getAngleGain()
{
	double x = fwdPt_pose_map.position.x;
    double AngleGain = 0;
    if(isU)
    {
        AngleGain = -11.5;
        return AngleGain;
    } 
    if(isStraight)
    {
	    if(fabs(eta) > PI / 9)
	        AngleGain = -10.5;
	    else
	        AngleGain = -9.5;
    } 
    if(isTurn_90)   
    {
	    if(U_count == 0)
    	{
    		if(x < 9)
	    	{
	    		AngleGain = -12;
	    	}
	    	else
	    	{
	    		AngleGain = -12.5;
	    	}
    	}
    	else
    	{
    		if(x > 9)
	    	{
	    		AngleGain = -12;
	    	}
	    	else
	    	{
	    		AngleGain = -12.5;
	    	}
    	}
    } 
    if(isTurn_120)
    {
	    if(fabs(eta) > PI / 9)
            AngleGain = -11.5;
        else
            AngleGain = -11.5;
    }
    if(isTurn)
    {
		if(fabs(eta) > PI / 9)
	        AngleGain = -11.5;
	    else
	        AngleGain = -11.5;
    }

    return AngleGain;
}
void L1Controller::controlLoopCB(const ros::TimerEvent&)
{
    geometry_msgs::Pose carPose = odom.pose.pose;
    geometry_msgs::Twist carVel = odom.twist.twist;//单位m/s 

    if(start_flag==0)
    {
    	cmd_vel.linear.x = 1530;
    	cmd_vel.angular.z = baseAngle; 
        pub_.publish(cmd_vel);
    }
    else
    {
        // cmd_vel.linear.x = 1500 + adrc.Calculate(carVel.linear.x,3);//PIDCal(&pid_speed, u, cmd_vel.linear.x);
        // std::cout<<"u = "<<cmd_vel.linear.x-1500<<std::endl;
        // cmd_vel.linear.x = cmd_vel.linear.x > maxDutySpeed ? maxDutySpeed : cmd_vel.linear.x;
        if(goal_received)
        {
            /*Estimate Steering Angle*/
            eta = getEta(carPose);  
            double setSpeed = getSetSpeed(carPose);
            double Angle_gain = getAngleGain(); 
            cmd_vel.linear.z = setSpeed;
            if(isTurn_90)
                ROS_INFO("isTurn 90!!!!!!!!!!");
            if(isTurn_120)
            	ROS_INFO("isTurn 120!!!!!!!!!!");
            if(isTurn)
            	ROS_INFO("isTurn!!!!!!!!!!");
            if(isStraight)
                ROS_INFO("isStraight !!!!!!!!!!");
            if(isU)
                ROS_INFO("isU !!!!!!!!!!");
            cout<< " angle_3m = "<< angle_3m <<endl;
            ROS_INFO("fwdPtPose2startPoseAngle = %.2f", fwdPtPose2startPoseAngle * 180 / PI);
            // cout << "x = "<<obs_point.x << "y = "<<obs_point.y<<endl;

            if(foundForwardPt)
            {
                    lastAngle = SteeringAngle;
                    SteeringAngle = getSteeringAngle(eta);
	              	cmd_vel.angular.z = baseAngle + SteeringAngle*Angle_gain + coe * (lastAngle - SteeringAngle);

	             if(!flag && isU)
             	{
             		if(fwdPt_pose_map.position.y > -0.7)
             			num = 0;
             		else if(eta < 0)
                    {
             			num = 0;
                    }
                    else
                        num = 0;
             		flag = true;
             	}
                if(loop <= num && isU)
                {
                	// double car_yaw = getYawFromPose(carPose);
                	// if(car_yaw <= 0)
                 //    {
                 		loop++;
                        if(num == 0)
                            cmd_vel.angular.z = baseAngle + 40;
                        else if(eta)
                            cmd_vel.angular.z = baseAngle + 70;

                    // }
                }
                /*Estimate Gas Input*/
                cmd_vel.angular.z = cmd_vel.angular.z > 180 ? 180 : cmd_vel.angular.z;
                cmd_vel.angular.z = cmd_vel.angular.z < 0 ? 0 : cmd_vel.angular.z;
                if(!goal_reached)
                {
                	if(carVel.linear.x > 0)
                	{
                		speed_count ++;
                    	speedSum += carVel.linear.x;
                	}
                    double u = getGasInput(setSpeed, carVel.linear.x);
                    cout<<"u = "<<u<<endl;                   
                    cmd_vel.linear.x = 1530 + PIDCal(&pid_speed, u, cmd_vel.linear.x);//adrc.Calculate(carVel.linear.x,setSpeed);
                    
                    /*---------------openLoop------------*/
                    if(u < -0.5)
                    {
                        // if(isU)
                        //     cmd_vel.linear.x = 1370;
                        // else
                            cmd_vel.linear.x = 1300;
                    }
                    // if(u > 1.0)
                    // {
                    //     cmd_vel.linear.x = maxDutySpeed;
                    // }
                    cmd_vel.linear.x = cmd_vel.linear.x > maxDutySpeed ? maxDutySpeed : cmd_vel.linear.x;
                    cmd_vel.linear.x = cmd_vel.linear.x < minDutySpeed ? minDutySpeed : cmd_vel.linear.x;                          
                    ROS_INFO("Gas = %.2f\tSteering angle = %.2f\tcurrentSpeed = %.2f\t\teta = %.2f\tsetSpeed = %.2f\tcurvature = %.2f\tAngle_gain = %.2f",
                                    cmd_vel.linear.x,cmd_vel.angular.z,carVel.linear.x,eta*180/PI, setSpeed, AvgCurvature, Angle_gain);
                    // } 
                }
            }
        }
        if(car_stop > 0)//停车
        {
            start_loop_flag = 0;
            double avgSpeed = speedSum / speed_count;
            ROS_INFO("avgSpeed = %.2f", avgSpeed);//平均速度
            if(carVel.linear.x > 0.5)
            {
                cmd_vel.linear.x = 1200; //反向刹车
                pub_.publish(cmd_vel);
            }
            else
            {
                car_stop = 0;
                cmd_vel.linear.x = 1300;
                pub_.publish(cmd_vel);
            }
        }
        else
        {
            pub_.publish(cmd_vel);
            car_stop = 0;
        }
    }
   
}


/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "art_car_controller");
    L1Controller controller;
    ros::spin();
    return 0;
}
