#ifndef PD_CONTROLLER_H
#define PD_CONTROLLER_H

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

class PDController
{
    public:
        PDController();
        void initMarker();
        double getYawFromPose(const geometry_msgs::Pose& carPose);
        bool isForwardPath(const geometry_msgs::Point& car_pos);
        double getCar2GoalDist();
        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void speedCB(const geometry_msgs::Vector3::ConstPtr& speedMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void goalReachingCB(const ros::TimerEvent&);
        void controlLoopCB(const ros::TimerEvent&);
        double getSetSpeed();
        double getSteerAngle();
        double getCTE();
        bool isFindRefPoint();

    private:
        ros::NodeHandle n_;
        ros::Subscriber odom_sub, path_sub, goal_sub,speed_sub, obstacle_sub;
        ros::Publisher pub_, marker_pub;
        ros::Timer timer1, timer2;
        tf::TransformListener tf_listener;
        visualization_msgs::Marker points, line_strip, goal_circle;

        nav_msgs::Odometry odom;
        geometry_msgs::PoseStamped map_path_pose, odom_path_pose;
        geometry_msgs::Pose odom_pose;
        geometry_msgs::Pose car_pose;//小车当前位置
        geometry_msgs::Point odom_goal_pos;//目标点位置
        int controller_freq;
        double turn_P, turn_I, turn_D;
        double speed_P, speed_I, speed_D;
        double speedSum;
        int speed_count, start_flag, car_stop;
        nav_msgs::Path path;
        geometry_msgs::Twist cmd_vel;
        double maxDutySpeed, minDutySpeed, goalRadius;
        bool goal_received, goal_reached, find_refPoint;
        geometry_msgs::Point refPoint;
};




#endif