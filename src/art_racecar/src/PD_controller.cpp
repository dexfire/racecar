#include "PD_controller.h"

using namespace std;

PID pid_turn;
PID pid_speed;

PDController::PDController()
{
    //Private parameters handler
    ros::NodeHandle pn("~");
    //加载参数
    pn.param("controller_freq", controller_freq, 100);
    pn.param("maxDutySpeed", maxDutySpeed, 1660.0);
    pn.param("minDutySpeed", minDutySpeed, 1200.0);
    pn.param("turn_P", turn_P, 1.0);
    pn.param("turn_I", turn_I, 1.0);
    pn.param("turn_D", turn_D, 1.0);
    pn.param("speed_P", speed_P, 1.0);
    pn.param("speed_I", speed_I, 1.0);
    pn.param("speed_D", speed_D, 1.0);

    odom_sub = n_.subscribe("/odometry/filtered", 1, &PDController::odomCB, this);
    path_sub = n_.subscribe("/move_base/GlobalPlanner/plan", 1, &PDController::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &PDController::goalCB, this);
    speed_sub = n_.subscribe("/car/speed", 1, &PDController::speedCB, this);
    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 10);
    pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);

    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &PDController::controlLoopCB, this);  //100hz
    timer2 = n_.createTimer(ros::Duration((0.5)/controller_freq), &PDController::goalReachingCB, this); //200hz
    //变量初始化
    goalRadius = 1.5;
    goal_received = false;
    goal_reached = false;
    start_flag=0;//电调是否打开的标志
    cmd_vel.linear.x = 0;//1500; // 1500 for stop
    cmd_vel.angular.z = 0;//90;
    car_stop = 0;
    speedSum = 0;
    speed_count = 0;
        
    initMarker();
    PIDInit(&pid_turn);
    pid_turn.mode = 0;
    pid_turn.Proportion = turn_P;
    pid_turn.Int = turn_I;
    pid_turn.Derivative = turn_D;
    PIDInit(&pid_speed);
    pid_speed.mode = 1;
    pid_speed.Proportion = speed_P;
    pid_speed.Int = speed_I;
    pid_speed.Derivative = speed_D;
}

void PDController::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goalRadius*2;
    goal_circle.scale.y = goalRadius*2;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.r = 1.0;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}
void PDController::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    odom = *odomMsg;
}

void PDController::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
	path = *pathMsg;
}

void PDController::speedCB(const geometry_msgs::Vector3::ConstPtr& speedMsg)
{
    start_flag = 1;//(*speedMsg).z;
}

void PDController::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
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
        cout << "goal recieved"<<endl;
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void PDController::goalReachingCB(const ros::TimerEvent&)
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

double PDController::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = odom.pose.pose.position;//车子当前位置
    double car2goal_x = odom_goal_pos.x - car_pose.x;//目标位置－当前位置＝离目标的距离
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);

    return dist2goal;
}

double PDController::getSetSpeed()
{
    return 2.0;
}

double PDController::getYawFromPose(const geometry_msgs::Pose& carPose)
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

bool PDController::isFindRefPoint()
{
    double car_yaw = getYawFromPose(car_pose);
    refPoint.x = cos(car_yaw)*(odom_pose.position.x - car_pose.position.x) + sin(car_yaw)*(odom_pose.position.y - car_pose.position.y);
    refPoint.y = -sin(car_yaw)*(odom_pose.position.x - car_pose.position.x) + cos(car_yaw)*(odom_pose.position.y - car_pose.position.y);
    if(refPoint.x > 0)
        return true;
    else
        return false;
        

}
double PDController::getCTE()
{
    double CTE = 0;
    geometry_msgs::Point ref_point, car_point;
    if(!goal_reached)
    {
        for(int i = 0; i < path.poses.size(); i++)
        {
            try
            {
                map_path_pose = path.poses[i];
                tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                odom_pose = odom_path_pose.pose;
                if(i <= 10)
                	continue;
                find_refPoint = isFindRefPoint();
                if(find_refPoint)
                {
                    CTE = refPoint.y;
                    ref_point = odom_pose.position;
                    car_point = car_pose.position;
                    cout<<"============"<<i<<endl;
                    break;
                }
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }

    points.points.clear();
    line_strip.points.clear();

    //可视化
    points.points.push_back(ref_point);
    points.points.push_back(car_point);
    line_strip.points.push_back(ref_point);
    line_strip.points.push_back(car_point);

    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    }
   
    return CTE;
}

void PDController::controlLoopCB(const ros::TimerEvent&)
{
    car_pose = odom.pose.pose;
    geometry_msgs::Twist carVel = odom.twist.twist;//单位m/s 

    if(start_flag == 0)
    {
    	cmd_vel.linear.x = 1530;
    	cmd_vel.angular.z = 90; 
        pub_.publish(cmd_vel);
    }
    else
    {
        if(goal_received)
        {
            if(carVel.linear.x > 0)
            {
                speed_count ++;
                speedSum += carVel.linear.x;
            }
            double setSpeed = getSetSpeed();
            double this_cte = getCTE();
            //方向控制
            cmd_vel.angular.z = /*PIDCal(&pid_turn, this_cte, cmd_vel.angular.z);*/90 + PIDCal(&pid_turn, this_cte, cmd_vel.angular.z);
            //限幅
            cmd_vel.angular.z = cmd_vel.angular.z > 180 ? 180 : cmd_vel.angular.z;
            cmd_vel.angular.z = cmd_vel.angular.z < 0 ? 0 : cmd_vel.angular.z;
            //速度控制
            double u = setSpeed - carVel.linear.x;
            cmd_vel.linear.x = 1530 + PIDCal(&pid_speed, u, cmd_vel.linear.x);
            //限幅
            cmd_vel.linear.x = cmd_vel.linear.x > maxDutySpeed ? maxDutySpeed : cmd_vel.linear.x;
            cmd_vel.linear.x = cmd_vel.linear.x < minDutySpeed ? minDutySpeed : cmd_vel.linear.x;

            cout << "steerAngle = " << cmd_vel.angular.z << " setSpeed = " << setSpeed << " currentSpeed = "
                 << carVel.linear.x << endl;
            cout << "this_cte = "<<this_cte<<endl;

        }

        if(car_stop > 0)//停车
        {
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

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "PD_controller");
    PDController controller;
    ros::spin();
    return 0;
}
