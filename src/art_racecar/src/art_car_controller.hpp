/********************/
/* CLASS DEFINITION */
/********************/
#include "ADRC.h"
#include <vector>
using namespace std;
class L1Controller
{
    public:
        L1Controller();
        void initMarker();
        //bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
        bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);
        double getYawFromPose(const geometry_msgs::Pose& carPose);        
        double getEta(const geometry_msgs::Pose& carPose);
        double getCar2GoalDist();
        double getL1Distance(const double& _Vcmd);
        double getSteeringAngle(double eta);
        double getGasInput(const double& setSpeed, const float& current_v);
        geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);
        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void speedCB(const geometry_msgs::Vector3::ConstPtr& speedMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void obstacleCB(const geometry_msgs::Point::ConstPtr& obstacleMsg);
        void goalReachingCB(const ros::TimerEvent&);
        void controlLoopCB(const ros::TimerEvent&);
        double getSetSpeed(const geometry_msgs::Pose& carPose);
        // double getfwdPtPose2carPoseAngle(const geometry_msgs::Pose& carPose);
        bool isForwardPath(const geometry_msgs::Point& car_pos);
        double getCurvature(double dx,double ds);
        double getTwoPoseAngle(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);//获取两位姿间夹角
        double getAngleGain();
        double getfwdPtPose2startPoseAngle(const geometry_msgs::Pose& startPose);
    private:
        ros::NodeHandle n_;
        ros::Subscriber odom_sub, path_sub, goal_sub,speed_sub, obstacle_sub;
        ros::Publisher pub_, marker_pub;
        ros::Timer timer1, timer2;
        tf::TransformListener tf_listener;

        visualization_msgs::Marker points, line_strip, goal_circle, start_point;
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::Point odom_goal_pos;
        nav_msgs::Odometry odom;
        nav_msgs::Path map_path, odom_path, lock_path, path;

        double L, Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v, eta, last_eta, turn_coe;
        double Gas_gain, baseAngle, Angle_gain, goalRadius,setTurnAngle;
        int controller_freq, baseSpeed,turnSpeed;
        bool foundForwardPt, goal_received, goal_reached;
        int car_stop,start_flag;
        geometry_msgs::Pose fwdPt_pose, fwdPt_pose_map;//前瞻点的位姿
        geometry_msgs::Point obs_point; //障碍物坐标
        double speedSum;
        double maxDutySpeed, minDutySpeed;
        ADRC adrc;
        double AvgCurvature;
        bool isU, isStraight, isTurn_90, isTurn_120, isTurn;
        bool turn_left, turn_right;
        bool pathLock;
        int path_temp, loop_count;
        double fwdPtPose2startPoseAngle;
        double angle_3m;
        double SteeringAngle, lastAngle, coe;
        double speed_U, speed_90 ,speed_120 ,speed_turn, speed_straight;
}; // end of class
