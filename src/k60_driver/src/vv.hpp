/* 负责与单片机通讯：发送控制指令、发布编码器获取的速度
 * 异常恢复
 */

class Driver
{
    public:
        Driver();
        void send_cmd(uint16_t motor_pwm,uint16_t servo_pwm);
        void read_speed();
        void relocate();

    private:
        ros::NodeHandle n_;
        ros::Publisher speed_pub,pose_pub;

		ros::Subscriber cmd_sub,laser_sub,goal_sub,odom_sub;
        ros::Timer timer1,timer2;

        serial::Serial sp;
        
        geometry_msgs::PoseStamped goal;
        nav_msgs::Odometry odom,last_odom,cur_odom;
        geometry_msgs::PoseWithCovarianceStamped initialpose;
        sensor_msgs::LaserScan laser;
        geometry_msgs::Vector3 speed;


		float lengths[100],distance,max_dis;
		int speeds[100],area[12][2],max_dir;
		int status,odom_err,speed_err; //0->ok 1->error
        uint8_t data[7];

        void cmdCB(const geometry_msgs::Twist& twist);
        void laserCB(const sensor_msgs::LaserScan::ConstPtr& laserMsg);
        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);

        void checkLoop(const ros::TimerEvent&);
        void recoveryLoop(const ros::TimerEvent&);

};