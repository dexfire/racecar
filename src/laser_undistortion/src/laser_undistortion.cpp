#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>

#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl-1.7/pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <dirent.h>
#include <fstream>
#include <iostream>

pcl::visualization::CloudViewer g_PointCloudView("PointCloud View");

class LidarMotionCalibrator
{
public:

    LidarMotionCalibrator(tf::TransformListener* tf)
    {
        tf_ = tf;
        scan_sub_ = nh_.subscribe("/original_scan", 10, &LidarMotionCalibrator::ScanCallBack, this);
        scan_pub = nh_.advertise<sensor_msgs::LaserScan>("/scan", 1000);
    }


    ~LidarMotionCalibrator()
    {
        if(tf_!=NULL)
            delete tf_;
    }

    // æ¿å°åå§çæ¿åæ°æ®æ¥è¿è¡å¤ç
    void ScanCallBack(const sensor_msgs::LaserScanPtr& scan_msg)
    {
        //è½¬æ¢å°ç«æ­£éè¦çæ°æ®
        sensor_msgs::LaserScan laserScanMsg = *scan_msg;
        ros::Time startTime, endTime;
        startTime = scan_msg->header.stamp;
        int num_angle;
        //å¾å°æç»ç¹çæ¶é´
        int beamNum = laserScanMsg.ranges.size();
        endTime = startTime + ros::Duration(laserScanMsg.time_increment * beamNum);

        // å°æ°æ®å¤å¶åºæ¥
        std::vector<double> angles,ranges;
        for(int i = beamNum - 1; i > 0;i--)
        {   
            double lidar_dist = laserScanMsg.ranges[i];
            double lidar_angle = (359-i)/180*3.1415926;//laserScanMsg.angles[i];

            if(lidar_dist < 0.05 || std::isnan(lidar_dist) || std::isinf(lidar_dist))
                lidar_dist = 0.0;

            ranges.push_back(lidar_dist);
            angles.push_back(lidar_angle);
        }

        //è½¬æ¢ä¸ºpcl::pointcloud for visuailization

        tf::Stamped<tf::Pose> visualPose;
        if(!getLaserPose(visualPose, startTime, tf_))
        {

            ROS_WARN("Not visualPose,Can not Calib");
            return ;
        }

        double visualYaw = tf::getYaw(visualPose.getRotation());

        visual_cloud_.clear();
        for(int i = 0; i < ranges.size();i++)
        {

            if(ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            double x = ranges[i] * cos(angles[i]);
            double y = ranges[i] * sin(angles[i]);

            pcl::PointXYZRGB pt;
            pt.x = x * cos(visualYaw) - y * sin(visualYaw) + visualPose.getOrigin().getX();
            pt.y = x * sin(visualYaw) + y * cos(visualYaw) + visualPose.getOrigin().getY();
            pt.z = 1.0;

            // pack r/g/b into rgb
            unsigned char r = 255, g = 0, b = 0;    //red color
            unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
            pt.rgb = *reinterpret_cast<float*>(&rgb);

            visual_cloud_.push_back(pt);
        }
        std::cout << std::endl;



        //è¿è¡ç«æ­£
        Lidar_Calibration(ranges,angles,
                          startTime,
                          endTime,
                          tf_);

        //è½¬æ¢ä¸ºpcl::pointcloud for visuailization
        for(int i = 0; i < ranges.size();i++)
        {

            if(ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            double x = ranges[i] * cos(angles[i]);
            double y = ranges[i] * sin(angles[i]);


            pcl::PointXYZRGB pt;
            pt.x = x * cos(visualYaw) - y * sin(visualYaw) + visualPose.getOrigin().getX();
            pt.y = x * sin(visualYaw) + y * cos(visualYaw) + visualPose.getOrigin().getY();
            pt.z = 1.0;

            unsigned char r = 0, g = 255, b = 0;    // green color
            unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
            pt.rgb = *reinterpret_cast<float*>(&rgb);

            visual_cloud_.push_back(pt);
        }

        //è¿è¡æ¾ç¤º
         g_PointCloudView.showCloud(visual_cloud_.makeShared());
         num_angle = int(angle_towards/3.1415);
         cout<< "num_angle" << num_angle << endl;
            for(int i = beamNum - 1; i > 0;i--)
          {   
            laserScanMsg.ranges[beamNum-i-1] = ranges[i];
          }
          for(int i = beamNum - 1; i > 0;i--)
          {   
            int k=beamNum-i-1+num_angle;
            if (k>=360){k=k-360;}
            laserScanMsg.ranges[beamNum-i-1] = laserScanMsg.ranges[beamNum-i-1+num_angle];
          }
         scan_pub.publish(laserScanMsg);
    }


    /**
     * @name getLaserPose()
     * @brief å¾å°æºå¨äººå¨éç¨è®¡åæ ç³»ä¸­çä½å§¿tf::Pose
     *        å¾å°dtæ¶å»æ¿åé·è¾¾å¨odomåæ ç³»çä½å§¿
     * @param odom_pos  æºå¨äººçä½å§¿
     * @param dt        dtæ¶å»
     * @param tf_
    */
    bool getLaserPose(tf::Stamped<tf::Pose> &odom_pose,
                      ros::Time dt,
                      tf::TransformListener * tf_)
    {
        odom_pose.setIdentity();

        tf::Stamped < tf::Pose > robot_pose;
        robot_pose.setIdentity();
        robot_pose.frame_id_ = "base_laser";
        robot_pose.stamp_ = dt;   //è®¾ç½®ä¸ºros::Time()è¡¨ç¤ºè¿åæè¿çè½¬æ¢å³ç³»

        // get the global pose of the robot
        try
        {
            if(!tf_->waitForTransform("/odom", "/base_laser", dt, ros::Duration(0.5)))             // 0.15s çæ¶é´å¯ä»¥ä¿®æ¹
            {
                ROS_ERROR("LidarMotion-Can not Wait Transform()");
                return false;
            }
            tf_->transformPose("/odom", robot_pose, odom_pose);
        }
        catch (tf::LookupException& ex)
        {
            ROS_ERROR("LidarMotion: No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_ERROR("LidarMotion: Connectivity Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_ERROR("LidarMotion: Extrapolation Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }


    /**
     * @brief Lidar_MotionCalibration
     *        æ¿åé·è¾¾è¿å¨ç¸åå»é¤åæ®µå½æ°;
     *        å¨æ­¤åæ®µå½æ°ä¸­ï¼è®¤ä¸ºæºå¨äººæ¯åéè¿å¨ï¼
     * @param frame_base_pose       æ å®å®æ¯ä¹åçåºååæ ç³»
     * @param frame_start_pose      æ¬åæ®µç¬¬ä¸ä¸ªæ¿åç¹å¯¹åºçä½å§¿
     * @param frame_end_pose        æ¬åæ®µæåä¸ä¸ªæ¿åç¹å¯¹åºçä½å§¿
     * @param ranges                æ¿åæ°æ®ï¼ï¼è·ç¦»
     * @param angles                æ¿åæ°æ®ï¼ï¼è§åº¦
     * @param startIndex            æ¬åæ®µç¬¬ä¸ä¸ªæ¿åç¹å¨æ¿åå¸§ä¸­çä¸æ 
     * @param beam_number           æ¬åæ®µçæ¿åç¹æ°é
     */
    void Lidar_MotionCalibration(
            tf::Stamped<tf::Pose> frame_base_pose,
            tf::Stamped<tf::Pose> frame_start_pose,
            tf::Stamped<tf::Pose> frame_end_pose,
            std::vector<double>& ranges,
            std::vector<double>& angles,
            int startIndex,
            int& beam_number)
    {
       //TODO
       //æ¯ä¸ªä½å§¿è¿è¡çº¿æ§æå¼æ¶çæ­¥é¿
        double beam_step = 1.0 / (beam_number-1);

        //æºå¨äººçèµ·å§è§åº¦ å æç»è§åº¦
        tf::Quaternion start_angle_q =   frame_start_pose.getRotation();
        tf::Quaternion   end_angle_q =   frame_end_pose.getRotation();

        //è½¬æ¢å°å¼§åº¦
        double start_angle_r = tf::getYaw(start_angle_q);
        double base_angle_r = tf::getYaw(frame_base_pose.getRotation());

        //æºå¨äººçèµ·å§ä½å§¿
        tf::Vector3 start_pos = frame_start_pose.getOrigin();
        start_pos.setZ(0);

        //æç»ä½å§¿
        tf::Vector3 end_pos = frame_end_pose.getOrigin();
        end_pos.setZ(0);

        //åºç¡åæ ç³»
        tf::Vector3 base_pos = frame_base_pose.getOrigin();
        base_pos.setZ(0);

        double mid_angle;
        tf::Vector3 mid_pos;
        tf::Vector3 mid_point;

        double lidar_angle, lidar_dist;
        //æå¼è®¡ç®åºæ¥æ¯ä¸ªç¹å¯¹åºçä½å§¿
        for(int i = 0; i< beam_number;i++)
        {
            //è§åº¦æå¼
            mid_angle =  tf::getYaw(start_angle_q.slerp(end_angle_q, beam_step * i));

            //çº¿æ§æå¼
            mid_pos = start_pos.lerp(end_pos, beam_step * i);

            //å¾å°æ¿åç¹å¨odom åæ ç³»ä¸­çåæ  æ ¹æ®
            double tmp_angle;

            //å¦ææ¿åé·è¾¾ä¸ç­äºæ ç©·,åéè¦è¿è¡ç«æ­£.
            if( tfFuzzyZero(ranges[startIndex + i]) == false)
            {
                //è®¡ç®å¯¹åºçæ¿åç¹å¨odomåæ ç³»ä¸­çåæ 

                //å¾å°è¿å¸§æ¿åæè·ç¦»åå¤¹è§
                lidar_dist  =  ranges[startIndex+i];
                lidar_angle =  angles[startIndex+i];

                //æ¿åé·è¾¾åæ ç³»ä¸çåæ 
                double laser_x,laser_y;
                laser_x = lidar_dist * cos(lidar_angle);
                laser_y = lidar_dist * sin(lidar_angle);

                //éç¨è®¡åæ ç³»ä¸çåæ 
                double odom_x,odom_y;
                odom_x = laser_x * cos(mid_angle) - laser_y * sin(mid_angle) + mid_pos.x();
                odom_y = laser_x * sin(mid_angle) + laser_y * cos(mid_angle) + mid_pos.y();

                //è½¬æ¢å°ç±»åä¸­å»
                mid_point.setValue(odom_x, odom_y, 0);

                //æå¨odomåæ ç³»ä¸­çæ¿åæ°æ®ç¹ è½¬æ¢å° åºç¡åæ ç³»
                double x0,y0,a0,s,c;
                x0 = base_pos.x();
                y0 = base_pos.y();
                a0 = base_angle_r;
                s = sin(a0);
                c = cos(a0);
                /*
                 * æbaseè½¬æ¢å°odom ä¸º[c -s x0;
                 *                   s c y0;
                 *                   0 0 1]
                 * æodomè½¬æ¢å°baseä¸º [c s -x0*c-y0*s;
                 *               -s c x0*s - y0*c;
                 *                0 0 1]ä»£æ°ä½å­å¼åé
                 */
                double tmp_x,tmp_y;
                tmp_x =  mid_point.x()*c  + mid_point.y()*s - x0*c - y0*s;
                tmp_y = -mid_point.x()*s  + mid_point.y()*c  + x0*s - y0*c;
                mid_point.setValue(tmp_x,tmp_y,0);

                //ç¶åè®¡ç®ä»¥èµ·å§åæ ä¸ºèµ·ç¹ç dist angle
                double dx,dy;
                dx = (mid_point.x());
                dy = (mid_point.y());
                lidar_dist = sqrt(dx*dx + dy*dy);
                lidar_angle = atan2(dy,dx);

                //æ¿åé·è¾¾è¢«ç«æ­£
                ranges[startIndex+i] = lidar_dist;
                angles[startIndex+i] = lidar_angle;
            }
            //å¦æç­äºæ ç©·,åéä¾¿è®¡ç®ä¸ä¸è§åº¦
            else
            {
                //æ¿åè§åº¦
                lidar_angle = angles[startIndex+i];

                //éç¨è®¡åæ ç³»çè§åº¦
                tmp_angle = mid_angle + lidar_angle;
                tmp_angle = tfNormalizeAngle(tmp_angle);

                //å¦ææ°æ®éæ³ ååªéè¦è®¾ç½®è§åº¦å°±å¯ä»¥äºãæè§åº¦æ¢ç®æstart_posåæ ç³»åçè§åº¦
                lidar_angle = tfNormalizeAngle(tmp_angle - start_angle_r);

                angles[startIndex+i] = lidar_angle;
            }
       //end of TODO
        }
    }
    //æ¿åé·è¾¾æ°æ®ãåæ®µçº¿æ§è¿è¡æå¼ãåæ®µçå¨æä¸º10ms
    //è¿éä¼è°ç¨Lidar_MotionCalibration()
    /**
     * @name Lidar_Calibration()
     * @brief æ¿åé·è¾¾æ°æ®ãåæ®µçº¿æ§è¿è¡å·®å¼ãåæ®µçå¨æä¸º5ms
     * @param ranges æ¿åæçè·ç¦»å¼éå
     * @param angleãæ¿åæçè§åº¦å¼éå
     * @param startTimeãç¬¬ä¸ææ¿åçæ¶é´æ³
     * @param endTimeãæåä¸ææ¿åçæ¶é´æ³
     * @param *tf_
    */
    void Lidar_Calibration(std::vector<double>& ranges,
                           std::vector<double>& angles,
                           ros::Time startTime,
                           ros::Time endTime,
                           tf::TransformListener * tf_)
    {
        //ç»è®¡æ¿åæçæ°é
        int beamNumber = ranges.size();
        if(beamNumber != angles.size())
        {
            ROS_ERROR("Error:ranges not match to the angles");
            return ;
        }

        // 5msæ¥è¿è¡åæ®µ
        int interpolation_time_duration = 5 * 1000;

        tf::Stamped<tf::Pose> frame_start_pose;
        tf::Stamped<tf::Pose> frame_mid_pose;
        tf::Stamped<tf::Pose> frame_base_pose;
        tf::Stamped<tf::Pose> frame_end_pose;

        //èµ·å§æ¶é´ us
        double start_time = startTime.toSec() * 1000 * 1000;
        double end_time = endTime.toSec() * 1000 * 1000;
        double time_inc = (end_time - start_time) / beamNumber; // æ¯ææ¿åæ°æ®çæ¶é´é´é

        //å½åæå¼çæ®µçèµ·å§åæ 
        int start_index = 0;

        //èµ·å§ç¹çä½å§¿ è¿éè¦å¾å°èµ·å§ç¹ä½ç½®çåå æ¯ãèµ·å§ç¹å°±æ¯æä»¬çbase_pose
        //ææçæ¿åç¹çåºåä½å§¿é½ä¼æ¹ææä»¬çbase_pose
        // ROS_INFO("get start pose");

        if(!getLaserPose(frame_start_pose, ros::Time(start_time /1000000.0), tf_))
        {
            ROS_WARN("Not Start Pose,Can not Calib");
            return ;
        }

        if(!getLaserPose(frame_end_pose,ros::Time(end_time / 1000000.0),tf_))
        {
            ROS_WARN("Not End Pose, Can not Calib");
            return ;
        }
        //æºå¨äººçèµ·å§è§åº¦ å æç»è§åº¦

        tf::Quaternion start_angle_q =   frame_base_pose.getRotation();
        tf::Quaternion   end_angle_q =   frame_end_pose.getRotation();
        double start_angle_r = tf::getYaw(start_angle_q);
        double end_angle_r = tf::getYaw(end_angle_q);
        angle_towards = end_angle_r-start_angle_r;
        int cnt = 0;
        //åºååæ å°±æ¯ç¬¬ä¸ä¸ªä½å§¿çåæ 
        frame_base_pose = frame_start_pose;
        for(int i = 0; i < beamNumber; i++)
        {
            //åæ®µçº¿æ§,æ¶é´æ®µçå¤§å°ä¸ºinterpolation_time_duration
            double mid_time = start_time + time_inc * (i - start_index);
            if(mid_time - start_time > interpolation_time_duration || (i == beamNumber - 1))
            {
                cnt++;

                //å¾å°èµ·ç¹åç»ç¹çä½å§¿
                //ç»ç¹çä½å§¿
                if(!getLaserPose(frame_mid_pose, ros::Time(mid_time/1000000.0), tf_))
                {
                    ROS_ERROR("Mid %d Pose Error",cnt);
                    return ;
                }

                //å¯¹å½åçèµ·ç¹åç»ç¹è¿è¡æå¼
                //interpolation_time_durationä¸­é´æå¤å°ä¸ªç¹.
                int interp_count = i - start_index + 1;

                Lidar_MotionCalibration(frame_base_pose,
                                        frame_start_pose,
                                        frame_mid_pose,
                                        ranges,
                                        angles,
                                        start_index,
                                        interp_count);

                //æ´æ°æ¶é´
                start_time = mid_time;
                start_index = i;
                frame_start_pose = frame_mid_pose;
            }
        }
    }

public:
    tf::TransformListener* tf_;
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher scan_pub;
    pcl::PointCloud<pcl::PointXYZRGB> visual_cloud_;
    double  angle_towards;
};




int main(int argc,char ** argv)
{
    ros::init(argc,argv,"LidarMotionCalib");

    tf::TransformListener tf(ros::Duration(10.0));

    LidarMotionCalibrator tmpLidarMotionCalib(&tf);

    ros::spin();
    return 0;
}


