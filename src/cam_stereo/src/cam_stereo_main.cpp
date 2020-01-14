//程序版本：2019-3-16-V1.0
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <string>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>    
#include <opencv2/opencv.hpp> 
#include <thread>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include "stereo_main.h"

using namespace std;
using namespace cv;

//io_stereo stereo;
const int imageWidth  = 1280;                      //摄像头单目的分辨率########--【需要调整参数的位置1】--#############
const int imageHeight = 720;

Size imageSize = Size(imageWidth, imageHeight);

Mat rgbImageL, grayImageL;
Mat rgbImageR, grayImageR;
Mat rectifyImageL, rectifyImageR;

Rect validROIL;                                   //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  
Rect validROIR;

Mat mapLx, mapLy, mapRx, mapRy;                   //映射表  
Mat Rl, Rr, Pl, Pr, Q;                            //校正旋转矩阵R，投影矩阵P, 重投影矩阵Q
Mat xyz;                                          //三维坐标

Point origin;                                     //鼠标按下的起始点
Rect selection;                                   //定义矩形选框
bool selectObject = false;                        //是否选择对象
Ptr<StereoBM> bm = StereoBM::create(16, 9);


//########--【以下双目的标定参数为：需要调整参数的位置2】--#############
//相机双目标定的结果与如下各参数的对应关系见：双目标定结果说明.pdf，pdf文档位于main.cpp（即本文档）同级文件夹--#############

/*左目相机标定参数------------------------
fc_left_x   0            cc_left_x
0           fc_left_y    cc_left_y
0           0            1
-----------------------------------------*/

Mat cameraMatrixL = (Mat_<double>(3, 3) << 905.1608,        0,   633.3046,
	                                          0, 908.0385,   361.7447,
	                                          0,        0,          1);


Mat distCoeffL = (Mat_<double>(5, 1) << 0.0551,     -0.0891,     -0.0012,    -0.00087,     0.0000);
                                     //[kc_left_01,  kc_left_02,  kc_left_03,  kc_left_04,   kc_left_05]
/*右目相机标定参数------------------------
fc_right_x   0              cc_right_x
0            fc_right_y     cc_right_y
0            0              1
-----------------------------------------*/
Mat cameraMatrixR = (Mat_<double>(3, 3) << 916.4431,         0,  688.8407,
	                                          0,  919.7748,  354.4873,
	                                          0,         0,         1);

Mat distCoeffR = (Mat_<double>(5, 1) << 0.0321,      -0.0742,     0.00064,     0.0011,      0.0000);
                                     //[kc_right_01,  kc_right_02,  kc_right_03,  kc_right_04,   kc_right_05]


//Mat T = (Mat_<double>(3, 1) << -28.11909,    0.11966,    0.21590);    //T平移向量
Mat T = (Mat_<double>(3, 1) << -61.0749,    -2.5814,    3.4041);    //T平移向量
							 //[T_01,        T_02,       T_03]


Mat src = (Mat_<double>(3, 3)<< 1.0000, 0.00026,    0.0018,
	                           -0.00026,  1.0000,  -0.00014,
                                    -0.0018, 0.00014,    1.0000);
Mat rec;// = (Mat_<double>(3, 1) << -0.01688,   -0.00781,   -0.00766);   //rec旋转向量
							  //[rec_01,     rec_02,     rec_03]
//########--双目的标定参数填写完毕-----------------------------------------------------------------------

Mat R;                                                     //R矩阵，用于中间计算
Mat depthmap;


using namespace std;
using namespace cv;

void GenerateFalseMap(cv::Mat &src, cv::Mat &disp)                             //颜色变换
{ 
	float max_val = 255.0f;
	float map[8][4] = { { 0,0,0,114 },{ 0,0,1,185 },{ 1,0,0,114 },{ 1,0,1,174 },
	{ 0,1,0,114 },{ 0,1,1,185 },{ 1,1,0,114 },{ 1,1,1,0 } };
	float sum = 0;
	for (int i = 0; i<8; i++)
		sum += map[i][3];

	float weights[8];   
	float cumsum[8];  
	cumsum[0] = 0;
	for (int i = 0; i<7; i++) {
		weights[i] = sum / map[i][3];
		cumsum[i + 1] = cumsum[i] + map[i][3] / sum;
	}

	int height_ = src.rows;
	int width_ = src.cols;
 
	for (int v = 0; v<height_; v++) {
		for (int u = 0; u<width_; u++) {
 
			float val = std::min(std::max(src.data[v*width_ + u] / max_val, 0.0f), 1.0f);

			int i;
			for (i = 0; i<7; i++)
				if (val<cumsum[i + 1])
					break;
 
			float   w = 1.0 - (val - cumsum[i])*weights[i];
			uchar r = (uchar)((w*map[i][0] + (1.0 - w)*map[i + 1][0]) * 255.0);
			uchar g = (uchar)((w*map[i][1] + (1.0 - w)*map[i + 1][1]) * 255.0);
			uchar b = (uchar)((w*map[i][2] + (1.0 - w)*map[i + 1][2]) * 255.0);
			 
			disp.data[v*width_ * 3 + 3 * u + 0] = b;                               //rgb内存连续存放 
			disp.data[v*width_ * 3 + 3 * u + 1] = g;
			disp.data[v*width_ * 3 + 3 * u + 2] = r;
		}
	}
}

//--BM算法立体匹配--------------------------------------------------------------------
void stereo_match_bm(int, void*)
{
	int blockSize = 18, uniquenessRatio = 5,  numDisparities = 11; //BM算法相关的参数，【需要调整参数的位置3，仅用于BM算法有效】--############

	bm->setBlockSize(2 * blockSize + 5);                           //SAD窗口大小，5~21之间为宜
	bm->setROI1(validROIL);                                        //左右视图的有效像素区域
	bm->setROI2(validROIR);
	bm->setPreFilterCap(61);                                       //预处理滤波器值
	bm->setMinDisparity(32);                                       //最小视差，默认值为0, 可以是负值，int型
	bm->setNumDisparities(numDisparities * 16 );                   //视差窗口，即最大视差值与最小视差值之差,16的整数倍
	bm->setTextureThreshold(10);
	bm->setUniquenessRatio(uniquenessRatio);                       //视差唯一性百分比,uniquenessRatio主要可以防止误匹配
	bm->setSpeckleWindowSize(100);                                 //检查视差连通区域变化度的窗口大小
	bm->setSpeckleRange(32);                                       //32视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零
	bm->setDisp12MaxDiff(-1);
	Mat disp;
	Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);               //显示  
	bm->compute(rectifyImageL, rectifyImageR, disp);               //输入图像必须为灰度图

	reprojectImageTo3D(disp, xyz, Q, true);                        //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)
	xyz = xyz * 16;

	disp.convertTo(disp, CV_32F, 1.0 / 16);                        //除以16得到真实视差值,disp.convertTo(disp, CV_32F, 1.0 );
	normalize(disp, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);

	medianBlur(disp8U, disp8U, 9);  //中值滤波

	Mat dispcolor(disp8U.size(), CV_8UC3);
	GenerateFalseMap(disp8U, dispcolor);//上色
	depthmap = dispcolor;
	imshow("disparity", dispcolor);
}

void stereo_match_sgbm(int, void*)                                         //SGBM匹配算法
{
	int mindisparity = 32;                                                 //最小视差
	int SADWindowSize = 16;                                                //滑动窗口的大小
	int ndisparities = 176;                                                //最大的视差，要被16整除
	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(mindisparity, ndisparities, SADWindowSize);

	int P1 = 4 * rectifyImageL.channels() * SADWindowSize* SADWindowSize;  //惩罚系数1
	int P2 = 32 * rectifyImageL.channels() * SADWindowSize* SADWindowSize; //惩罚系数2
	sgbm->setP1(P1);
	sgbm->setP2(P2);

	sgbm->setPreFilterCap(60);                                             //滤波系数
	sgbm->setUniquenessRatio(30);                                          //代价方程概率因子
	sgbm->setSpeckleRange(2);                                              //相邻像素点的视差值浮动范围
	sgbm->setSpeckleWindowSize(200);                                       //针对散斑滤波的窗口大小
	sgbm->setDisp12MaxDiff(1);                                             //视差图的像素点检查
	//sgbm->setMode(cv::StereoSGBM::MODE_HH);  

	Mat disp;
	sgbm->compute(rectifyImageL, rectifyImageR, disp);

	Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);                       //显示  

	reprojectImageTo3D(disp, xyz, Q, true);                                //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)
	xyz = xyz * 16;

	disp.convertTo(disp, CV_32F, 1.0 / 16);                                //除以16得到真实视差值,disp.convertTo(disp, CV_32F, 1.0 );
	normalize(disp, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);

	medianBlur(disp8U, disp8U, 9);                                             //中值滤波

	Mat dispcolor(disp8U.size(), CV_8UC3);
	GenerateFalseMap(disp8U, dispcolor);
	depthmap = dispcolor;
	imshow("disparity", dispcolor);

}


int stereo_computer()
{
	
	cout << "src = " << endl << " " << src << endl << endl;
	Rodrigues(src,rec);
	cout << "rec = " << endl << " " << rec << endl << endl;
//--立体校正-------------------------------------------------------------------
	Rodrigues(rec, R);                                   //Rodrigues变换
	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
		0, imageSize, &validROIL, &validROIR);
	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

//--读取图片，【需要调整参数的位置4】----------------------------------------------------------------
	//rgbImageL = imread("Test-01-left.bmp", CV_LOAD_IMAGE_COLOR);
	cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);
	//rgbImageR = imread("Test-01-right.bmp", CV_LOAD_IMAGE_COLOR);
	cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);

	//namedWindow("ImageL Before Rectify", WINDOW_NORMAL);  imshow("ImageL Before Rectify", grayImageL);
	//namedWindow("ImageR Before Rectify", WINDOW_NORMAL);  imshow("ImageR Before Rectify", grayImageR);

//--经过remap之后，左右相机的图像已经共面并且行对准----------------------------------------------
	remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
	remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

//--把校正结果显示出来---------------------------------------------------------------------------
	Mat rgbRectifyImageL, rgbRectifyImageR;
	cvtColor(rectifyImageL, rgbRectifyImageL, CV_GRAY2BGR);  
	cvtColor(rectifyImageR, rgbRectifyImageR, CV_GRAY2BGR);
	//imwrite("rectifyImageL.jpg", rectifyImageL);imwrite("rectifyImageR.jpg", rectifyImageR);

	//namedWindow("ImageL After Rectify", WINDOW_NORMAL); imshow("ImageL After Rectify", rgbRectifyImageL);
	//namedWindow("ImageR After Rectify", WINDOW_NORMAL); imshow("ImageR After Rectify", rgbRectifyImageR);


//--显示在同一张图上-----------------------------------------------------------------------------
	Mat canvas;
	double sf;
	int w, h;
	sf = 600. / MAX(imageSize.width, imageSize.height);
	w = cvRound(imageSize.width * sf);
	h = cvRound(imageSize.height * sf);
	canvas.create(h, w * 2, CV_8UC3);                                             //注意通道

//--左图像画到画布上-----------------------------------------------------------------------------
	Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //得到画布的一部分  
	resize(rgbRectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);    //把图像缩放到跟canvasPart一样大小  
	Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),                  //获得被截取的区域    
		cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
	//rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                     
	cout << "Painted ImageL" << endl;

//--右图像画到画布上-----------------------------------------------------------------------------
	canvasPart = canvas(Rect(w, 0, w, h));                                        //获得画布的另一部分  
	resize(rgbRectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
		cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
	//rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
	cout << "Painted ImageR" << endl;

//--画上对应的线条-------------------------------------------------------------------------------
	for (int i = 0; i < canvas.rows; i += 16)
		line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
	//namedWindow("rectified", WINDOW_NORMAL);  imshow("rectified", canvas);

	//stereo_match_sgbm(0, 0);   //--【需要调整参数的位置5】，本行调用sgbm算法，下一行调用BM算法，二选一进行距离测量。
	stereo_match_bm(0, 0);

	//waitKey(0);
	return 0;
}              
int main(int argc, char** argv)            //程序主函数
{
	
     	//Initiate ROS
     	ros::init(argc, argv, "cam_stereo");
		ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Publisher pub = it.advertise("image", 1);

	tf::TransformBroadcaster broadcaster;

	VideoCapture cap;
        double fScale = 0.5 ;
	double rate = 30.0;                          //视频的帧率  
	int videosource = 0;
	Mat frame,frame_L,frame_R;
	cap.open(videosource);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 2560);  //设置捕获视频的宽度
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);  //设置捕获视频的高度

        cap.set(CV_CAP_PROP_FPS, 30);
	cap >> frame;                                //从相机捕获一帧图像
	cout<<"Setted Size:"<<frame.cols<<"*"<<frame.rows<<"\n";
	cout<<"Camera rate:"<<cap.get(CV_CAP_PROP_FPS)<<"\n";
	
	Size dsize = Size(frame.cols*fScale, frame.rows*fScale);
	Mat imagedst = Mat(dsize, CV_32S);
	resize(frame, imagedst, dsize);

	
	Size videoSize(2560, 720);
        Size videoSize_L(1280, 720);
        Size videoSize_R(1280, 720);            


        unsigned int pair_id = 0;
        ros::Time capture_time = ros::Time::now();
	ros::Time ends = ros::Time::now();
	while (ros::ok())
	{
                //ros::spinOnce();
		cap >> frame;                            //从相机捕获一帧图像
		resize(frame,frame,videoSize);                       //将抓拍的图像写入AVI视频文件
		namedWindow("Video", 2); 
		imshow("Video", frame);
                rgbImageL = frame(Rect(0, 0, 1280, 720));
                rgbImageR = frame(Rect(1280, 0, 1280, 720));
                stereo_computer();
                sensor_msgs::ImagePtr depth_image =  cv_bridge::CvImage(std_msgs::Header(), "bgr8", depthmap).toImageMsg();
                depth_image->height = imageHeight;
                depth_image->width  = imageWidth;
                depth_image->header.stamp = capture_time;
                depth_image->header.frame_id = "camera";
                depth_image->header.seq = pair_id;  
                pub.publish(depth_image);
                ++pair_id;
		broadcaster.sendTransform(
                              tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), 						tf::Vector3(0.4, 0.0, 0.0)), 
                                                   ros::Time::now(), 
                                                   "base_link", 
                                                   "camera"));
		if (waitKey(30) >= 0) break;
		
	}
	cap.release();                               //释放对相机的控制
    
    	ros::spin();
	return 0;
}
