#include "kinect_init.h"
#ifndef ROBOT_LOCATOR_H_
#define ROBOT_LOCATOR_H_

#define SHOOT_RGB_Camera 1
#define MAX_BLUE_H 119
#define MIN_BLUE_H 98
#define MAX_RED_H 180
#define MIN_RED_H 160
#define MAX_L_RED_H 0
#define MIN_L_RED_H 0
#define MAX_S 255
#define MIN_S 130
#define MAX_V 255
#define MIN_V 90

#define TR
// #define DR
// #define DEBUG
// #define IMAGE
// #define TIME
// #define IMU
// #define SHOWPCL
// #define COUTFLAG


#define RED 	1
#define BLUE  	2

#define NORTH	1
#define SOUTH 	2
#define WEST	3
#define EAST	4
#define MID		5

struct PicRoi
{
	int leftEdge = 0;
	int rightEdge = 0;
	int topEdge = 0;
	int bottomEdge = 0;
};

class RobotLocator
{
public:
	KinectInit*         thiskinectDK;
//function
    RobotLocator();
	RobotLocator(const RobotLocator&) = delete;
	RobotLocator& operator=(const RobotLocator&) = delete;
	~RobotLocator();

    void init(KinectInit& kinectDK);
	void updateImage(void);
	void zero(void);
	
	void showPointCloud(void);
	void imageThreshold(float range, float error);
	void barrelLocator(float x, float y, float coorAngle, char colorNum, char barrelNum);
	void rgbdGet(void);
	void contoursDetection(Mat roiImage, 
						vector<vector<Point>> &contours,
						vector<Vec4i> &hierarchy);

	bool roiClassify(const vector<vector<Point> > &contours, 
                    const std::vector<Point2i> &allroiXY, 
                    std::vector<vector<Point2i> > &roiXYs);

	bool pointCloudGenerate(const vector<Point2i > &roiXYs, pPointRgbCloud &outCloud, vector<cv::Point2f> points);
	bool barrelMeasure(const pPointRgbCloud &inCloud, Eigen::Vector4f &centroid);
	float distModel(float distance);
	float angleModel(float angle);
	float compensation(float depth);

	char colorStatus = 0;
	char selectBarrel = 0; 
	char lastBarrel = 0;
	char lastColor = 0;

	float lastX = 0.f; 
	float lastY = 0.f;
	float lastCoorAngle = 0.f;

	cv::Point2f situation;

	float finalAngle = 0.f;
	float finalDistance = 0.f;
	char   cameraState = 0;
	float DistanceModol = 0.f;
	float FinalAngleModol = 0.f;
	int NumCount_distance = 0;
	int NumCount_angle = 0;

private:
	
	int lostCnt = 0;
	int quiverCnt = 0;
	bool changeColor = 0;

//opencv
	cv::Mat				srcImage;

	
	cv::Mat				redRoiImage;
	cv::Mat				blueRoiImage;
	cv::Mat				rgbdImage;
	cv::Mat				blueroiDepthImage;
	cv::Mat				redroiDepthImage;

	vector<cv::Point2f> points;

	
//k4a	
	k4a_image_t transformed_depth_image = NULL;
	k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;

//pcl
	pPointRgbCloud		srcCloud;
	pPointRgbCloud		barrelCloud;
	pcl::visualization::PCLVisualizer::Ptr dstViewer;

//std
	int picNum = 0;
	std::string picName;
	std::vector<Point2i> redRoiXY;
	std::vector<Point2i> blueRoiXY;
	PicRoi colorRoi;
	

//点位
	//中心距
	float Adistance = 280.f;
	float Bdistance = 350.f;
	float Cdistance = 350.f;
	//中心位置
	cv::Point2f WestCenter  = cv::Point2f(5950.f, 3950.f);
	cv::Point2f EastCenter  = cv::Point2f(5950.f, 7950.f);
	cv::Point2f NorthCenter = cv::Point2f(3450.f, 5950.f);	
	cv::Point2f SouthCenter = cv::Point2f(8450.f, 5950.f);
	cv::Point2f MidCenter   = cv::Point2f(5950.f, 5950.f);
	cv::Point2f NorthCenterBarral = cv::Point2f(3450.f, 6300.f);	
	cv::Point2f SouthCenterBarral = cv::Point2f(8450.f, 6300.f);
	cv::Point2f MidCenterBarral   = cv::Point2f(5950.f, 6300.f);
	//桶位置
	cv::Point2f EastRed     = cv::Point2f(5670.f, 7950.f);
	cv::Point2f WestRed     = cv::Point2f(5670.f, 3950.f);
	cv::Point2f EastBlue    = cv::Point2f(6230.f, 7950.f);
	cv::Point2f WestBlue    = cv::Point2f(6230.f, 3950.f);
	//误差
	float disError = 400.f;
	float angleError = 0.f;
	cv::Point2f Barrel  	= cv::Point2f(0.f, 0.f);

};
#endif