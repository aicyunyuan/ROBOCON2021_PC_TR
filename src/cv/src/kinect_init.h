
#define _CRT_SECURE_NO_WARNINGS

#ifndef KINECT_INIT_H_
#define KINECT_INIT_H_
#include <opencv2/opencv.hpp>
#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <pcl/visualization/cloud_viewer.h>



// #define VEDIO
#define TXT

extern "C"
{
	#include "process_comm.h"
}

using namespace std;
using namespace cv;

typedef pcl::PointXYZRGB 			pointRgbType;
typedef pcl::PointCloud<pointRgbType> 	pointRgbCloud;
typedef pointRgbCloud::Ptr 			pPointRgbCloud;

class KinectInit
{
public:
//function
    KinectInit();
	KinectInit(const KinectInit&) = delete;
	KinectInit operator=(const KinectInit&) = delete;
	~KinectInit();
    int init(void);
	void update(void);
    void create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table);
    void generate_point_cloud(const k4a_image_t depth_image,
                                const k4a_image_t xy_table,
                                pPointRgbCloud point_cloud,
                                int *point_count);
	void release(void);
	
//pcl
	pPointRgbCloud pointCloud;
//opencv
	cv::Mat cv_rgbImage_no_alpha;

//k4a
	k4a_calibration_t calibration;
	k4a_image_t rgbImage = NULL;
	k4a_image_t depthImage = NULL;
	k4a_transformation_t transformation = NULL;

//std
	int colorWidth;
	int colorHeight;

private:
 //k4a   
	k4a::image irImage;
    k4a_capture_t capture;
    k4a_device_t device;
	k4a_image_t xy_table;
	

//opencv	
	cv::Mat cv_rgbImage_with_alpha;
	cv::Mat cv_depth;
	cv::Mat cv_depth_8U;
	cv::Mat cv_irImage;
	cv::Mat cv_irImage_8U;

//std
	int pointNum = 0;
	const int32_t TIMEOUT_IN_MS = 1000;
	int count = 0;


};

#endif 