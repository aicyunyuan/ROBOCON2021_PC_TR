#include "robot_locator.h"
 
#ifdef VEDIO
VideoWriter writer("src.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 25.0, Size(1280, 720));
#endif

#ifdef TXT
ofstream fout("data.txt");
ofstream fdout("hidedata.txt");
ofstream frout("recivedata.txt");
#endif

#ifdef SHOWPCL
RobotLocator::RobotLocator():srcCloud(new pointRgbCloud),
barrelCloud(new pointRgbCloud),
dstViewer(new pcl::visualization::PCLVisualizer("Advanced Viewer"))
{
	dstViewer->setBackgroundColor(0.259, 0.522, 0.957);
	dstViewer->addPointCloud<pointRgbType>(barrelCloud, "barrelCloud");
	dstViewer->addCoordinateSystem(0.2, "view point");
	dstViewer->initCameraParameters();

};

#else
RobotLocator::RobotLocator():srcCloud(new pointRgbCloud),
barrelCloud(new pointRgbCloud)
{
};
#endif
RobotLocator::~RobotLocator()
{

};

void RobotLocator::init(KinectInit& kinectDK)
{
    cout << "Initializing locator..." << endl;

	//-- Set input device
	thiskinectDK = &kinectDK;
    

    k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                    thiskinectDK->colorWidth,
                    thiskinectDK->colorHeight,
                    thiskinectDK->colorWidth * (int)sizeof(uint16_t),
                    &transformed_depth_image);

    colorRoi.bottomEdge = thiskinectDK->colorHeight;
    colorRoi.topEdge = 0;
    colorRoi.leftEdge = thiskinectDK->colorWidth/3;
    colorRoi.rightEdge = 2*thiskinectDK->colorWidth/3;
    cout<<" ROI bottomEdge "<<colorRoi.bottomEdge<<" topEdge "<<colorRoi.topEdge;
    cout<<" leftEdge "<<colorRoi.leftEdge<<" rightEdge "<<colorRoi.rightEdge<<endl;
    cout << "Done initialization." << endl;
}
void RobotLocator::updateImage(void)
{
    thiskinectDK->update();
    srcCloud = thiskinectDK->pointCloud;

    srcImage = thiskinectDK->cv_rgbImage_no_alpha;
    
#if (SHOOT_RGB_Camera == 1)

    if(char(cv::waitKey(1)) == 's') 
    {
        picNum++;
        picName= std::to_string(picNum) + ".jpg";
        cv::imwrite(picName, srcImage);

    }

#endif
}
void RobotLocator::showPointCloud(void)
{
    
    
    dstViewer->updatePointCloud(barrelCloud, "barrelCloud");
	dstViewer->spinOnce(1);

    #ifdef VEDIO
    cv::imshow("srcImage",srcImage);
    writer << srcImage;
    cv::waitKey(1);
    #endif 
}

void RobotLocator::imageThreshold(float range, float error)
{
    cv::Mat	hsvImage;
    cvtColor(srcImage, hsvImage, cv::COLOR_BGR2HSV);
    Point2d xy;
    // cout << "max = " << range + error << "\t" << "min = " << range - error << endl;
    cv::Mat	thresholdImage= cv::Mat::zeros(thiskinectDK->colorHeight, thiskinectDK->colorWidth, CV_8UC3);
    blueRoiXY.clear();
    redRoiXY.clear();
    int countColor = 0;
    uint16_t *transformed_depth_image_data = (uint16_t *)(void *)k4a_image_get_buffer(transformed_depth_image);

    for(int i = colorRoi.topEdge;i<colorRoi.bottomEdge;i++)  
    {  
        auto data = hsvImage.ptr(i);
        for(int j = colorRoi.leftEdge;j<colorRoi.rightEdge;j++)  
        {  
            int hChannel = data[j * 3];
            int sChannel = data[j * 3 + 1];
            int vChannel = data[j * 3 + 2];

            // int hChannel = hsvImage.at<Vec3b>(i,j)[0];
            // int sChannel = hsvImage.at<Vec3b>(i,j)[1];
            // int vChannel = hsvImage.at<Vec3b>(i,j)[2];
            
            if(sChannel <= MAX_S && sChannel >= MIN_S
            && vChannel <= MAX_V && vChannel >= MIN_V)
            {
                if(hChannel <= MAX_BLUE_H && hChannel >= MIN_BLUE_H && colorStatus == BLUE && vChannel <= 255 )
                {
                    double pointdepth = transformed_depth_image_data[i * thiskinectDK->colorWidth + j];
                    if(pointdepth < range+error && pointdepth > range-error) //滤除无效点
                    // if(1)
                    {
                        // if(colorStatus == BLUE)
                        #ifdef DEBUG
                        thresholdImage.at<Vec3b>(i,j)[0] = srcImage.at<Vec3b>(i,j)[0];
                        thresholdImage.at<Vec3b>(i,j)[1] = srcImage.at<Vec3b>(i,j)[1];
                        thresholdImage.at<Vec3b>(i,j)[2] = srcImage.at<Vec3b>(i,j)[2];
                        #endif
                        // blueRoiImage.at<uchar>(i,j) = 1;
                        xy.x=j;
                        xy.y=i;
                        blueRoiXY.push_back(xy);
                        blueroiDepthImage.at<uchar>(i,j) = 255;
                    }
                }
                else if(((hChannel <= MAX_RED_H && hChannel >= MIN_RED_H) || (hChannel <= MAX_L_RED_H && hChannel >= MIN_L_RED_H)) && colorStatus == RED)
                {
                    double pointdepth = transformed_depth_image_data[i * thiskinectDK->colorWidth + j];
                    if(pointdepth < range+error && pointdepth >range-error) //滤除无效点
                    // if(1)
                    {
                        // if(colorStatus == RED)
                        #ifdef DEBUG
                        thresholdImage.at<Vec3b>(i,j)[0] = srcImage.at<Vec3b>(i,j)[0];
                        thresholdImage.at<Vec3b>(i,j)[1] = srcImage.at<Vec3b>(i,j)[1];
                        thresholdImage.at<Vec3b>(i,j)[2] = srcImage.at<Vec3b>(i,j)[2];
                        #endif
                        // redRoiImage.at<uchar>(i,j) = 1;
                        xy.x=j;
                        xy.y=i;
                        redRoiXY.push_back(xy); 
                        // redroiDepthImage.at<uchar>(i,j) = pointdepth / 10000.0f * 255;
                        redroiDepthImage.at<uchar>(i,j) = 255;
                    }
                    
                }
                
            }   
        }  
    }  

    #ifdef DEBUG
    cout << "red = " << redRoiXY.size() << "\t" << "blue = " << blueRoiXY.size() << endl;
    cv::imshow("threImage",thresholdImage);
    cv::imshow("blueroiDepthImage",blueroiDepthImage);
    cv::imshow("redRoiImage",redRoiImage);
    #endif
}
//get rgbd
void RobotLocator::rgbdGet(void)
{   
    k4a_transformation_depth_image_to_color_camera(thiskinectDK->transformation, thiskinectDK->depthImage, transformed_depth_image);  
    rgbdImage = cv::Mat(thiskinectDK->colorHeight, thiskinectDK->colorWidth, CV_16UC1, (void *)k4a_image_get_buffer(transformed_depth_image));
}

void RobotLocator::contoursDetection(Mat roiImage, 
                                    vector<vector<Point> > &contours, 
                                    vector<Vec4i> &hierarchy)
{   
    Mat dst = cv::Mat::zeros(thiskinectDK->colorHeight, thiskinectDK->colorWidth, CV_8UC1);
    //imshow("dst",dst);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
    //imshow("kernel",kernel);
	morphologyEx(roiImage, dst, MORPH_CLOSE, kernel);
    //imshow("mor",roiImage);

    #ifdef DEBUG
    Mat cannyOutput = cv::Mat::zeros(thiskinectDK->colorHeight, thiskinectDK->colorWidth, CV_8UC1);
    Canny(dst, cannyOutput, 50, 100, 5, false);
    imshow("canny", cannyOutput);
    imshow("dst", dst);
    #endif

    findContours(dst, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
//     Scalar color( 0, 0, 0);
//     cv::Mat	contoursImage= cv::Mat::zeros(thiskinectDK->colorHeight, thiskinectDK->colorWidth, CV_8UC3);

//      drawContours(srcImage, contours, -1 ,color, 3); //轮廓
//    // imshow("contours1", contoursImage);
//     imshow("contours2",srcImage);

}

bool RobotLocator::roiClassify(const vector<vector<Point> > &contours, 
                            const std::vector<Point2i> &allroiXY, 
                            std::vector<vector<Point2i> > &roiXYs)
{
    roiXYs.clear();
    //判断是否为圆筒轮廓
    if (contours.size() == 0)
    {
        cout<< " Not have enough contours: in function roiClassify " << endl;
        return false;
    }
    RotatedRect rect;
    vector<vector<cv::Point>> filterContours;
    vector<RotatedRect > filterRects;
    for(int i= 0; i<contours.size(); i++)
    {
        rect = minAreaRect(contours[i]);
        
        float height = rect.size.height;
        float width = rect.size.width;
        int angle = static_cast<int>(rect.angle);
        double area = contourArea(contours[i]);
        double length = arcLength(contours[i], true);

        cv::Point2f pt[4] = {};
        rect.points(pt);
        // cout << pt[0].y << "\t" << pt[1].y << "\t" << pt[2].y << "\t" << pt[3].y << "\t" << endl;
        // circle(srcImage, pt[0], 3, Scalar(255, 0, 0), -1, 8);
        // circle(srcImage, pt[1], 3, Scalar(0, 255, 0), -1, 8);
        // circle(srcImage, pt[2], 3, Scalar(0, 255, 255), -1, 8);
        // circle(srcImage, pt[3], 3, Scalar(0, 0, 255), -1, 8);
        if (pt[0].x <= pt[2].x && pt[1].x <= pt[2].x
            && pt[0].x <= pt[3].x && pt[1].x <= pt[3].x
            )
        {
            height = rect.size.width;
            width = rect.size.height;
            // cout << "change" << endl; 
        }
        // std::cout <<" width = " << width << "\t" << "height =" << height << "\t" ;
        // std::cout << "高宽比 = " << height / width << "\t" << std::endl;
        // cout << "angle = " << angle << "\t";
        // std::cout << "高宽比 = " << width / height  << "\t" << std::endl;
        // std::cout << "area = " << area << "\t" << "面积比 = " << abs( ( height * width ) - area )/ area << "\t"  << std::endl;
        if (  
            width > 10 
            && height > 20 
            && width / height < 0.9
            && width / height > 0.47
            &&( (angle > -15 && angle <= 0) 
            || (angle <=-75  && angle >= -90))
            // && fabs(height * width - area) / area < 0.4
            // && fabs((height + width) * 2 - length) / length < 0.2 
            && area > 100
            )

        {
            filterContours.push_back(contours[i]);
            
        } 
    }
    
    // cout << "filterContours = " << filterContours.size() << endl;
    
    //cv::Mat	contoursImage= cv::Mat::zeros(thiskinectDK->colorHeight, thiskinectDK->colorWidth, CV_8UC3);
    
    #ifdef DEBUG
    Scalar color( 255, 255, 255);
    drawContours(srcImage, filterContours, -1 ,color);
    std::cout << "桶的数量　＝　" << filterContours.size() << std::endl;
    #endif


    

    //找出符合的轮廓中的点
    cv::Point2i pt[2] = {};
    vector<Point2i> singleRoixy;
    Point2i roipt;
    if (filterContours.size() > 0)
    {
        for (size_t i = 0; i < filterContours.size(); i++)
        {
            Rect2f boundRect = boundingRect(filterContours[i]);
            cv::Point2f pt[2] = {};
            pt[0].x = boundRect.x + float(boundRect.width)*float(1.f/6.f);
            pt[0].y = boundRect.y + float(boundRect.height)*float(1.f/8.f);

            pt[1].x = boundRect.x + float(boundRect.width)*float(2.f/3.f);
            pt[1].y = boundRect.y + float(boundRect.height)*float(7.f/8.f);
            // cout << pt[0].x << "\t" << pt[0].y << "\t" << pt[1].x << "\t" << pt[1].y << endl;
            singleRoixy.clear();
            for (size_t j = 0; j < allroiXY.size(); j++)
            {
                if(allroiXY[j].x >= pt[0].x && allroiXY[j].x <= pt[1].x
                && allroiXY[j].y >= pt[0].y && allroiXY[j].y <= pt[1].y)
                {
                    roipt.x = allroiXY[j].x;
                    roipt.y = allroiXY[j].y;
                    singleRoixy.push_back(roipt);
                }
            }
            
            if(singleRoixy.size()>0)
            {
                roiXYs.push_back(singleRoixy);
            }
            
        }
        
    }
    if(roiXYs.size()>0)
    {
        return true;

    }
    else
    {
        cout<< " Not find enough roiXYs: in function roiClassify() " << endl;
    }
    

}

float RobotLocator::compensation(float depth)
{
    if(depth<1000)
    {
      return depth;
    }
    float compensation_depth;
    float difference = 0;
    if(depth >= 1000 && depth <= 8500)
    {
      difference = depth*depth*(-2.678e-06) - 0.01708*depth + 5.837;//matlab拟合的曲线
      compensation_depth = depth - difference;
    }
    else
    {
      compensation_depth = depth;
    }
    
    return compensation_depth;
}

bool RobotLocator::pointCloudGenerate(const vector<Point2i > &roiXY, pPointRgbCloud &outCloud, vector<cv::Point2f> points)
{
    if(roiXY.size()==0)
    {
        cout<< " Not have enough roiXYs: in function pointCloudGenerate() " << endl;
        return false;
    }

    outCloud->points.clear();
    points.clear();
    uint16_t *transformed_depth_image_data = (uint16_t *)(void *)k4a_image_get_buffer(transformed_depth_image);
    k4a_float3_t	target_point3d_mm;
    k4a_float2_t 	source_point2d;
    int ifvaule;
    pointRgbType pclpoint;
    cv::Point2f point;
    
    for(int i = 0; i < roiXY.size(); i++)
    {
        source_point2d.xy.x = roiXY[i].x;
        source_point2d.xy.y = roiXY[i].y;

        k4a_calibration_2d_to_3d(&thiskinectDK->calibration,
                                &source_point2d,
                                transformed_depth_image_data[roiXY[i].y * thiskinectDK->colorWidth + int(roiXY[i].x)],
                                K4A_CALIBRATION_TYPE_COLOR,
                                K4A_CALIBRATION_TYPE_COLOR,
                                &target_point3d_mm ,
                                &ifvaule);
        if(ifvaule && target_point3d_mm.xyz.z>0.15)
        {
            pclpoint.x = target_point3d_mm.xyz.x;
            pclpoint.y = -target_point3d_mm.xyz.y;
            pclpoint.z = target_point3d_mm.xyz.z;
            outCloud->points.push_back(pclpoint);
        }
        
    }

    if(outCloud->points.size()>0)
    {
        return true;
    }else
    {   
        cout<< " Not get enough roiXYs: in function pointCloudGenerate() " << endl;
        return false;
    }
    
    
}

bool RobotLocator::barrelMeasure(const pPointRgbCloud &inCloud, Eigen::Vector4f &centroid)
{
    Point3f nearPoint;
    float tempDistance = 0.f;
    float nearDistance = 15000.f;
    for (size_t i = 0; i < inCloud->points.size(); i++)
	{
        tempDistance = sqrtf(inCloud->points[i].z*inCloud->points[i].z + inCloud->points[i].y*inCloud->points[i].y);
	    if(tempDistance < nearDistance)
        {
            nearDistance = tempDistance;
            nearPoint = Point3f(inCloud->points[i].x, inCloud->points[i].y, inCloud->points[i].z);
        }
	}
    // #ifdef SHOWPCL
    // for (int i = 0; i<inCloud->points.size()-1; i++) { //控制n-1趟冒泡
	// 	for (int j = 0; j<inCloud->points.size() - 1 - i; j++)
	// 	{
	// 		if (inCloud->points[j].z > inCloud->points[j+1].z) { //比较相邻的两个元素
	// 			float tmpX, tmpY, tmpZ; //临时变量
	// 			tmpX = inCloud->points[j].x; //交换
    //             tmpY = inCloud->points[j].y; 
    //             tmpZ = inCloud->points[j].z; 
	// 			inCloud->points[j].x = inCloud->points[j + 1].x;
    //             inCloud->points[j].y = inCloud->points[j + 1].y;
    //             inCloud->points[j].z = inCloud->points[j + 1].z;
	// 			inCloud->points[j + 1].x = tmpX;
    //             inCloud->points[j + 1].y = tmpY;
    //             inCloud->points[j + 1].z = tmpZ;
	// 		}
	// 	}
	// }
    // #endif

    // nearPoint = Point3f(centroid[0], centroid[1], centroid[2]);
    Point3f averPoint;
    pointRgbType pclpoint;
    pPointRgbCloud tempPcl(new pointRgbCloud);
    float count = 0.f;
    for (size_t i = 0; i < inCloud->points.size(); i++)
	{
	    if(fabs(inCloud->points[i].y - nearPoint.y) < 10)
        {
            count++;
            averPoint.x += inCloud->points[i].x;
            averPoint.y += inCloud->points[i].y;
            averPoint.z += inCloud->points[i].z;
            
            #ifdef SHOWPCL
            pclpoint.x = inCloud->points[i].x;
            pclpoint.y = inCloud->points[i].y;
            pclpoint.z = inCloud->points[i].z;
            pclpoint.r = 255;
            pclpoint.g = 255;
            pclpoint.b = 255;
            tempPcl->points.push_back(pclpoint);
            #endif
        } 
	}
    #ifdef SHOWPCL
    // cout << "count" << tempPcl->points.size() << endl;
    for (size_t i = 0; i < tempPcl->points.size(); i++)
	{
        pclpoint.x = tempPcl->points[i].x;
        pclpoint.y = tempPcl->points[i].y;
        pclpoint.z = tempPcl->points[i].z;
        pclpoint.r = 255;
        pclpoint.g = 255;
        pclpoint.b = 255;
        barrelCloud->points.push_back(pclpoint);
	}
    // for (size_t i = 0; i < inCloud->points.size(); i++)
	// {
    //     inCloud->points[i].r = 255;
    //     inCloud->points[i].g = 255;
    //     inCloud->points[i].b = 255;
    //     cout << inCloud->points[i].z << "\t";
    //     showPointCloud();
    //     waitKey(50);
	// }
    // cout << endl;
    #endif  

    centroid[0] = averPoint.x/count;
    centroid[1] = averPoint.y/count;
    centroid[2] = averPoint.z/count;
    centroid[3] = 1;
    #ifdef COUTFLAG
    cout << "nearest = " << nearPoint << endl;
    #endif
}
void RobotLocator::barrelLocator(float x, float y, float coorAngle, char colorNum, char barrelNum)
{

    redRoiImage.release();
    blueRoiImage.release();
    blueroiDepthImage.release();
    barrelCloud->clear();
    redRoiImage = cv::Mat::zeros(thiskinectDK->colorHeight, thiskinectDK->colorWidth, CV_8UC1);
    blueRoiImage = cv::Mat::zeros(thiskinectDK->colorHeight, thiskinectDK->colorWidth, CV_8UC1);
    blueroiDepthImage = cv::Mat::zeros(thiskinectDK->colorHeight, thiskinectDK->colorWidth, CV_8UC1);
    redroiDepthImage = cv::Mat::zeros(thiskinectDK->colorHeight, thiskinectDK->colorWidth, CV_8UC1);

    colorStatus = colorNum;
    selectBarrel = barrelNum;
    float error = 0.f;
    
    // x = 5330;
    // y = 11541;
    // coorAngle = 0;
    #ifdef TXT
        frout << "x  " << x << "  y  " << y << "  coorAngle  " << coorAngle << "  colorNum  " << (int)colorNum << "  barrelNum  " << (int)barrelNum << "  lostcnt  " << lostCnt << endl;
    #endif
    if(changeColor)
    {
        colorStatus = colorStatus%2 + 1;
    }
    if(colorNum == 3)
    {
        cameraState = 2;
    }
    #ifdef COUTFLAG
    std::cout << "lostCnt = " << lostCnt << std::endl;
    std::cout << "changeColor" << (int)changeColor << "颜色：" << (int)colorStatus << "  桶：" << (int)selectBarrel << "  x=" << x << "  y=" << y << "  angle=" << coorAngle << std::endl;
    #endif
    // if(lastBarrel != selectBarrel || fabs(x-lastX)>10 || fabs(y-lastY)>10 || fabs(coorAngle-lastCoorAngle) > 0.5)
    if(lastBarrel != selectBarrel)
    {
        colorStatus = colorNum;
        lostCnt = 0;
        quiverCnt = 0;
        changeColor = 0;
        NumCount_angle = 0;
        FinalAngleModol = 0;
        NumCount_distance = 0;
        DistanceModol = 0;
    }
    lastBarrel = selectBarrel;
    // lastX = x; 
	// lastY = y;
	// lastCoorAngle = coorAngle;

    switch (selectBarrel)
    {
        case EAST:
        {
            cout << "EAST" << endl;
            Barrel = EastRed;
            error = disError;
            break;
        }
        case WEST:
        {  
            cout << "WEST" << endl;
            Barrel = WestRed;
            error = disError;
            break;
        }
        case NORTH:
        {
            cout << "NORTH" << endl;
            Barrel = NorthCenter;
            error = disError + Bdistance;
            break;
        }
        case SOUTH:
        {
            cout << "SOUTH" << endl;
            Barrel = SouthCenter;
            error = disError + Bdistance;
            break;
        }
        case MID:
        {
            cout << "MID" << endl;
            Barrel = MidCenter;
            error = disError + Cdistance;
            break;
        }
        default:
        {
            cout << "NO choice" << endl;
            lostCnt = 0;
            quiverCnt = 0;
            changeColor = 0;
            NumCount_angle = 0;
            FinalAngleModol = 0;
            NumCount_distance = 0;
            DistanceModol = 0;
            return;
            break;
        }
        
    }

    float radius = sqrt(pow(fabs(x - Barrel.x), 2) + pow(fabs(y - Barrel.y), 2)) - 168.79;
    // if(changeColor==1 && (selectBarrel != EAST || selectBarrel != WEST)) radius = radius - 350; //可删除
    #ifdef TR
    bool transformFlag = 0;
    if(radius > 8500  && x<1500 && y<1500)
    {
        transformFlag = 1;
        Barrel = WestRed;
        error  = disError;
        selectBarrel = WEST;
        Point2f vector1 = Point2f(WestRed.x-x, WestRed.y-y);
        Point2f vector2 = Point2f(SouthCenterBarral.x-x, SouthCenterBarral.y-y);
        float tan1 = vector1.y/vector1.x;
        float tan2 = vector2.y/vector2.x;
        float tempAngle = acosf((vector1.x*vector2.x + vector1.y*vector2.y)/
                            (sqrtf(vector1.x*vector1.x + vector1.y*vector1.y)*sqrtf(vector2.x*vector2.x + vector2.y*vector2.y)));
        // cout << "vector1 = " << vector1 << "\tvector2 = " << vector2 << endl;
        // cout << "tempAngle = " << tempAngle << endl;
        if(tan1 < tan2)
        {
            angleError = -(tempAngle/CV_PI)*180.f;
        }
        else
        {
            angleError = (tempAngle/CV_PI)*180.f;
        }
        radius = sqrt(pow(fabs(x - Barrel.x), 2) + pow(fabs(y - Barrel.y), 2)) - 168.79;
        // cout << "angleError" << angleError << endl;
    } 
    #endif
    #ifdef DR
    bool transformFlag = 0;
    if(x>4950 && x<5950 && y>10400 && selectBarrel==MID)
    {
        lostCnt = 0;
        transformFlag = 1;
        Barrel = EastRed;
        error  = disError;
        selectBarrel = EAST;
        Point2f vector1 = Point2f(EastRed.x-x, EastRed.y-y);
        Point2f vector2 = Point2f(MidCenterBarral.x-x, MidCenterBarral.y-y);
        float tan1 = vector1.y/vector1.x;
        float tan2 = vector2.y/vector2.x;
        float tempAngle = acosf((vector1.x*vector2.x + vector1.y*vector2.y)/
                            (sqrtf(vector1.x*vector1.x + vector1.y*vector1.y)*sqrtf(vector2.x*vector2.x + vector2.y*vector2.y)));
        // cout << "vector1 = " << vector1 << "\tvector2 = " << vector2 << endl;
        // cout << "tempAngle = " << tempAngle << endl;
        if(tan1 < tan2)
        {
            angleError = -(tempAngle/CV_PI)*180.f;
        }
        else
        {
            angleError = (tempAngle/CV_PI)*180.f;
        }
        radius = sqrt(pow(fabs(x - Barrel.x), 2) + pow(fabs(y - Barrel.y), 2)) - 168.79;
        // cout << "angleError" << angleError << endl;
    } 
    #endif
    #ifdef TIME
    double time0 = static_cast<double>(cv::getTickCount());//记录起始时间
    #endif
    //获得rgbd图像 大小与彩色图相同 只有一个通道 为深度值
    rgbdGet();


    #ifdef TIME
    double time1=((double)cv::getTickCount()-time0)/cv::getTickFrequency();//time1
    #endif
    
    //HSV划定roi  得到对应的深度图
    #ifdef COUTFLAG
    cout << "error = " << error << "  radius = " << radius << endl;
    #endif
    imageThreshold(radius, error);
    k4a_float3_t	target_point3d_mm;
    k4a_float2_t 	source_point2d;
    int ifvaule;
    
    pointRgbType point;
    vector<vector<Point2i> > roiXYs;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    #ifdef TIME
    double time2=((double)cv::getTickCount()-time0)/cv::getTickFrequency();//time2
    #endif
    
    //对roi进行寻找边缘
    if(colorStatus == BLUE ) contoursDetection(blueroiDepthImage, contours, hierarchy); 
    else            contoursDetection(redroiDepthImage, contours, hierarchy); 

    #ifdef TIME
    double time3=((double)cv::getTickCount()-time0)/cv::getTickFrequency();//time3  
    #endif
    
    //边缘进行分类与筛选transformFlag
    if(colorStatus == BLUE) roiClassify(contours, blueRoiXY, roiXYs);
    else            roiClassify(contours, redRoiXY, roiXYs);


    #ifdef TIME
    double time4=((double)cv::getTickCount()-time0)/cv::getTickFrequency();//time4
    #endif

    //double center_x, center_z, radius;
    Eigen::Vector4f centroid;
    int minx = thiskinectDK->colorHeight;
    int minxnum = 0;
    // cout << "roiXYs.size() = " << roiXYs.size() << endl; 
    if(roiXYs.size()>0)
    {
        
        for (size_t i = 0; i < roiXYs.size(); i++)
        {
            if(abs(abs(roiXYs[i][0].y)-thiskinectDK->colorHeight/2) < minx)               
            {
                minxnum = i;
                minx = abs(abs(roiXYs[i][0].y)-thiskinectDK->colorHeight/2);
            }
        }
        //生成点云
        pointCloudGenerate(roiXYs[minxnum], barrelCloud, points);
        //计算质心与角度
        // pcl::compute3DCentroid(*barrelCloud,centroid);
        barrelMeasure(barrelCloud,centroid);
        //circleFit.circleFitL1(points, center_x, center_z, radius);
        // std::cout << "y = " << centroid[1] << " z = " << centroid[2] << " radius = " << radius << endl;
        float barralAngle = atanf(fabs(centroid[1]/centroid[2]));
        float errorcos = cosf(barralAngle);
        float errorsin = sinf(barralAngle);
        if( centroid[1] > 0.f)
        {
            centroid[1] = centroid[1] + errorsin*157.5 - 88.75;
            centroid[2] = centroid[2] + errorcos*157.5 + 168.79;
        }
        else
        {
            centroid[1] = centroid[1] - errorsin*157.5 - 88.75;
            centroid[2] = centroid[2] + errorcos*157.5 + 168.79;
        }
        // float var = (centroid[0] < 0) ? -1.0 : 1.0;
        #ifdef COUTFLAG
        std::cout << "x = " << centroid[0] << "\t" << "y = " << centroid[1] << "\t" <<"z = " << centroid[2] << "\t";
        #endif
        float angle = atan(centroid[1] / centroid[2]) / CV_PI * 180.0f + angleError;
        float distance = sqrt(centroid[1]*centroid[1] + centroid[2]*centroid[2]);
        if(lostCnt > 20 || quiverCnt > 5)
        {
            lostCnt = 0;
            quiverCnt = 0;
            changeColor = !changeColor;
        }
        if(changeColor)
        {
            float Beta = 0;
            Point2f BarrelCenter;
            Point2f coordinate, carCoor;
            float compensationAngle = 0.f;
            switch (selectBarrel)
            {
                case EAST:
                {
                    BarrelCenter = EastCenter;
                    #ifdef TR
                    compensationAngle = 0.0f;
                    #endif
                    break;
                }
                case WEST:
                {  
                    BarrelCenter = WestCenter;
                    #ifdef TR
                    compensationAngle = 0.8f;
                    #endif
                    break;
                }
                case NORTH:
                {
                    BarrelCenter = NorthCenter;
                    #ifdef TR
                    if(x<1000 && y<1000 && colorNum==RED)
                        compensationAngle = 0.f;
                    else if(x<1000 && y<1000 && colorNum==BLUE)
                        compensationAngle = 1.f;
                    else if(colorNum==BLUE)
                        compensationAngle = 1.5f;
                    else
                        compensationAngle = 0.f;
                    #endif
                    #ifdef DR
                    compensationAngle = 1.6f;

                    #endif
                    break;
                }
                case SOUTH:
                {
                    BarrelCenter = SouthCenter;
                    #ifdef TR
                    if(colorNum == RED)
                        compensationAngle = 1.4f;
                    else if(colorNum == BLUE)
                        compensationAngle = 1.4f;
                    #endif
                    #ifdef DR
                    compensationAngle = 0.6f;
                    #endif
                    break;
                }
                case MID:
                {
                    BarrelCenter = MidCenter;
                    #ifdef TR
                    if(colorNum == RED)
                        compensationAngle = 1.0;
                    else if(colorNum == BLUE)
                        compensationAngle = 1.0f;
                    #endif
                    #ifdef DR
                    compensationAngle = 1.0f;
                    #endif
                    break;
                }
                default:
                {
                    return;
                    break;
                }
            }
            angle = (angle/180.f)*CV_PI;
            float radAngle = (coorAngle/180.f)*CV_PI;
            Beta = atan2f((BarrelCenter.y - y), (BarrelCenter.x - x));
            carCoor = Point2f(distance*cosf(radAngle - angle), distance*sinf(radAngle - angle));
            // fout << "cos  " << cosf(radAngle - angle) << "  sin  " << sinf(radAngle - angle) << "  ";
            float Xc = 2.f*BarrelCenter.x - 2.f*x - carCoor.x;
            float Yc = 2.f*BarrelCenter.y - 2.f*y - carCoor.y;
            coordinate = Point2f(Xc, Yc);
            float a = Xc*carCoor.x + Yc*carCoor.y;
            float b = sqrt(carCoor.x*carCoor.x + carCoor.y*carCoor.y);
            float c = sqrt(Yc*Yc + Xc*Xc);
            float rotateAngle = acosf(a/(b*c));
            float tan1 = atan2f(Yc, Xc);
            float tan2 = atan2f(carCoor.y, carCoor.x);
            if(tan1 > tan2) rotateAngle = -rotateAngle;
            angle = angle + rotateAngle;
            angle = (angle/CV_PI)*180.f + compensationAngle;
            distance = sqrt(Xc*Xc + Yc*Yc);
            #ifdef TXT
            fdout << "x  " << x << "  y  " << y << "  coorAngle  " << coorAngle << "  angle  " << angle << "  carCoorX  " << carCoor.x << "  carCoorY  " << carCoor.y << "  Xc  " << Xc 
                 << "  Yc  " << Yc << "  rotateAngle  " << (rotateAngle/CV_PI)*180.f << "   change  " << (int)changeColor << "   distance  " << distance << "  compensationAngle  " << compensationAngle << std::endl;
            #endif
        }
        #ifdef TR
        if(transformFlag == 1)
        {
            distance = sqrt(powf(SouthCenterBarral.x - x, 2) + powf(SouthCenterBarral.y - y, 2));
        }
        #endif
        #ifdef DR
        
        if(transformFlag == 1)
        {
            distance = sqrt(powf(MidCenterBarral.x - x, 2) + powf(MidCenterBarral.y - y, 2));
        }
        #endif
        
        if(fabsf(angle) > 12)
        {
            lostCnt++;
            cameraState = 0;
            finalAngle = 0.f;
            finalDistance = 0.f;
            return;
        }
        if(lostCnt > 3)
        {
            quiverCnt++;
        }
        lostCnt = 0;
        
        finalAngle = angleModel(angle);
        finalDistance = distModel(distance);
        cameraState = 1;
        #ifdef COUTFLAG
        // std::cout << "x = " << centroid[0] << "\t" << "y = " << centroid[1] << "\t" <<"z = " << centroid[2] << "\t";
        cout << "demo distance = " << distance << endl;
        
        #endif
        std::cout << "angle = " << finalAngle << "\t" <<"distance = " << finalDistance << std::endl;
        #ifdef TXT
        fout << "barral  " << (int)barrelNum << "  color  " << (int)colorNum << "  x  " << x << "  y  " << y << "  coorAngle  " << coorAngle << "  angle  " << angle 
            << "  finalAngle  " << finalAngle  << "  distance  " <<  distance << "  finalDistance  " << finalDistance << "   change  " << (int)changeColor << std::endl;
        #endif
        //std::cout << "colorStatus = " << colorStatus << std::endl;
        //barrelMeasure(barrelCloud);
    }
    else if(lostCnt > 20 || quiverCnt > 5)
    {
        lostCnt = 0;
        quiverCnt = 0;
        changeColor = !changeColor;
    }
    else
    {
        lostCnt++;
        cameraState = 0;
        // #ifdef TXT
        // fout << "not find" <<std::endl;
        // #endif
    }
    
    
    
    #ifdef TIME
    double time5=((double)cv::getTickCount()-time0)/cv::getTickFrequency();//time5
    #endif
    #ifdef DEBUG
        imshow("srcImage", srcImage);
    #endif
    #ifdef IMAGE
        imshow("srcImage", srcImage);
        waitKey(1);
    #endif
    #ifdef SHOWPCL
        showPointCloud();
    #endif
    
    #ifdef TIME
        double time6=((double)cv::getTickCount()-time0)/cv::getTickFrequency();//time5
        std::cout << "time1 = " << ((time1-time0)*1000) << " ms" << "\t";
        std::cout << "time2 = " << ((time2-time1)*1000) << " ms" << "\t";
        std::cout << "time3 = " << ((time3-time2)*1000) << " ms" << "\t";
        std::cout << "time4 = " << ((time4-time3)*1000) << " ms" << "\t";
        std::cout << "time5 = " << ((time5-time4)*1000) << " ms" << "\t";
        std::cout << "time6 = " << ((time6-time5)*1000) << " ms" << "\t";
        std::cout << "all time == " << (time6)*1000 << "ms" << std::endl;
    #endif
    
    
}

float RobotLocator::distModel(float distance)
{
    
    if(isnanf(distance) || isinff(distance)) {
        return 0.f;
    }
    else{
        float difference=fabs(distance-DistanceModol);
        if(difference>80)
        {
            NumCount_distance = 0;
            DistanceModol = distance;
        }
        else
        {
            NumCount_distance++;
            DistanceModol = (DistanceModol*(NumCount_distance-1)+distance)/NumCount_distance;
            if(NumCount_distance >= 10)
            {
                NumCount_distance = 10;
            }
            // cout 
        }
        return(DistanceModol);
    }
    
}

float RobotLocator::angleModel(float angle)
{
    if(isnanf(angle) || isinff(angle)) {
        return 0.f;
    }
    else{
        if(fabs(angle) > 2){
            NumCount_angle = 0;
            FinalAngleModol = 0;
            return angle;
        }
        float difference=fabs(angle-FinalAngleModol);
        //cout<<"角度==="<<(rad/CV_PI)*180<<endl;
        if(difference>0.6) {
            NumCount_angle = 0;
            FinalAngleModol = angle;
        }
        else{
            NumCount_angle++;
            FinalAngleModol = (FinalAngleModol*(NumCount_angle-1)+angle)/NumCount_angle;
            if(NumCount_angle >= 10)
            {
                NumCount_angle = 10;
            }
        }
        return(FinalAngleModol);
    }
    
}

void RobotLocator::zero(void)
{
    colorStatus = 0;
    finalAngle = 0.f;
	finalDistance = 0.f;
    cameraState = 0;
    angleError = 0.f;
}



