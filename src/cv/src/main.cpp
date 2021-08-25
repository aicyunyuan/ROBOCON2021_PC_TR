// 修改人 周瑜杰
// 修改时间 2021-2-27
// 修改内容 运动控制的坐标系是以横向为y，竖向为x，，原点是墙内的红放TR出发区拐点处，对此修改了五个桶中心位置坐标


#include "kinect_init.h"
#include "robot_locator.h"
#include "own_serial.h"

#define COMMUN

int main(int argc, char* argv[])
{
	
	KinectInit KinectDK;
	RobotLocator robotLocator;

	
	//kinect DK init
	KinectDK.init();
 	robotLocator.init(KinectDK);

	#ifdef COMMUN
	//创建共享内存
	ProcessCommInit(MC_CV_ID, CV);
	own_serial ownSerial("/dev/ttyUSB0",B115200);
	#endif
		

	chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;
	auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);

	cv::TickMeter tk;

	while (true)
	{
		
		//总计时函数
		tk.start();
		
		//更新图片
		robotLocator.zero();
		robotLocator.updateImage();
		
		//接收数据
		#ifdef COMMUN
		ownSerial.getMCmessage();
		#endif
		 
		//圆筒定位
		#ifdef COMMUN
		robotLocator.barrelLocator(ownSerial.x, ownSerial.y, ownSerial.angle, ownSerial.color, ownSerial.barrel);
		#else
		robotLocator.barrelLocator(500, 500, 45, RED, MID);
		#endif
		
		//发送数据
		#ifdef COMMUN
		ownSerial.encrypt(robotLocator.finalAngle, robotLocator.finalDistance, robotLocator.cameraState);
		#endif

		//释放图片
		robotLocator.thiskinectDK->release();
		tk.stop();
		cout << " 程序总时间: " << tk.getTimeMilli() << endl;
		cout << endl;
		tk.reset();

		#ifdef VEDIO
		if(waitKey(10) == 'q' || waitKey(10) == 'Q')   //如果按下q退出
		{
			break;
		}
		#endif

	}


	return EXIT_SUCCESS;
}

