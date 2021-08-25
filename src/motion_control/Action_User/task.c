#include <stdio.h>
#define __USE_GNU
#define _GNU_SOURCE
#include <sched.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>   
#include "includes.h"
#include "robot.h"
#include <sys/time.h>
#include "spi_init.h"
#include "timer.h"
#include "mcucomm.h"
#include "pps.h"
#include "moveBase.h"   
#include "gpio.h"
#include "task.h"
#include "can.h"
#include "driver.h"
#include <linux/can.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/sem.h>
#include <semaphore.h>
#include <math.h>
#include "spi_init.h"
#include "timer.h"
#include "includes.h"
#include "path.h"
#include "ringbuffer.h"
#include "Move.h"	
#include "process_comm.h"
#include "cv.h"
extern FILE *fpWrite;
extern char filename[];

extern FILE *sFpWrite;
extern char sFilename[];

gRobot_t gRobot = {0};

/*
   ===============================================================
   初始化任务
   ===============================================================
   */
void ConfigTask(void)
{
	/*初始化*/
	printf("init ..\n");

	ThreadBindCPU();

	gRobot.courdID = BLUE_COURT;	//暂时设置为蓝场
    gRobot.walkStatus = waitForStart;
	gRobot.archeryStart = 1;//开始射箭
	gRobot.initialCnt = 3;

	//初始化定时器
	TimeInit();
	//测试轨迹，记得在Bspline.c修改宏定义
	// TestPath();//测试轨迹

	//改
	BufferZizeInit(200);
	ClearRingBuffer();

	WheelLockTransform(-60.f, 0.f, 60.f);
	printf("waiting for spi init...\n");
	SPIInit();
	printf("spi init done\n"); 

	printf("waiting for mcu init...\n");
	NewSpiThread();
	//捕获 ctrl + c
	signal(SIGINT, fun_sig);
	WaitMcuPrepare();

	WheelLockTransform(-60.f, 0.f, 60.f);

	printf("waitint for CV...\n");
	WaitCvPrepare();

	printf("waiting for pps ...\n");
	WaitPpsPrepare();

	// float initVelAngle = GetAngle()+ORI_TURN_DIR;
	// AngleLimit(&initVelAngle); 
	// OutputVel2Wheel(0.0f,initVelAngle,0.0f);//记得取消屏蔽
	
	// printf("waiting for mcu init...\n");
	// NewSpiThread();
	// //捕获 ctrl + c
	// signal(SIGINT, fun_sig);
	// WaitMcuPrepare();
	printf("init done!!!\n");

	printf("program is running without error...\n");
	
}

void VelCtrlTask(void)
{
	float vel = 0.0f;
	float velDireciton = 0.0f;
	velVector_t actVel = {0.0f};

	//获取实际车速
	actVel.xVel = GetSpeedX();
	actVel.yVel = GetSpeedY();

	vel = sqrtf(actVel.xVel * actVel.xVel + actVel.yVel * actVel.yVel);

	if(vel>0.01f)
	{
		velDireciton = atan2f(actVel.yVel,actVel.xVel)*CHANGE_TO_ANGLE;
	}
	else
	{
		velDireciton = velDireciton;
	}

	VelControl((robotVel_t){vel , velDireciton , GetWZ()});
}

void WalkTask(void)
{
	Walk();
}

extern struct timeval start, end;
extern struct timeval start2, end2;
extern long long total_time;
extern long long total_time2;
long long total_t;
//检测SPI单次时间
extern long long SPItotal_time;

//发送调试数据
void SendDebugInfo(void)
{
	printf("S %d %d %d N %d E %d %d M %d %d %d F %d %d %d %d %d ",\
		//s
		(int)gRobot.walkStatus,(int)gRobot.pathPlanFlag, (int)gRobot.walkMode,\
	 	//N
		(int)gRobot.shootArrowCnt,\
		//E
		(int)gRobot.mcuHeart,(int)gRobot.ppsHeart,\
		//M
		(int)gRobot.defencePotID, (int)gRobot.waveArchery, (int)gRobot.colorFlag,\
		//F
		(int)gRobot.attackPotID,(int)gRobot.archeryStart, (int)gRobot.archeryDone, (int)gRobot.shootArrowCnt, (int)gRobot.TRRetryFlag);

	printf("T %d %lld %lld %lld ",(int)GetTimeCounter(),total_time,total_time2,total_t);

	printf("cv %d %d %d ", (int)(gRobot.cvDir*1000), (int)(gRobot.cvDis*1000),(int)gRobot.cvAttackPotID);

	printf("p %d %d %d %d %d %d %d %d %d ",(int)GetX(),(int)GetY(),(int)(GetAngle()*10),\
		(int)GetSpeedX(),(int)GetSpeedY(),(int)(GetWZ()*10),\
		(int)GetCorrectX(),(int)GetCorrectY(),(int)(GetCorrectAngle()*10));

	printf("vp %d %d %d ",(int)gRobot.debugInfomation.virtualPos.point.x,\
		(int)gRobot.debugInfomation.virtualPos.point.y,\
		(int)gRobot.debugInfomation.virtualPos.direction);

	printf("vt %d %d %d ",(int)gRobot.debugInfomation.virtualTarget.point.x,\
		(int)gRobot.debugInfomation.virtualTarget.point.y,\
		(int)gRobot.debugInfomation.virtualTarget.direction);

	printf("2p %d %d %d z %d %d %d %d L %d %d ",\
		//2p
		(int)gRobot.debugInfomation.disRealPos2VirPos,\
		(int)gRobot.debugInfomation.disRealPos2VirTarget,\
		(int)gRobot.debugInfomation.VIEW_L,\
		//z
		(int)(gRobot.debugInfomation.posAngleVP*10),(int)(gRobot.debugInfomation.posAngleVT*10),\
		(int)(gRobot.debugInfomation.omega*10),(int)(gRobot.debugInfomation.outputOmega*10),\
		//L
		(int)gRobot.debugInfomation.robotlenVP,(int)gRobot.debugInfomation.robotlenVT);

	printf("v %d %d %d %d %d %d %d %d %d %d %d %d %d ",\
		(int)(gRobot.debugInfomation.originVel*cosf(ANGLE2RAD(gRobot.debugInfomation.originVelDir))),\
		(int)(gRobot.debugInfomation.originVel*sinf(ANGLE2RAD(gRobot.debugInfomation.originVelDir))),\
		(int)gRobot.debugInfomation.fixedVel,\
		(int)gRobot.debugInfomation.distance,\
		//4
		(int)(gRobot.debugInfomation.adjustVel.direction * 10),\
		(int)(gRobot.debugInfomation.adjustVel.vel*cosf(ANGLE2RAD(gRobot.debugInfomation.adjustVel.direction))),\
		(int)(gRobot.debugInfomation.adjustVel.vel*sinf(ANGLE2RAD(gRobot.debugInfomation.adjustVel.direction))),\
		(int)(gRobot.debugInfomation.sumVel*cosf(ANGLE2RAD(gRobot.debugInfomation.sumVelDir))),\
		(int)(gRobot.debugInfomation.sumVel*sin(ANGLE2RAD(gRobot.debugInfomation.sumVelDir))),\
		//9
		(int)gRobot.debugInfomation.velXErr,\
		(int)gRobot.debugInfomation.velYErr,\
		(int)(gRobot.debugInfomation.outputVel*cosf(ANGLE2RAD(gRobot.debugInfomation.outputDirection))),\
		(int)(gRobot.debugInfomation.outputVel*sinf(ANGLE2RAD(gRobot.debugInfomation.outputDirection))));

	printf("w %d %d %d %d %d %d \n",\
		(int)gRobot.wheelState.oneTarget.vel,\
		(int)gRobot.wheelState.twoTarget.vel,(int)gRobot.wheelState.thrTarget.vel,\
		(int)gRobot.wheelState.oneTarget.direction,(int)gRobot.wheelState.twoTarget.direction,\
		(int)gRobot.wheelState.thrTarget.direction);	
	printf("\n");
}

void WriteDebugInfo2File(void)
{
	static int debugDelay;
	debugDelay++;
	if(debugDelay > 50)
	{
		debugDelay = 0;
		printf("cv %d %d p %d %d %d mcu %d %d \n", (int)(gRobot.cvDir*100), (int)(gRobot.cvDis*10),(int)GetX(),(int)GetY(),(int)(GetAngle()*10),(int)gRobot.mcuHeart,(int)gRobot.mcuReInitCnt);

	}
	
	fprintf(fpWrite,"SPI %lld %d %d S %d %d %d N %d E %d %d M %d %d %d F %d %d %d ",\
		//SPI
		SPItotal_time,(int)gRobot.mcuHeart,(int)gRobot.mcuReInitCnt,\
		//S
		(int)gRobot.walkStatus,(int)gRobot.pathPlanFlag, (int)gRobot.walkMode,\
		//N
		(int)gRobot.shootArrowCnt,\
		//E
		(int)gRobot.mcuHeart,(int)gRobot.ppsHeart,\
		//M
		(int)gRobot.defencePotID, (int)gRobot.waveArchery, (int)gRobot.colorFlag,\
		//F
		(int)gRobot.attackPotID,(int)gRobot.archeryStart, (int)gRobot.archeryDone);

	fprintf(fpWrite,"T %d %lld %lld %lld ",(int)GetTimeCounter(),total_time,total_time2,total_t);

	fprintf(fpWrite,"cv %d %d TC %d Rt %d %d ", (int)(gRobot.cvDir*1000), (int)(gRobot.cvDis*10),(int)gRobot.takeCnt,\
	(int)gRobot.tryTakeFlag,(int)gRobot.TRRetryFlag);

	fprintf(fpWrite,"p %d %d %d %d %d %d %d %d %d %d %d ",\
		(int)GetX(),(int)GetY(),(int)(GetAngle()*10),\
		(int)GetSpeedX(),(int)GetSpeedY(),(int)(GetWZ()*10),\
		(int)GetCorrectX(),(int)GetCorrectY(),(int)(GetCorrectAngle()*10),\
		(int)GetSpeedWithoutOmega().xVel,(int)GetSpeedWithoutOmega().yVel);

	fprintf(fpWrite,"vp %d %d %d ",(int)gRobot.debugInfomation.virtualPos.point.x,\
		(int)gRobot.debugInfomation.virtualPos.point.y,\
		(int)gRobot.debugInfomation.virtualPos.direction);

	fprintf(fpWrite,"vt %d %d %d ",(int)gRobot.debugInfomation.virtualTarget.point.x,\
		(int)gRobot.debugInfomation.virtualTarget.point.y,\
		(int)gRobot.debugInfomation.virtualTarget.direction);

	fprintf(fpWrite,"2p %d %d %d z %d %d %d %d L %d %d ",(int)gRobot.debugInfomation.disRealPos2VirPos,\
		(int)gRobot.debugInfomation.disRealPos2VirTarget,\
		(int)gRobot.debugInfomation.VIEW_L,\
		(int)(gRobot.debugInfomation.posAngleVP*10),(int)(gRobot.debugInfomation.posAngleVT*10),\
		(int)(gRobot.debugInfomation.omega*10),(int)(gRobot.debugInfomation.outputOmega*10),\
		(int)gRobot.debugInfomation.robotlenVP,(int)gRobot.debugInfomation.robotlenVT);

	fprintf(fpWrite,"v1 %d %d %d v2 %d %d %d %d %d v3 %d %d %d %d ",\
		//v1
		(int)(gRobot.debugInfomation.originVel*cosf(ANGLE2RAD(gRobot.debugInfomation.originVelDir))),\
		(int)(gRobot.debugInfomation.originVel*sinf(ANGLE2RAD(gRobot.debugInfomation.originVelDir))),\
		(int)gRobot.debugInfomation.fixedVel,\
		//v2
		(int)gRobot.debugInfomation.distance,\
		(int)(gRobot.debugInfomation.adjustVel.vel*cosf(ANGLE2RAD(gRobot.debugInfomation.adjustVel.direction))),\
		(int)(gRobot.debugInfomation.adjustVel.vel*sinf(ANGLE2RAD(gRobot.debugInfomation.adjustVel.direction))),\
		(int)(gRobot.debugInfomation.sumVel*cosf(ANGLE2RAD(gRobot.debugInfomation.sumVelDir))),\
		(int)(gRobot.debugInfomation.sumVel*sin(ANGLE2RAD(gRobot.debugInfomation.sumVelDir))),\
		//v3
		(int)gRobot.debugInfomation.velXErr,\
		(int)gRobot.debugInfomation.velYErr,\
		(int)(gRobot.debugInfomation.outputVel*cosf(ANGLE2RAD(gRobot.debugInfomation.outputDirection))),\
		(int)(gRobot.debugInfomation.outputVel*sinf(ANGLE2RAD(gRobot.debugInfomation.outputDirection))));

	fprintf(fpWrite,"w %d %d %d %d %d %d \n",(int)gRobot.wheelState.oneTarget.vel,\
	(int)gRobot.wheelState.twoTarget.vel,(int)gRobot.wheelState.thrTarget.vel,\
	(int)gRobot.wheelState.oneTarget.direction,(int)gRobot.wheelState.twoTarget.direction,\
	(int)gRobot.wheelState.thrTarget.direction);
	fflush(fpWrite);
}

void WriteShootData(void)
{
	fprintf(sFpWrite,"N %d pot %d v %d T %d p %d i %d A %d CV %d dis %d \n",\
	(int)gRobot.shootArrowCnt,(int)gRobot.attackPotID,(int)(gRobot.autoShootVel*100) ,(int)(gRobot.voltageTemp*100),(int)(gRobot.autoShootKp*100),(int)(gRobot.autoShootKi*100),\
	(int)(gRobot.autoPitchAngle*100),(int)(gRobot.cvDir*100),(int)(gRobot.cvDis*100));
	fflush(sFpWrite);
}

void TestTurnWeel(void)
{
	static uint32_t timeCnt = 0;
	static uint8_t flag = 0;
	static int8_t cnt = 0;

	timeCnt++;
	if(timeCnt % 200 == 0)
	{
		if(flag)
		{
			cnt--;
		}
		else
		{
			cnt++;
		}

		if(cnt >=4 )
		{
			flag = 1;
		}
		else if(cnt <= -4)
		{
			flag = 0;
		}
	}

	gRobot.wheelState.oneTarget.vel = 0.0f;
	gRobot.wheelState.oneTarget.direction = 90.0f * cnt;
	
	gRobot.wheelState.twoTarget.vel = 0.0f;
	gRobot.wheelState.twoTarget.direction = 90.0f * cnt;
	
	gRobot.wheelState.thrTarget.vel = 0.0f;
	gRobot.wheelState.thrTarget.direction = 90.0f * cnt;

	Communicate2Mcu();
}

int8_t ThreadBindCPU(void)
{
	 int cpus = 0;
     cpu_set_t mask;
 
     cpus = sysconf(_SC_NPROCESSORS_CONF);
     printf("cpus: %d\n", cpus);
 
     CPU_ZERO(&mask);    /* 初始化set集，将set置为空*/
     CPU_SET(0, &mask);  /* 依次将0、1、2、3号cpu加入到集合，前提是你的机器是多核处理器*/
     
     /*设置cpu 亲和性（affinity）*/
     if (sched_setaffinity(0, sizeof(mask), &mask) == -1) {
         printf("Set CPU affinity failue, ERROR:%s\n", strerror(errno));
         return -1; 
     }   
     usleep(1000); /* 让当前的设置有足够时间生效*/

}

void TestPath(void)
{
	printf("test path running... \r\n");
	BufferZizeInit(200);
	ClearRingBuffer();
	//规划测试轨迹
	InputPoints2RingBuffer(testPath, TEST_PATH_NUM);
	printf("test path planned\r\n");
	while(1);
}

void fun_sig(int sig) //捕获ctrl c
{
    //  第二次收到信号时，以默认的方式处理
	signal(sig,SIG_DFL);
	pthread_cancel(spi_thread_id);//
    printf("\nclose spi thread done!!!\n");
	// for(int i = 0; i<3;i++ )
   	// {
	// 	   tx
	//              }
    // if (VSI_CloseDevice(VSI_USBSPI, 0) != ERR_SUCCESS)
	// {
 	// 	printf("Close device error!\n");
	// }
	// else{
	// 	printf("close spi device done!!!\n");
	// }
	printf("press ctrl c again\n");	   
	exit(0); 
}