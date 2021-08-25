#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include "robot.h"
#include "posSystem.h"
#include "path.h"
#include "pps.h"
#include "mcucomm.h"
#include "pathFollowing.h"
#include "ringbuffer.h"
#include "Move.h"
#include "moveBase.h"
/*  ********************************************	计时	***************************************************** */
/*  ****************************记得所有在射箭区的轮子抱死******************/
static int timeCounter = 0;
static uint8_t countTimeFlag = 0;
extern FILE *fpWrite;

int GetTimeCounter(void)
{
	return timeCounter;
}

void CountTime(void)
{
	if (countTimeFlag)
	{
		timeCounter++;
	}
	timeCounter %= 500000000;
}

void SetCountTimeFlag(void)
{
	countTimeFlag = 1;
}

/*  ********************************************Walk***************************************************** */
static int delayCount = 0;
static int firstPathFlag = 0;
void Walk(void)
{
	// gRobot.walkStatus = testPara;
	//mcu连续通信失败20次 停下
	if (gRobot.mcuHeart > 20)
	{
		fprintf(fpWrite, " MCU ERR! ");
		printf(" MCU ERR! ");
		gRobot.walkStatus = stop;
		SPIInit();
		gRobot.mcuHeart = 0;
		gRobot.mcuReInitCnt ++;
	}
	//pps连续通信失败20次 停下
	if (gRobot.ppsHeart > 20)
	{
		fprintf(fpWrite, " pps ERR ");
		printf(" pps ERR ");
		gRobot.walkStatus = stop;
	}
	if (gRobot.ppsSameHeart > 20)
	{
		fprintf(fpWrite, " pps same ERR ");
		printf(" pps same ERR ");
		gRobot.walkStatus = stop;
	}
	if (gRobot.TRRetryFlag == 2) //重试取箭
	{
		gRobot.walkStatus = waitForTakeAgain; //重新回到出发区
	}
	switch (gRobot.walkStatus)
	{
	//等待触发(在出发区射3支箭)
	case waitForStart:
	{
		//开始计时
		SetCountTimeFlag();
		// float initVelAngle = 0;//记得改回来
		// initVelAngle = GetAngle()+ORI_TURN_DIR;
		// AngleLimit(&initVelAngle);
		// OutputVel2Wheel(0.0f,initVelAngle,0.0f);

		//banche fetch again
		if(gRobot.TRRetryFlag == 1)
		{
			gRobot.initialCnt = 5;
			gRobot.tryTakeFlag = 3;
		}
		if(gRobot.tryTakeFlag == 3)
		{
			if(gRobot.walkMode == 1)
			{
				gRobot.walkStatus = waitForFirstPlace;
			}
			else if(gRobot.walkMode == 2)
			{
				gRobot.walkStatus = waitForSecPlace;
			}
		}

		if ((gRobot.fetchReady == 2) || gRobot.tryTakeFlag == 1 || gRobot.TRRetryFlag == 4) //一开始和取箭重试
		{
			float initVelAngle = 0;
			initVelAngle = GetAngle() + ORI_TURN_DIR;
			AngleLimit(&initVelAngle);
			OutputVel2Wheel(0.0f, initVelAngle, 0.0f);

			gRobot.pathPlanFlag = IS_PLANNING; //防止路径规划时间过长触发错误检测
			Communicate2Mcu();
			ClearRingBuffer();

			//规划第一段轨迹
			AttackPath1[0].point.x = GetX();
			AttackPath1[0].point.y = GetY();
			AttackPath1[0].direction = GetAngle();
			InputPoints2RingBuffer(AttackPath1, ATTACK_PATH_1_NUM);
			gRobot.pathPlanFlag = NO_PLANNING;
		}
		// printf("R %d %d ",  gRobot.tryTakeFlag, gRobot.shootArrowCnt);

		if ((gRobot.tryTakeFlag == 1) || (gRobot.TRRetryFlag == 0 && gRobot.fetchReady == 2) || gRobot.TRRetryFlag == 4) //射完箭后启动走形或取箭重试
		{
			gRobot.archeryStart = 0; //开始行走，不射箭

			if (gRobot.tryTakeFlag == 1)
			{
				gRobot.takeCnt--; //重试，取箭次数减1
				gRobot.tryTakeFlag = 0;
			}

			gRobot.walkStatus = goForFirstPath;

			ClearPathLen(); //后清除记录的轨迹长度
		}
		// else if(gRobot.shootArrowCnt == 3 && gRobot.TRRetryFlag == 1 && gRobot.retryTakeArrow == 0)//重试
		// {
		// 	gRobot.archeryStart = 0;//开始行走，不射箭

		// 	gRobot.walkStatus = stop;

		// 	ClearPathLen();//后清除记录的轨迹长度
		// }
		break;
	}

	//取箭轨迹
	case goForFirstPath:
	{
		static int takeCnt = 0;
		float dis2FinalX = GetX() - AttackPath1[ATTACK_PATH_1_NUM_DEF - 1].point.x;
		float dis2FinalY = GetY() - AttackPath1[ATTACK_PATH_1_NUM_DEF - 1].point.y;
		float dis2FinalAngle = GetAngle() - AttackPath1[ATTACK_PATH_1_NUM_DEF - 1].direction;

		AngleLimit(&dis2FinalAngle);
		if (FetchDone())
		{

			// takeCnt++;
			// fprintf(fpWrite,"Tcn %d ",takeCnt);

			// if(takeCnt > 3)
			// {
			takeCnt = 10;
			OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);

			gRobot.archeryStart = 0;

			if (gRobot.walkMode == 1) //1号射箭区
			{
				gRobot.takeCnt++; //取道箭后加1

				gRobot.walkStatus = waitForForthPath;
				// gRobot.walkStatus = stop;
			}
			else if (gRobot.walkMode == 2) //2号射箭区
			{
				gRobot.takeCnt++;

				gRobot.walkStatus = waitForEighthPath;
				// gRobot.walkStatus = stop;
			}

			// gRobot.walkStatus = stop;
			return;
			// }
		}

		//跟随轨迹
		PathFollowing(0.65f); //0.65

		break;
	}

	
	//取完箭，去射箭
	case waitForForthPath:
	{
		OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);

		ClearRingBuffer();
		gRobot.pathPlanFlag = IS_PLANNING; //防止路径规划时间过长触发错误检测

		if (gRobot.walkMode == 1)
		{
			//规划第三段轨迹
			AttackPath4[0].point.x = GetX();
			AttackPath4[0].point.y = GetY();
			AttackPath4[0].direction = GetAngle();
			InputPoints2RingBuffer(AttackPath4, ATTACK_PATH_4_NUM);

			OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);

			gRobot.pathPlanFlag = NO_PLANNING; //防止路径规划时间过长触发错误检测
			gRobot.walkStatus = goForForthPath;
		}

		if (gRobot.walkMode == 2) //2号射箭区
		{
			gRobot.walkStatus = waitForEighthPath;
			// gRobot.walkStatus = stop;
		}

		gRobot.archeryStart = 0; //stop archerying

		break;
	}
	case goForForthPath:
	{
		static int stopCnt = 0;
		static int takeCnt = 0;
		float dis2FinalX = GetX() - AttackPath4[ATTACK_PATH_4_NUM - 1].point.x;
		float dis2FinalY = GetY() - AttackPath4[ATTACK_PATH_4_NUM - 1].point.y;
		float dis2FinalAngle = GetAngle() - AttackPath4[ATTACK_PATH_4_NUM - 1].direction;

		AngleLimit(&dis2FinalAngle);

		if (sqrtf(dis2FinalX * dis2FinalX + dis2FinalY * dis2FinalY) < 12.0f && JudgeSpeedLessEqual(88.0f) && fabs(dis2FinalAngle) <= 0.8f && fabs(GetWZ()) < 15.0f)
		{
			// OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			WheelLockTransform(-60.f, 0.f, 60.f);

			gRobot.archeryStart = 1; //停下来开始射箭
			gRobot.walkStatus = waitForFifthPath;
		}
		else if (sqrtf(dis2FinalX * dis2FinalX + dis2FinalY * dis2FinalY) < 45.0f && JudgeSpeedLessEqual(88.0f) && fabs(dis2FinalAngle) <= 0.8f && fabs(GetWZ()) < 15.0f)
		{
			stopCnt++;
			if(stopCnt > 80)
			{
				stopCnt = 0;
				// OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
				WheelLockTransform(-60.f, 0.f, 60.f);

				gRobot.archeryStart = 1; //停下来开始射箭
				gRobot.walkStatus = waitForFifthPath;
			}
			
		}
		//跟随轨迹
		PathFollowing(0.55f);//0.45

		break;
	}
	//射完箭去取箭
	case waitForFifthPath:
	{
		// OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
		WheelLockTransform(-60.f, 0.f, 60.f);

		ClearRingBuffer();

		LidarCorrect();

		gRobot.archeryStart = 1;

		if (gRobot.fetchReady == 2) //射箭完成
		{
			gRobot.pathPlanFlag = IS_PLANNING; //防止路径规划时间过长触发错误检测
			//规划第三段轨迹
			AttackPath5[0].point.x = GetX();
			AttackPath5[0].point.y = GetY();
			AttackPath5[0].direction = GetAngle();
			InputPoints2RingBuffer(AttackPath5, ATTACK_PATH_5_NUM);

			OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);
			gRobot.archeryStart = 0;
			gRobot.pathPlanFlag = NO_PLANNING; //防止路径规划时间过长触发错误检测

			gRobot.walkStatus = goForFifthPath;
		}

		if (gRobot.walkMode == 2) //2号射箭区(1到2)
		{
			gRobot.walkStatus = waitForNinthPath;
		}
		// gRobot.walkStatus = stop;
		break;
	}
	case goForFifthPath:
	{
		float dis2FinalX = GetX() - AttackPath5[ATTACK_PATH_5_NUM - 1].point.x;
		float dis2FinalY = GetY() - AttackPath5[ATTACK_PATH_5_NUM - 1].point.y;
		float dis2FinalAngle = GetAngle() - AttackPath5[ATTACK_PATH_5_NUM - 1].direction;

		AngleLimit(&dis2FinalAngle);

		if (FetchDone())
		{
			gRobot.takeCnt++;

			OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);

			gRobot.archeryStart = 0;

			gRobot.walkStatus = waitForForthPath;
			// gRobot.walkStatus = stop;
			return;
		}
		//跟随轨迹
		PathFollowing(0.51f);

		break;
	}

	//去射箭区射箭(DR进攻)
	case waitForEighthPath:
	{
		OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);

		ClearRingBuffer();
		gRobot.pathPlanFlag = IS_PLANNING; //防止路径规划时间过长触发错误检测

		//规划第三段轨迹
		AttackPath8[0].point.x = GetX();
		AttackPath8[0].point.y = GetY();
		AttackPath8[0].direction = GetAngle();
		InputPoints2RingBuffer(AttackPath8, ATTACK_PATH_8_NUM);

		OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);
		gRobot.archeryStart = 0;
		gRobot.pathPlanFlag = NO_PLANNING; //防止路径规划时间过长触发错误检测

		gRobot.walkStatus = goForEighthPath;
		// gRobot.walkStatus = stop;
		break;
	}
	//取箭处到2号射箭区
	case goForEighthPath:
	{
		static int stopCnt = 0;
		float dis2FinalX = GetX() - AttackPath8[ATTACK_PATH_8_NUM - 1].point.x;
		float dis2FinalY = GetY() - AttackPath8[ATTACK_PATH_8_NUM - 1].point.y;
		float dis2FinalAngle = GetAngle() - AttackPath8[ATTACK_PATH_8_NUM - 1].direction;

		AngleLimit(&dis2FinalAngle);

		if (sqrtf(dis2FinalX * dis2FinalX + dis2FinalY * dis2FinalY) < 20.0f && JudgeSpeedLessEqual(100.0f) && fabs(dis2FinalAngle) <= 1.5f && fabs(GetWZ()) < 15.0f)
		{
			// OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			WheelLockTransform(-60.f, 0.f, 60.f);

			gRobot.archeryStart = 1;

			gRobot.walkStatus = waitForArchery;
			// gRobot.walkStatus = stop;

			return;
		}
		else if (sqrtf(dis2FinalX * dis2FinalX + dis2FinalY * dis2FinalY) < 80.0f && JudgeSpeedLessEqual(100.0f) && fabs(dis2FinalAngle) <= 1.5f && fabs(GetWZ()) < 15.0f)
		{
			stopCnt++;
			if (stopCnt > 50)
			{
				stopCnt = 0;
				// OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
				WheelLockTransform(-60.f, 0.f, 60.f);

				gRobot.archeryStart = 1;

				gRobot.walkStatus = waitForArchery;
				// gRobot.walkStatus = stop;

				return;
			}
		}
		//跟随轨迹
		PathFollowing(0.4f); //0.6

		break;
	}
	//1号射箭区到2号射箭区
	case waitForNinthPath:
	{
		OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);

		ClearRingBuffer();
		gRobot.pathPlanFlag = IS_PLANNING; //防止路径规划时间过长触发错误检测

		//规划第三段轨迹
		AttackPath9[0].point.x = GetX();
		AttackPath9[0].point.y = GetY();
		AttackPath9[0].direction = GetAngle();
		InputPoints2RingBuffer(AttackPath9, ATTACK_PATH_9_NUM);

		OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);
		gRobot.archeryStart = 0;
		gRobot.pathPlanFlag = NO_PLANNING; //防止路径规划时间过长触发错误检测

		gRobot.walkStatus = goForNinthPath;
		// gRobot.walkStatus = stop;
		break;
	}
	case goForNinthPath:
	{
		float dis2FinalX = GetX() - AttackPath9[ATTACK_PATH_9_NUM - 1].point.x;
		float dis2FinalY = GetY() - AttackPath9[ATTACK_PATH_9_NUM - 1].point.y;
		float dis2FinalAngle = GetAngle() - AttackPath9[ATTACK_PATH_9_NUM - 1].direction;

		AngleLimit(&dis2FinalAngle);

		if (sqrtf(dis2FinalX * dis2FinalX + dis2FinalY * dis2FinalY) < 80.0f && JudgeSpeedLessEqual(100.0f) && fabs(dis2FinalAngle) <= 1.5f && fabs(GetWZ()) < 15.0f)
		{
			OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);
			// WheelLockTransform(30.f, -90.f, -30.f);

			gRobot.archeryStart = 1;

			//gRobot.walkStatus = waitForSixthPath;
			gRobot.walkStatus = waitForArchery;

			return;
		}
		//跟随轨迹
		PathFollowing(0.2f);

		break;
	}
	//2号射箭区到1号射箭区
	case waitForTenthPath:
	{
		OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);

		ClearRingBuffer();
		gRobot.pathPlanFlag = IS_PLANNING; //防止路径规划时间过长触发错误检测

		//规划第三段轨迹
		AttackPath10[0].point.x = GetX();
		AttackPath10[0].point.y = GetY();
		AttackPath10[0].direction = GetAngle();
		InputPoints2RingBuffer(AttackPath10, ATTACK_PATH_10_NUM);

		OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);
		gRobot.archeryStart = 0;
		gRobot.pathPlanFlag = NO_PLANNING; //防止路径规划时间过长触发错误检测

		gRobot.walkStatus = goForTenthPath;
		// gRobot.walkStatus = stop;
		break;
	}
	case goForTenthPath:
	{
		float dis2FinalX = GetX() - AttackPath10[ATTACK_PATH_10_NUM - 1].point.x;
		float dis2FinalY = GetY() - AttackPath10[ATTACK_PATH_10_NUM - 1].point.y;
		float dis2FinalAngle = GetAngle() - AttackPath10[ATTACK_PATH_10_NUM - 1].direction;

		AngleLimit(&dis2FinalAngle);

		if (sqrtf(dis2FinalX * dis2FinalX + dis2FinalY * dis2FinalY) < 80.0f && JudgeSpeedLessEqual(100.0f) && fabs(dis2FinalAngle) <= 1.5f && fabs(GetWZ()) < 15.0f)
		{
			// OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			WheelLockTransform(-60.f, 0.f, 60.f);

			gRobot.archeryStart = 1;

			gRobot.walkStatus = waitForFifthPath;
			// gRobot.walkStatus = stop;

			return;
		}
		//跟随轨迹
		PathFollowing(0.3f);

		break;
	}

	//2号射箭区去取箭
	case waitForEleventhPath:
	{
		WheelLockTransform(-60.f, 0.f, 60.f);
		// OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);

		ClearRingBuffer();

		if ( gRobot.fetchReady == 2) //射箭完成
		{
			gRobot.pathPlanFlag = IS_PLANNING; //防止路径规划时间过长触发错误检测

			//规划第三段轨迹
			AttackPath11[0].point.x = GetX();
			AttackPath11[0].point.y = GetY();
			AttackPath11[0].direction = GetAngle();
			InputPoints2RingBuffer(AttackPath11, ATTACK_PATH_11_NUM);

			OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);
			gRobot.archeryStart = 0;
			gRobot.pathPlanFlag = NO_PLANNING; //防止路径规划时间过长触发错误检测

			gRobot.walkStatus = goForEleventhPath;
			// gRobot.walkStatus = stop;
		}
		break;
	}
	case goForEleventhPath:
	{
		float dis2FinalX = GetX() - AttackPath11[ATTACK_PATH_11_NUM - 1].point.x;
		float dis2FinalY = GetY() - AttackPath11[ATTACK_PATH_11_NUM - 1].point.y;
		float dis2FinalAngle = GetAngle() - AttackPath11[ATTACK_PATH_11_NUM - 1].direction;

		AngleLimit(&dis2FinalAngle);

		if (FetchDone())
		{

			OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);

			gRobot.archeryStart = 0;

			if (gRobot.walkMode == 1) //1号射箭区
			{
				gRobot.takeCnt++; //取道箭后加1

				gRobot.walkStatus = waitForForthPath;
				// gRobot.walkStatus = stop;
			}
			else if (gRobot.walkMode == 2) //2号射箭区
			{
				gRobot.takeCnt++;

				gRobot.walkStatus = waitForEighthPath;
				// gRobot.walkStatus = stop;
			}

			// gRobot.walkStatus = stop;
			return;
		}
		//跟随轨迹
		PathFollowing(0.4f); //0.6

		break;
	}

	//重新回到出发区重试区取箭
	case waitForTakeAgain:
	{
		OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);

		ClearRingBuffer();
		gRobot.pathPlanFlag = IS_PLANNING; //防止路径规划时间过长触发错误检测
		gRobot.archeryStart = 0;
		if (gRobot.TRRetryFlag == 3) //走回出发区
		{
			AttackPathTakeAgain[0].point.x = GetX();
			AttackPathTakeAgain[0].point.y = GetY();
			AttackPathTakeAgain[0].direction = GetAngle();
			if (GetX() < 800.f && GetY() > 11300.f) //在取箭处
			{
				//规划第三段轨迹
				AttackPathTakeAgain[1] = (Pose_t){762.50f, 10050.0f, -167.14f, 6000.f},
				AttackPathTakeAgain[2] = (Pose_t){725.00f, 8685.71f, -154.29f, 6000.f},
				AttackPathTakeAgain[3] = (Pose_t){687.50f, 7321.42f, -141.43f, 6000.f},
				AttackPathTakeAgain[4] = (Pose_t){650.00f, 5957.14f, -128.57f, 6000.f},
				AttackPathTakeAgain[5] = (Pose_t){612.50f, 4592.85f, -115.71f, 6000.f},
				AttackPathTakeAgain[6] = (Pose_t){575.00f, 3228.57f, -102.86f, 6000.f},
				AttackPathTakeAgain[7] = (Pose_t){537.50f, 1864.28f, -90.00f, 6000.f},
				AttackPathTakeAgain[8] = (Pose_t){500.00f, 500.f, -90.00f, 0.0f},
				ATTACK_PATH_TAKE_AGAIN_NUM = 9;
			}
			else if (GetX() > 7350.f) //在2号射箭区
			{
				AttackPathTakeAgain[1] = (Pose_t){GetX() + (687.50 - GetX()) * 1 / 3.f, GetY() + (7350.f - GetY()) * 1 / 3, GetAngle() + (-140.f - GetAngle()) * 1 / 3, 4000.f},
				AttackPathTakeAgain[2] = (Pose_t){GetX() + (687.50 - GetX()) * 2 / 3.f, GetY() + (7350.f - GetY()) * 2 / 3, GetAngle() + (-140.f - GetAngle()) * 2 / 3, 4000.f},
				AttackPathTakeAgain[3] = (Pose_t){687.50f, 7350.00f, -140.f, 6000.f},
				AttackPathTakeAgain[4] = (Pose_t){650.00f, 5980.00f, -128.57f, 6000.f},
				AttackPathTakeAgain[5] = (Pose_t){612.50f, 4610.00f, -115.71f, 6000.f},
				AttackPathTakeAgain[6] = (Pose_t){575.00f, 3240.00f, -102.86f, 6000.f},
				AttackPathTakeAgain[7] = (Pose_t){537.50f, 1870.00f, -90.00f, 6000.f},
				AttackPathTakeAgain[8] = (Pose_t){500.00f, 500.00f, -90.00f, 0.0f},
				ATTACK_PATH_TAKE_AGAIN_NUM = 9;
			}
			else //1号射箭区
			{
				AttackPathTakeAgain[1] = (Pose_t){GetX() + (500.f - GetX()) * 1 / 5.f, GetY() + (500.f - GetY()) * 1 / 5, -150.f, 6000.f},
				AttackPathTakeAgain[2] = (Pose_t){GetX() + (500.f - GetX()) * 2 / 5.f, GetY() + (500.f - GetY()) * 2 / 5, -120.f, 6000.f},
				AttackPathTakeAgain[3] = (Pose_t){GetX() + (500.f - GetX()) * 3 / 5.f, GetY() + (500.f - GetY()) * 3 / 5, -100.f, 6000.f},
				AttackPathTakeAgain[4] = (Pose_t){GetX() + (500.f - GetX()) * 4 / 5.f, GetY() + (500.f - GetY()) * 4 / 5, -90.f, 6000.f},
				AttackPathTakeAgain[5] = (Pose_t){500.00f, 500.00f, -90.00f, 0.0f},
				gRobot.archeryStart = 0;
				ATTACK_PATH_TAKE_AGAIN_NUM = 6;
			}
			InputPoints2RingBuffer(AttackPathTakeAgain, ATTACK_PATH_TAKE_AGAIN_NUM);
			gRobot.pathPlanFlag = NO_PLANNING; //防止路径规划时间过长触发错误检测
			ClearPathLen();
			gRobot.walkStatus = goForTakeAgain;
		}

		// gRobot.walkStatus = stop;
		break;
	}
	case goForTakeAgain:
	{
		float dis2FinalX = GetX() - AttackPathTakeAgain[ATTACK_PATH_TAKE_AGAIN_NUM - 1].point.x;
		float dis2FinalY = GetY() - AttackPathTakeAgain[ATTACK_PATH_TAKE_AGAIN_NUM - 1].point.y;
		float dis2FinalAngle = GetAngle() - AttackPathTakeAgain[ATTACK_PATH_TAKE_AGAIN_NUM - 1].direction;
		gRobot.archeryStart = 0;

		AngleLimit(&dis2FinalAngle);

		if (sqrtf(dis2FinalX * dis2FinalX + dis2FinalY * dis2FinalY) < 50.0f && JudgeSpeedLessEqual(100.0f) && fabs(dis2FinalAngle) <= 1.5f && fabs(GetWZ()) < 15.0f)
		{
			OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);
			// WheelLockTransform(-60.f, 0.f, 60.f);

			// gRobot.archeryStart = 1;

			//gRobot.walkStatus = waitForThirdPath;
			if (gRobot.TRRetryFlag == 0)
			{
				gRobot.tryTakeFlag = 1;
				if (gRobot.walkMode == 1) //1号射箭区
				{
					gRobot.walkStatus = waitForFirstPlace; //直接去射1
														   // gRobot.walkStatus = stop;
				}
				else if (gRobot.walkMode == 2) //2号射箭区
				{
					gRobot.walkStatus = waitForSecPlace; //直接去射2
														 // gRobot.walkStatus = stop;
				}
				else
				{
					gRobot.walkStatus = waitForStart;
				}
			}

			return;
		}
		//跟随轨迹
		PathFollowing(0.5f);

		break;
	}

	case waitForFirstPlace:
	{
		OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);

		ClearRingBuffer();
		gRobot.pathPlanFlag = IS_PLANNING; //防止路径规划时间过长触发错误检测

		//规划第三段轨迹
		AttackPathArcheryFirst[0].point.x = GetX();
		AttackPathArcheryFirst[0].point.y = GetY();
		AttackPathArcheryFirst[0].direction = GetAngle();
		InputPoints2RingBuffer(AttackPathArcheryFirst, ATTACK_PATH_ARCHERY_FIRST_NUM);

		OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);
		gRobot.archeryStart = 0;
		gRobot.pathPlanFlag = NO_PLANNING; //防止路径规划时间过长触发错误检测

		gRobot.walkStatus = goForFirstPlace;
		// gRobot.walkStatus = stop;
		break;
	}
	case goForFirstPlace:
	{
		float dis2FinalX = GetX() - AttackPathArcheryFirst[ATTACK_PATH_ARCHERY_FIRST_NUM - 1].point.x;
		float dis2FinalY = GetY() - AttackPathArcheryFirst[ATTACK_PATH_ARCHERY_FIRST_NUM - 1].point.y;
		float dis2FinalAngle = GetAngle() - AttackPathArcheryFirst[ATTACK_PATH_ARCHERY_FIRST_NUM - 1].direction;

		AngleLimit(&dis2FinalAngle);

		if (sqrtf(dis2FinalX * dis2FinalX + dis2FinalY * dis2FinalY) < 80.0f && JudgeSpeedLessEqual(100.0f) && fabs(dis2FinalAngle) <= 1.5f && fabs(GetWZ()) < 15.0f)
		{
			// OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			WheelLockTransform(-60.f, 0.f, 60.f);

			gRobot.archeryStart = 1;

			gRobot.walkStatus = waitForFifthPath;
			// gRobot.walkStatus = stop;

			return;
		}
		//跟随轨迹
		PathFollowing(0.5f);

		break;
	}

	case waitForSecPlace:
	{
		OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);

		ClearRingBuffer();
		gRobot.pathPlanFlag = IS_PLANNING; //防止路径规划时间过长触发错误检测

		//规划第三段轨迹
		AttackPathArcherySec[0].point.x = GetX();
		AttackPathArcherySec[0].point.y = GetY();
		AttackPathArcherySec[0].direction = GetAngle();
		InputPoints2RingBuffer(AttackPathArcherySec, ATTACK_PATH_ARCHERY_SEC_NUM);

		OutputVel2Wheel(0.0f, GetRingBufferPointAngle(1), 0.0f);
		gRobot.archeryStart = 0;
		gRobot.pathPlanFlag = NO_PLANNING; //防止路径规划时间过长触发错误检测

		gRobot.walkStatus = goForSecPlace;
		// gRobot.walkStatus = stop;
		break;
	}
	case goForSecPlace:
	{
		float dis2FinalX = GetX() - AttackPathArcherySec[ATTACK_PATH_ARCHERY_SEC_NUM - 1].point.x;
		float dis2FinalY = GetY() - AttackPathArcherySec[ATTACK_PATH_ARCHERY_SEC_NUM - 1].point.y;
		float dis2FinalAngle = GetAngle() - AttackPathArcherySec[ATTACK_PATH_ARCHERY_SEC_NUM - 1].direction;

		AngleLimit(&dis2FinalAngle);

		if (sqrtf(dis2FinalX * dis2FinalX + dis2FinalY * dis2FinalY) < 80.0f && JudgeSpeedLessEqual(100.0f) && fabs(dis2FinalAngle) <= 1.5f && fabs(GetWZ()) < 15.0f)
		{
			// OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			WheelLockTransform(-60.f, 0.f, 60.f);

			gRobot.archeryStart = 1;

			gRobot.walkStatus = waitForArchery;
			// gRobot.walkStatus = stop;

			return;
		}
		//跟随轨迹
		PathFollowing(0.2f);

		break;
	}

	case testPara:
	{
		TestParaMotor();
		break;
	}

	//停止状态
	case stop:
	{
		// WheelLockTransform(60.f, 0.f, -60.f);
		//  OutputVel2Wheel(0.0f,180.0f,0.0f);
		WheelLockTransform(-60, 0, 60);
		gRobot.archeryStart = 1;
		//SendCmd2Driver(0,gRobot.wheelState.oneTarget.direction,0,gRobot.wheelState.twoTarget.direction,0,gRobot.wheelState.thrTarget.direction);
		if(gRobot.TRRetryFlag == 1 && gRobot.ppsHeart < 20 && gRobot.ppsSameHeart < 20)
		{
			gRobot.walkStatus = waitForTakeAgain;
		}
		break;
	}

	case waitForArchery:
	{

		ClearRingBuffer();
		// OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
		WheelLockTransform(-60.f, 0.f, 60.f);
		gRobot.archeryStart = 1;

		LidarCorrect();

		if (gRobot.walkMode == 1) //1号射箭区(2到1)
		{
			gRobot.walkStatus = waitForTenthPath;
			// gRobot.walkStatus = stop;
		}
		else if ((gRobot.shootArrowCnt-gRobot.initialCnt)%5 == 0) //需改
		{
			gRobot.walkStatus = waitForEleventhPath; //改成从2号射箭区去取箭
		}

		// gRobot.walkStatus = stop;
		break;
	}

	default:
		break;
	}
}

/*  ***************************************测试加速度程序*********************************************************** */
void TestParaMotor(void)
{
#define VEL (3000.0f)
#define DIRECTION (0.0f)

	static uint8_t testStatus = 0;

	delayCount++;

	delayCount %= 10000;

	switch (testStatus)
	{
	case 0:
	{
		SendCmd2Driver(1000.0f, -60.0f, 1000.0f, -60.0f, 1000.0f, -60.0f);
		// SetTargetVel(500.0f,180.0f,0.0f);

		if (delayCount > 50)
		{
			delayCount = 0;
			testStatus = 1;
		}

		break;
	}
	case 1:
	{
		SendCmd2Driver(0.0f, -60.0f, 0.0f, -60.0f, 0.0f, -60.0f);
		// SetTargetVel(0.0f,180.0f,0.0f);

		break;
	}
	default:
		break;
	}
}

/*  ********************************************Other Functions***************************************************** */
uint8_t JudgeSpeedLessEqual(float speedCompared)
{
	velVector_t actSpeed = GetSpeedWithoutOmega();
	float speedX = actSpeed.xVel;
	float speedY = actSpeed.yVel;

	float speed = sqrtf(speedX * speedX + speedY * speedY);

	if (speed > speedCompared)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

//射箭区雷达矫正（为取箭做矫正）
void LidarCorrect(void)
{
	if ((gRobot.shootArrowCnt - 3) % 5 == 1) //射完所有的箭
	{
		gRobot.DRRetryFlag = 3;
		writePpsData[0] = 'S';
		writePpsData[1] = 'T';
		writePpsData[MC_LIDAR_BUF_LENGTH - 3] = gRobot.DRRetryFlag;
		fprintf(fpWrite, "retryflag  %d ", (int)writePpsData[2]);
		writePpsData[MC_LIDAR_BUF_LENGTH - 2] = '\r';
		writePpsData[MC_LIDAR_BUF_LENGTH - 1] = '\n';
		ProcessCommWrite(writePpsData, MC_LIDAR_BUF_LENGTH);
	}
}

int FetchDone(void)
{
	static int doneStep = 0;
	switch (doneStep)
	{
	case 0:
	{
		if (GetX() < 640.0f && GetY() > 11300.0f)
		{
			doneStep = 1;
		}
		break;
	}
	case 1:
	{
		if (JudgeSpeedLessEqual(420.0f))
		{
			doneStep = 0;
			return 1;
		}
	}
	}
	return 0;
}
