#ifndef __ROBOT_H
#define __ROBOT_H

#include <stdint.h>
#include "calculate.h"
#include "moveBase.h"


//模式选择
#define OPERATING_MODE				NORMAL_MODE

//红场
#define RED_COURT (0)
//蓝场
#define BLUE_COURT (1)
/*机器人模式*/
//复位模式
#define RESET_MODE (0)
//走形模式
#define WALK_MODE (1)
/*走形模式*/
#define ATTACK_MODE (1)//进攻
//防御模式
#define DEFENCE_MODE (2)//防御
//自检模式
#define SELF_CHECK_MODE (4)
//调参模式
#define DEBUG_PARA_MODE (5)
//重试模式
#define RETRY_MODE (6)
//测试加速度
#define ACC_TEST				(2)
#define NO_PLANNING (0)   
#define IS_PLANNING (1)
//调试数据结构体
typedef struct
{
	//PathFollowing轨迹跟随变量
	float VIEW_L;
	float robotlenVP;
	float robotlenVT;
	PointU_t virtualPos;
	PointU_t virtualTarget;
	float disRealPos2VirPos;
	float disRealPos2VirTarget;
	float disAdd;
	float posAngleVP;
	float posAngleVT;
	float omega;
	float originVel;
	float originVelDir;
	float fixedVel;
	robotVel_t adjustVel;
	float sumVel;
	float sumVelDir;
	float distance;
	
	//VelControl速度环变量
	float velXErr;
	float velYErr;
	float velXErrL;
	float velYErrL;
	float outputVel;
	float outputDirection;
	float outputOmega;
	

}debugInfo_t;
//走行状态变量枚举类型变量
typedef enum
{
	waitForStart,	//0
	goForFirstPath,//1
	waitForSecondPath,//2
	goForSecondPath,//3
	waitForThirdPath,//4
	goForThirdPath,//5
	waitForForthPath,//6
	goForForthPath,//7
	waitForFifthPath,//8
	goForFifthPath,//9
	waitForSixthPath,//10
	goForSixthPath,//11
	waitForSeventhPath,//12
	goForSeventhPath,//13
	waitForEighthPath,//14
	goForEighthPath,//15
	waitForNinthPath,//16
	goForNinthPath,//17
	waitForTenthPath,//18
	goForTenthPath,//19
	waitForEleventhPath,//20
	goForEleventhPath,//21
	goForPot2ADefence,//22
    goForPot2BDefence,//23
    goForPot1ADefence,//24
    goForPot3Defence,//25
	goBackForPot2ADefece,//26
	goBackForPot3Defece,//27
	waitForPushPot,//28
	goToPushPot,//29
	goAttackPath1,//30
	goAttackPath2,//31
	waitForArchery,//32
	startArchery, //开始防守33
	waitFordefenceRetry,//34
	waitForTakeArrow,//35
	goForTakeArrow,//36

	//重试取箭（再现规划到出发区）
	waitForTakeAgain,//37
	goForTakeAgain,//38
	waitForFirstPlace,
	goForFirstPlace,
	waitForSecPlace,
	goForSecPlace,

	testPara,
	stop
}walkStatus_t;
//全局变量结构体
typedef struct
{
	//轮子状态
	wheelState_t wheelState;
	//调试数据
	debugInfo_t debugInfomation;
	//走行状态
	walkStatus_t walkStatus;
	//mcu通信心跳包
	int mcuHeart;
	//pps心跳包
	int ppsHeart;
	int ppsSameHeart;
	//红蓝场
	uint8_t courdID;
	//轨迹规划ing 标志位
	uint8_t pathPlanFlag;
	//机器人模式
	uint8_t robotMode;
	//走形模式
	uint8_t walkMode;
	//防守的桶号
	uint8_t defencePotID;
	//进攻桶号
	uint8_t attackPotID;
	//开场红蓝场信息
	uint8_t colorFlag;
	//射箭命令
	uint8_t archeryStart;
	//射完箭命令
	uint8_t archeryDone;
	//挥箭命令
	uint8_t waveArchery;
	//MCU's angle
	int cvMcuDir;
	//视觉传过来的偏离角度
	float cvDir;
	//视觉传过来的距离
    float cvDis;
	//转盘角度
	float turnAngle;
	//CV state
	uint8_t CVstate;

	uint8_t fetchReady;
	//已射箭的个数
	uint8_t shootArrowCnt; 
	//重试标志位
	uint8_t TRRetryFlag;
	//重试取箭标志
	uint8_t writeFlag;
	//重试标志位
	uint8_t DRRetryFlag;
	//LiDar返回信号，2是他收到了；3是他矫正完了
	uint8_t DRRetryReturn;
	//取箭次数
	uint8_t takeCnt;
	//在出发区重试取箭
	uint8_t tryTakeFlag;
	//给视觉的桶号标志位(射箭时再赋值)
	uint8_t cvAttackPotID;

	//新增自动射箭数据
	float autoShootVel;
	float voltageTemp;
	float autoShootKp;
	float autoShootKi;
	float autoPitchAngle;
	int mcuReInitCnt;

	//initial shotCnt
	uint8_t initialCnt;


}gRobot_t;

extern gRobot_t gRobot;
int GetTimeCounter(void);

void CountTime(void);

void SetCountTimeFlag(void);

void Walk(void);

uint8_t JudgeSpeedLessEqual(float speedCompared);

void VelControl(robotVel_t actVel);

//5号桶切换到1号桶
void DefencePot5toPot1(void);

//5号桶切换到2号桶
void DefencePot5toPot2(void);

//1号桶切换到2号桶
void DefencePot1toPot2(void);

//2号桶切换到1号桶
void DefencePot2toPot1(void);

//1号桶切换到5号桶
void DefencePot1toPot5(void);

//测试加速度程序
void TestParaMotor(void);

//射箭区雷达矫正（为取箭做矫正）
void LidarCorrect(void);

int FetchDone(void);

#endif
