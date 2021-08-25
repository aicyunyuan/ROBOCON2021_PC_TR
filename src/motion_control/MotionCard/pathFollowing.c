/********************************************************************
*Copyright(C）2014-2016,沈阳艾克申机器人技术开发有限责任公司
*FileName：	   
*Author：      
*Date：       
*Description： 
*
*Version：     V1.0
********************************************************************/
#include <math.h>
#include "pathFollowing.h"
#include "ringbuffer.h"
#include "Bspline.h"
#include "posSystem.h"
#include "MotionCard.h"
#include "Move.h"
#include "stdint.h"
#include "robot.h"
#include "SpeedPlaning.h"
#include "pps.h"
#include "path.h"

#define EXPERT_PID
PathfollowPara_t goForFirstPara = 
{
	{2.9,0,0,2.0},//0.15
	{3.8,0,0,2.9},//0.2
	{4.0,4.5,0,3.25},//0.25
	{4.5,0,0,0},//3.0
	{4.8,5.2,0,0},//3.5
	{5.3,5.9,2.4,3.9},//4
	{0,0,0,0},
	{0,0,0,0},
	{0,0,0,0},
	{0,0,0,0}
};
/*********************************************************************************
* @name 	PathFollowingNew
* @brief	路径跟随函数
* @param	percent 速度的百分比，若为1代表100%所规划的速度运行。范围为大于0,如果超过1，会超过机器人承受速度
* @retval	无
**********************************************************************************/
//调节量：按当前点到虚拟位置点作为偏差 提前30m到终点不调
//降速的量：TR第一段直线：垂直于速度方向的误差作为偏移量降速
//提前量：TR第一段直线:会减去 垂直于速度方向的误差 这个偏移量
int PathFollowing(float percent)
{
	static float vell = 150.0f;
	float velDir = 0.0f;
	float angularVel = 0.0f;
	float angleErr = 0.0f;
	float posAngleVP = 0.0f;
	float posAngleVT = 0.0f;
	float robotlen = 0.0f;
	float disRealPos2VirTarget = 0.0f;
	float disRealPos2VirPos = 0.0f;
	robotVel_t adjustVel = {0.0f};
	PointU_t virtualPos,virtualTarget;

	if(percent < 0.0f || percent > 1.2f)
	{
		printf("Invalid parameter\n");
		return -1;
	}

	//当前点与虚拟位置点距离 和 虚拟位置点到虚拟目标点距离之和
	float VIEW_L = 0.0f;
	//提前量
	switch(gRobot.walkStatus)
	{
		case goForFirstPath:
		{
			VIEW_L = GetPosPresent().vel/50.0f*5.4f;//5.0
			// if(GetX()>1214.0f && GetY()>10719.0f)//走圆弧，增大提前量
			// {
			// 	VIEW_L = GetPosPresent().vel/50.0f*5.9f;
			// }
			break;
		}
		case goForSecondPath:
		{
			VIEW_L = GetPosPresent().vel/50.0f*2.5f;//5.0
			break;
		}
		case goForThirdPath:
		{
			VIEW_L = GetPosPresent().vel/50.0f*2.8f;
			break;
		}
		case goForForthPath:
		{
			VIEW_L = GetPosPresent().vel/50.0f*6.9f;
			break;
		}
		case goForFifthPath:
		{
			VIEW_L = GetPosPresent().vel/50.0f*6.7f;
			break;
		}
		case goForEighthPath:
		{
			VIEW_L = GetPosPresent().vel/50.0f*4.8f;//5.8
			// if(GetY() < 3700.0f || GetY() > 10200.0f)
			// {
			// 	VIEW_L = GetPosPresent().vel/50.0f*1.8f;
			// }
			break;
		}
		case goForNinthPath:
		{
			VIEW_L = GetPosPresent().vel/50.0f*3.0f;
			break;
		}
		case goForTenthPath:
		{
			VIEW_L = GetPosPresent().vel/50.0f*4.5f;
			break;
		}
		case goForEleventhPath:
		{
			VIEW_L = GetPosPresent().vel/50.0f*3.8f;//4.7
			break;
		}
		case goForTakeAgain:
		{
			VIEW_L = GetPosPresent().vel/50.0f*4.8f;//4.85//4.7
			break;
		}
		case goForFirstPlace:
		{
			VIEW_L = GetPosPresent().vel/50.0f*4.8f;//4.85//4.7
			break;
		}
		default:
		{
			VIEW_L = GetPosPresent().vel/50.0f*2.0f;
			break;
		}
	}

	//获取定位系统所计算的机器人实际行走路径长度
	robotlen = GetPath();
	
	gRobot.debugInfomation.robotlenVP = robotlen;

	//虚拟位置点
	virtualPos = SerchVirtualPoint(robotlen);
	
	//计算当前点到虚拟位置点的距离(直线距离)	
	disRealPos2VirPos = CalculatePoint2PointDistance(GetPosPresent().point,virtualPos.point);

	if(VIEW_L - disRealPos2VirPos >= 0.0f)
	{
		//加上 提前量与偏移量的差值
		robotlen = GetPath() + VIEW_L - disRealPos2VirPos;
	}
	
	gRobot.debugInfomation.robotlenVT = robotlen;
	
	//求取虚拟目标点
	virtualTarget = SerchVirtualPoint2(robotlen);
	
	//计算实际位置距离虚拟目标点距离
	disRealPos2VirTarget = CalculatePoint2PointDistance(GetPosPresent().point,virtualTarget.point);

	float disAdd = (VIEW_L - disRealPos2VirPos) - disRealPos2VirTarget;
	
	if(GetPath() < GetLength())
	{
		if(disAdd > 0)
		{
			AddPath(2*disAdd);
		}
	}
	else
	{
		//当记录的路程大于轨迹总路程后停止记录路程
		UpdateLenStop();
	}
	
	//虚拟位置点姿态角
	posAngleVP = CalculateAngleAdd(GetRingBufferPointPoseAngle(1),CalculateAngleSub(GetRingBufferPointPoseAngle(2) , GetRingBufferPointPoseAngle(1))*virtualPos.u);
	
	//两端点之间角度的插值
	angleErr = CalculateAngleSub(GetRingBufferPointPoseAngle(virtualTarget.endPtr) , GetRingBufferPointPoseAngle(virtualTarget.startPtr));
	
	//虚拟目标点姿态角
	posAngleVT = CalculateAngleAdd(GetRingBufferPointPoseAngle(virtualTarget.startPtr),angleErr*virtualTarget.u);
	//角速度
	switch(gRobot.walkStatus)
	{
		case goForFirstPath:
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,3.2f,2.9f);

			// if(GetX() < 1200.0f && GetY() > 11350.0f && GetX() >= 748.0f)
			// {
			// 	angularVel = AngleControl(GetPosPresent().direction,posAngleVT,1.5f,1.3f);
			// }	
			// else if(GetX() < 748.0f && GetY() > 11350.0f)
			// {
			// 	angularVel = AngleControl(GetPosPresent().direction,posAngleVT,2.1f,2.0f);
			// }
			break;
		}
		case goForSecondPath:
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,2.4f,2.3f);
			break;
		}
		case goForTakeArrow:
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,1.5f,1.4f);
			break;
		}
		case goForThirdPath:
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,2.05f,2.0f);
			break;
		}
		case goForForthPath:
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,2.7f,2.56f);
			break;
		}
		case goForFifthPath:
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,2.8f,2.5f);
			break;
		}
		case goForEighthPath:
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,3.3f,2.8f);
			break;
		}
		case goForNinthPath:
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,2.5f,2.2f);
			break;
		}
		case goForTakeAgain:
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,3.0f,2.7f);
			break;
		}
		case goForEleventhPath:
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,3.5f,3.2f);
			break;
		}
		default:
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,2.35f,2.15f);
			break;
		}
	}
		
	//目标速度方向
	velDir = virtualTarget.direction;
	
	AngleLimit(&velDir);
	
	//目标速度
	vell = GetRingBufferPointVell(virtualTarget.startPtr)+(GetRingBufferPointVell(virtualTarget.endPtr) - GetRingBufferPointVell(virtualTarget.startPtr))*virtualTarget.u;
	
	vell = vell*percent;
	
	gRobot.debugInfomation.originVel = vell;
	gRobot.debugInfomation.originVelDir = velDir;
	
	//fix me 直线段应只将垂直于轨迹方向的分量作为偏移量
	//偏移量过大修正速度(适当降速)
	// if(gRobot.walkStatus == goForFirstPath)
	// {
	// 	if(disRealPos2VirPos>50.0f && disRealPos2VirPos < 120.0f)
	// 	{
	// 		if(vell*vell > 2.0f*(0.5f*GetAccLimit())*(disRealPos2VirPos - 50.0f))
	// 		{			
	// 			vell = vell*(1-0.011f*(disRealPos2VirPos - 50.f));
	// 			printf("/ncutspeed1/n");
	// 		}	
	// 		else 
	// 		{
	// 			printf("/ncutspeed2/n");
	// 			vell = 0.62f*vell;
	// 		}
	// 	}
	// 	// else if (disRealPos2VirPos < 50.0f)
	// 	// {
	// 	// 	vell = vell * (0.00016f * disRealPos2VirPos * disRealPos2VirPos - 0.008f * disRealPos2VirPos + 1.0f);		
	// 	// }
	// 	else if(disRealPos2VirPos >= 120.0f)
	// 	{
	// 		if(vell*vell > 2.0f*(0.5f*GetAccLimit())*(disRealPos2VirPos - 130.0f))
	// 		{		
	// 			vell = 0.6f*vell;
	// 			printf("/ncutspeed3/n");
	// 		}
	// 		else
	// 		{
	// 			printf("/ncutspeed4/n");
	// 			vell = 0.75f*vell;
	// 		}
	// 	}
	// }
	// else
	// {
	// 	if(disRealPos2VirPos>50.0f && disRealPos2VirPos < 120.0f)
	// 	{
	// 		if(vell*vell > 2.0f*(0.5f*GetAccLimit())*(disRealPos2VirPos - 50.0f))
	// 		{			
	// 			vell = vell*(1-0.006f*(disRealPos2VirPos - 50.f));
	// 		}
	// 		else
	// 		{
	// 			vell = 0.75f*vell;
	// 		}
	// 	}
	// 	else if(disRealPos2VirPos >= 130.0f)
	// 	{
	// 		if(vell*vell > 2.0f*(0.5f*GetAccLimit())*(disRealPos2VirPos - 130.0f))
	// 		{		
	// 			vell = 0.75f*vell;
	// 		}
	// 		else
	// 		{
	// 			vell = 0.85f*vell;
	// 		}
	// 	}
	// }
	
	

	gRobot.debugInfomation.fixedVel = vell;
	
//	static float lastVell = 0.0f;

//	if(GetPath() < 15.f)
//	{
//		if(vell < lastVell)
//		{
//			vell = lastVell;
//		}
//	}
//	
//	lastVell = vell;
	//计算当前点到目标点的位置调节量
	adjustVel = GetAdjustVel(GetPosPresent().point,virtualPos,vell);

	AdjustVel(&vell,&velDir,adjustVel);

	SetTargetVel(vell,velDir,angularVel);
		
	gRobot.debugInfomation.VIEW_L = VIEW_L;								//提前量
	gRobot.debugInfomation.virtualPos = virtualPos;						//虚拟位置点
	gRobot.debugInfomation.posAngleVP = posAngleVP;						//虚拟位置点姿态角
	gRobot.debugInfomation.virtualTarget = virtualTarget;				//虚拟目标点
	gRobot.debugInfomation.posAngleVT = posAngleVT;						//虚拟目标点姿态角
	gRobot.debugInfomation.disRealPos2VirPos = disRealPos2VirPos;		//点到虚拟位置点距离
	gRobot.debugInfomation.disRealPos2VirTarget = disRealPos2VirTarget;	//点到虚拟目标点距离
	gRobot.debugInfomation.disAdd = disAdd;								//修正距离
	gRobot.debugInfomation.adjustVel = adjustVel;						//调节速度
	gRobot.debugInfomation.sumVel = vell;								//目标速度
	gRobot.debugInfomation.sumVelDir = velDir;							//目标速度方向
	gRobot.debugInfomation.omega = angularVel;							//目标角速度
	
	return 1;
}




/*********************************************************************************
* @name 	AngleControl
* @brief	角度闭环控制程序
* @param	anglePresent 当前的角度 单位 度
* @param  angleTarget  目标角度   单位 度
* @retval	无
**********************************************************************************/
float AngleControl(float anglePresent,float angleTarget,float kp,float kd)
{
	/****************************普通PD控制*******************************/
	
	float angleErr = 0.0f,angularVel = 0.0f, angularVelErr = 0.0f;
	static float lastAngleErr = 0.0f, preAngleErr = 0.0f, lastAngledTerm = 0.0f;
	float dTerm = 0.0f,dTermFliter = 0.0f;
	//PD控制器
	//目标角度减去当前角度
	angleErr = CalculateAngleSub(angleTarget,anglePresent);
	dTerm = (angleErr - lastAngleErr);
	//低通滤波
	dTermFliter = 0.5f*dTerm + 0.5f*lastAngledTerm;

	angularVel = angleErr * kp + dTermFliter * kd;
	
	lastAngledTerm = dTerm;
	lastAngleErr = angleErr;
	preAngleErr = lastAngleErr;
	gRobot.debugInfomation.omega = angularVel;

	
	/******************************************************************/

	/********************************专家PID****************************
	 *    5条规则
	 * 1. 误差绝对值很大，按照最大（绝对值）输出
	 * 2. 误差×delta误差>0，误差增大，如果误差值大于中值，则加大控制力度，否则一般控制
	 * 3. 误差×delta误差<0 && delta误差×delta（上一次误差）>0，误差减小 ，一般控制
	 * 4. 误差×delta误差<0 && delta误差×delta（上一次误差）<0，处于极值 ，一般控制（保持输出不变即可）
	 * 5. 误差很小，一般控制（可能是系统静差）
	 * /
	/*定义结构体和公用体*/
	// typedef struct
	// {
	// 	float setpoint;   	  	/*设定值*/
	// 	float kp;             	/*比例系数*/
	// 	float ki;          		/*积分系数*/
	// 	float kd;             	/*微分系数*/
	// 	float thisErr;          /*偏差*/
	// 	float lastErr;        	/*前一拍偏差*/
	// 	float preErr;           /*前两拍偏差*/
	// 	float dErr;             /*偏差差值（微分）*/ 
	// 	float lastdErr;         /*上一次偏差差值*/ 
	// 	float result;           /*PID控制器结果*/
	// 	float maximum;          /*输出值上限*/
	// 	float minimum;          /*输出值下限*/
	// 	float errabsmax;        /*偏差绝对值最大值*/
	// 	float errabsmid;        /*偏差绝对值中位值*/
	// 	float errabsmin;        /*偏差绝对值最小值*/
	// }ExpertPID_t;

	// static ExpertPID_t anglePID;
  	// float angularVel = 0.0f, angularVelErr = 0.0f;//本次调节输出值
	
	// anglePID.maximum = 200.f;//最大输出 待修正
	// anglePID.minimum = -200.f;//待修正
	// anglePID.errabsmax = 40.f;//角度最大偏差阈值 40° 
	// anglePID.errabsmid = 25.f;//角度中等偏差阈值 25° 
	// anglePID.errabsmin =  1.f;//角度最小偏差阈值  1° 

	// anglePID.kp = kp;
	// anglePID.kd = kd;
	// anglePID.thisErr = CalculateAngleSub(angleTarget,anglePresent);//本次偏差
	// anglePID.dErr = anglePID.thisErr - anglePID.lastErr;//偏差微分

	// if(fabs(anglePID.thisErr) > anglePID.errabsmax)//误差太大，执行规则1
	// {
	// 	anglePID.result = fabs(anglePID.thisErr) / anglePID.thisErr * anglePID.maximum;//最大输出
	// }	
	// else if((anglePID.thisErr * anglePID.dErr > 0)||(fabs(anglePID.dErr) < 0.1f))//误差增大，执行规则2
	// {
	// 	if(fabs(anglePID.thisErr) >= anglePID.errabsmid)//误差较大,加大控制
	// 	{
	// 		anglePID.result = 2.0*(anglePID.thisErr * anglePID.kp + anglePID.dErr * anglePID.kd);
	// 	}
	// 	else//误差较小，一般控制
	// 	{
	// 		anglePID.result = 1.5*(anglePID.thisErr * anglePID.kp + anglePID.dErr * anglePID.kd);
	// 	}
	// }
 	// else if(((anglePID.thisErr * anglePID.dErr < 0) && (anglePID.dErr * anglePID.lastdErr > 0)) || (anglePID.thisErr == 0))//误差减小，执行规则3，一般控制
  	// {
    // 	anglePID.result = 1.0*(anglePID.thisErr * anglePID.kp + anglePID.dErr * anglePID.kd);
  	// }
	// else if((anglePID.thisErr * anglePID.dErr < 0) && (anglePID.dErr * anglePID.lastdErr < 0))//误差极值，执行规则4
  	// {
    // 	if(abs(anglePID.thisErr) >= anglePID.errabsmid) //误差较大，则较强控制
	// 	{
	// 		anglePID.result = 2.0 * anglePID.kp * anglePID.thisErr;
	// 	}
	// 	else//误差一般大，一般控制
	// 	{
	// 		anglePID.result = 1.0 * anglePID.kp * anglePID.thisErr;
	// 	}
  	// }
	// else if((fabs(anglePID.thisErr) <= anglePID.errabsmin) && (abs(anglePID.thisErr)>0))//误差很小，执行规则5
	// {
	// 	anglePID.result = 0.8 * anglePID.kp * anglePID.dErr + 0.8 * anglePID.ki * anglePID.thisErr;//可能存在稳态误差，添加ki控制
	// }
	// anglePID.preErr = anglePID.lastErr;
	// anglePID.lastErr = anglePID.thisErr;
	// anglePID.lastdErr = anglePID.dErr;

	// angularVel = anglePID.result;

	// printf("Ae %d %d %d %d ",(int)anglePID.thisErr, (int)anglePID.dErr,\
	// 	(int)anglePID.lastdErr, (int)anglePID.result);
	/********************************专家PID***************************/
	angularVelErr = angularVel - GetWZ();

	gRobot.debugInfomation.omega = angularVel;
	
	if(gRobot.walkStatus == goForFirstPath) //增加角速度闭环
	{
		angularVel = angularVel + angularVelErr *0.2f;
	}
	else if(gRobot.walkStatus == goForSecondPath) //增加角速度闭环
	{
		angularVel = angularVel + angularVelErr *0.2f;
	}
	else if(gRobot.walkStatus == goForThirdPath) //增加角速度闭环
	{
		angularVel = angularVel + angularVelErr *0.2f;
	}
	else if(gRobot.walkStatus == goForForthPath) //增加角速度闭环
	{
		angularVel = angularVel + angularVelErr *0.2f;
	}
	else
	{
		angularVel = angularVel + angularVelErr *0.0f;
	}
	//限幅
	if(angularVel>240.0f)
	{
		angularVel = 240.0f;
	}
	else if(angularVel<-240.0f)
	{
		angularVel = -240.0f;
	}

	if(sqrt(GetSpeedX()*GetSpeedX()+GetSpeedY()*GetSpeedY()) < 250.0f)
	{
		if(angularVel>30.0f)
		{
			angularVel = 30.0f;
		}
		else if(angularVel<-30.0f)
		{
			angularVel = -30.0f;
		}
	}

	gRobot.debugInfomation.outputOmega = angularVel;
		
	return angularVel;
}

/*********************************************************************************
* @name 	PostionControl
* @brief	位置闭环控制程序
* @param	distance 当前的位置误差 单位 mm
* @param  	kp 专家PID参数
* @param  	kd 专家PID参数

* @retval	无
**********************************************************************************/
float PostionControl(float distance, float kp, float kd)
{
	/********************************专家PID****************************
	 *    5条规则
	 * 1. 误差绝对值很大，按照最大（绝对值）输出
	 * 2. 误差×delta误差>0，误差增大，如果误差值大于中值，则加大控制力度，否则一般控制
	 * 3. 误差×delta误差<0 && delta误差×delta（上一次误差）>0，误差减小 ，一般控制
	 * 4. 误差×delta误差<0 && delta误差×delta（上一次误差）<0，处于极值 ，一般控制（保持输出不变即可）
	 * 5. 误差很小，一般控制（可能是系统静差）
	 */

	static ExpertPID_t distancePID;
  	float adjustVelOutput = 0.0f, adjustVelOutputErr = 0.0f;//本次调节输出值
	
	distancePID.maximum = 1500.f;//最大输出速度 待修正
	distancePID.minimum = 0.f;//待修正
	distancePID.errabsmax = 400.f;// 最大距离误差 
	distancePID.errabsmid = 225.f;//中等距离误差 
	distancePID.errabsmin =  46.f;//小距离误差   

	distancePID.kp = kp;
	distancePID.kd = kd;
	distancePID.thisErr = distance;//本次偏差
	distancePID.dErr = distancePID.thisErr - distancePID.lastErr;//偏差微分

	if(fabs(distancePID.thisErr) > distancePID.errabsmax)//误差太大，执行规则1
	{
		// printf("11111\r\n");
		distancePID.result = fabs(distancePID.thisErr) / distancePID.thisErr * distancePID.maximum;//最大输出
	}	
	else if((distancePID.thisErr * distancePID.dErr > 0)||(fabs(distancePID.dErr) < 0.1f))//误差增大，执行规则2
	{
		// printf("222222\r\n");
		if(fabs(distancePID.thisErr) >= distancePID.errabsmid)//误差较大,加大控制
		{
			distancePID.result = 1.0*(distancePID.thisErr * distancePID.kp + distancePID.dErr * distancePID.kd);
		}
		else//误差较小，一般控制
		{
			distancePID.result = 1.0*(distancePID.thisErr * distancePID.kp + distancePID.dErr * distancePID.kd);
		}
	}
 	else if(((distancePID.thisErr * distancePID.dErr < 0) && (distancePID.dErr * distancePID.lastdErr > 0)) || (distancePID.thisErr == 0))//误差减小，执行规则3，一般控制
  	{
		// printf("333333\r\n");
    	distancePID.result = 1.0*(distancePID.thisErr * distancePID.kp + distancePID.dErr * distancePID.kd);
  	}
	else if((distancePID.thisErr * distancePID.dErr < 0) && (distancePID.dErr * distancePID.lastdErr < 0))//误差极值，执行规则4
  	{
		// printf("444444\r\n");
    	if(abs(distancePID.thisErr) >= distancePID.errabsmid) //误差较大，则较强控制
		{
			distancePID.result = 1.0* distancePID.kp * distancePID.thisErr;
		}
		else//误差一般大，一般控制
		{
			distancePID.result = 1.0 * distancePID.kp * distancePID.thisErr;
		}
  	}
	else if((fabs(distancePID.thisErr) <= distancePID.errabsmin) && (abs(distancePID.thisErr)>0))//误差很小，执行规则5
	{
		// printf("555555\r\n");
		distancePID.result = 1.0 * distancePID.kp * distancePID.dErr + 0.8 * distancePID.ki * distancePID.thisErr;//可能存在稳态误差，添加ki控制
	}
	distancePID.preErr = distancePID.lastErr;
	distancePID.lastErr = distancePID.thisErr;
	distancePID.lastdErr = distancePID.dErr;

	return distancePID.result;

	printf("De %d %d %d %d ",(int)distancePID.thisErr, (int)distancePID.dErr,\
		(int)distancePID.lastdErr, (int)distancePID.result);
	/********************************专家PID***************************/
}

robotVel_t GetAdjustVel(Point_t robotPos,PointU_t adjustTarget,float vell)
{
	#define MAX_ADJUST_VEL (770.0f)
	robotVel_t adjustVel = {0};
	float distance = 0.0f;
	float angle = 0.0f;

	angle = CalculateLineAngle(robotPos,adjustTarget.point);
	
	distance = CalculatePoint2PointDistance(robotPos,adjustTarget.point);
	
	//死区
	// if(distance<5.0f)
	// {
	// 	distance = 0.0f;
	// }
	// else
	// {
	// 	distance-=5.0f;
	// }
	
	if(distance<=0.0f)
	{
		distance = 0.0f;
	}
	gRobot.debugInfomation.distance = distance;
	//计算调节速度大小和方向

	if(distance <= 20.0f)
	{
		distance = distance * 1.5f;
//		adjustVel.vel = distance * 7.5f;
	}
	else if(distance <= 40.0f) 
	{
		distance = distance * 2.8f - 26.0f;//2.5-20
	}
	else if(distance <= 60.0f)
	{
		distance = distance * 2.2f - 2.0f;//3.3  -52
	}
	else if(distance <= 100.0f)
	{
		distance = distance * 2.5 - 20.0f;//4.0  -94//4.5 -124//6.0 -214
	}
	else if(distance <= 200.0f)
	{
		distance = distance * 3.0f -70.0f;//4.5  -144//4.5 -124
//		adjustVel.vel = distance * 4.5f + 120.0f;
	}
	else
	{
		distance = distance * 3.0f;
//		adjustVel.vel = distance * 4.0f + 320.0f;
	}

	switch(gRobot.walkStatus)
	{
		case goForFirstPath:
		{
			adjustVel.vel = distance * 2.85f;//2.85
			if(GetY() > 9000.0f && GetY() < 11350.0f)
			{
				adjustVel.vel = distance * 2.4f;//2.4
			}
			else if(GetY() > 11350.0f)
			{
				adjustVel.vel = distance * 2.45f;//2.45
			}
			if(GetX() < 850 && GetY() > 11000.0f && fabs(vell) < 100.0f)
			{
				adjustVel.vel = 0;
			}
			break;
		}
		case goForSecondPath:
		{
			adjustVel.vel = distance * 1.8f;//2.3
			if(distance > 100)
			{
				adjustVel.vel = distance * 1.5f;//1.5
			}
			break;
		}
		case goForThirdPath:
		{
			adjustVel.vel = distance * 2.2f;
			if(GetX() < 850 && GetY() > 11000.0f && fabs(vell) < 280.0f)
			{
				adjustVel.vel = 0;
			}
			break;
		}
		case goForForthPath:
		{
			adjustVel.vel = distance * 3.2;
			if(GetY() < 10050.f)
			{
				adjustVel.vel = distance * 1.3;
			}
			break;
		}
		case goForFifthPath:
		{
			adjustVel.vel = distance * 3.15f;
			if(GetX() < 850 && GetY() > 11000.0f && fabs(vell) < 280.0f)
			{
				adjustVel.vel = 0;
			}
			break;
		}
		case goForSixthPath:
		{
			adjustVel.vel = distance * 1.5f;
			break;
		}
		case goForSeventhPath:
		{
			adjustVel.vel = distance * 1.5f;
			break;
		}
		case goForEighthPath:
		{
			adjustVel.vel = distance * 2.85f;//3.3
			if(GetY() < 3700.0f)
			{
				adjustVel.vel = distance * 1.2f;
			}
			break;
		}
		case goForNinthPath:
		{
			adjustVel.vel = distance * 2.2f;
			break;
		}
		case goForTenthPath:
		{
			adjustVel.vel = distance * 2.5f;
			if(GetY() < 4500.f)
			{
				adjustVel.vel = distance * 1.f;
			}
			break;
		}
		case goForEleventhPath:
		{
			adjustVel.vel = distance * 2.8f;//3.3
			if(GetX() < 800 && GetY() > 11000.0f && fabs(vell) < 100.0f)
			{
				adjustVel.vel = 0;
			}
			else if(GetY() < 3700.0f || GetY() > 10200.0f)
			{
				adjustVel.vel = distance * 2.15f;//2.45
			}
			break;
		}
		case goForTakeAgain:
		{
			adjustVel.vel = distance * 2.9f;
			break;
		}
		case goForFirstPlace:
		{
			adjustVel.vel = distance * 3.25f;//3.3
			if( GetY() > 8000.0f)
			{
				adjustVel.vel = distance * 1.6f;
			}
			break;
		}
		default:
		{
			adjustVel.vel = distance * 1.45f;
			break;
		}
	}
	
	
	//规划速度小于0.5m/s 不调节
	//if(vell < 500.f)
	// if(vell < 200.f)
	// {
	// 	adjustVel.vel = 0.0f;
	// }
	/*else if(vell < 1000.f)
	{
		adjustVel.vel = adjustVel.vel * 0.5f;
//		adjustVel.vel = 0.0f;
	}*/
	
	adjustVel.direction = angle;
	
	//对调节速度大小进行限制
	if(adjustVel.vel>=MAX_ADJUST_VEL)
	{
		adjustVel.vel = MAX_ADJUST_VEL;
	}
	
	return adjustVel;
}

void AdjustVel(float *carVel,float *velAngle,robotVel_t adjustVel)
{
	#define ADJUST_KP (1.0f)
	
	float angleErr = 0.0f;
	float projectOnvel = 0.0f;
	vector_t oriVel = {0} , adjust = {0} , result = {0};
	
	oriVel.module = *carVel;
	oriVel.direction = *velAngle;
	
	//计算调节目标速度的大小
	adjust.module = ADJUST_KP * adjustVel.vel;
	
//	if(oriVel.module>(10.0f * adjust.module))
//	{
//		adjust.module = oriVel.module/10.0f;
//	}
	
//	if(adjust.module>*carVel)
//	{
//		adjust.module = *carVel;
//	}
	
	//计算速度方向的调节量并进行限制
	angleErr = adjustVel.direction - *velAngle;
	
	angleErr = angleErr > 180.0f ? angleErr - 360.0f : angleErr; 
	angleErr = angleErr < -180.0f ? 360.0f + angleErr : angleErr;

	angleErr =  ADJUST_KP * angleErr;
	if(angleErr>180.0f)
	{
		angleErr = 180.0f;
	}
	else if(angleErr<-180.0f)
	{
		angleErr = -180.0f;
	}
	
	adjust.direction = *velAngle + angleErr;
	
	adjust.direction = adjust.direction > 180.0f ? adjust.direction - 360.0f : adjust.direction; 
	adjust.direction = adjust.direction < -180.0f ? 360.0f + adjust.direction : adjust.direction;
	
	
	//计算调节量在原速度上的投影大小
	// projectOnvel = CalculateVectorProject(adjust,oriVel);
	// //根据投影大小限制调节后的速度方向
	// if(projectOnvel<=-oriVel.module)
	// {
	// 	adjust = CalculateVectorFromProject(-oriVel.module,adjust.direction,oriVel.direction);
	// }
	
	result = CalculateVectorAdd(oriVel , adjust);
	//对调整结果的速度大小进行限制
	if(result.module>=GetVelMax())
	{
		result.module=GetVelMax();
	}
	
	*carVel = result.module;
	*velAngle = result.direction;
}


