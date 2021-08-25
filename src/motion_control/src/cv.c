#include "cv.h"
#include "process_comm.h"
#include "pps.h"
#include "mcucomm.h"
#include "robot.h"

 

transCVData_t cvReceiveMessage;
uint8_t CvTalkOk = 0;
char recCvData[MC_CV_BUF_LENGTH] = {0};
//等待cv初始化
void WaitCvPrepare(void)
{
	ProcessCommInit(MC_CV_ID,MC);
    // printf("success 111\n\n\n");
    char cvDataInit[MC_CV_BUF_LENGTH] = {0};
    printf("start write data  ");
    ProcessCommWrite(cvDataInit,MC_CV_BUF_LENGTH);
    // //加入一定的响应机制
    while(!CvTalkOk)
    {
        usleep(5000);
        Talk2Cv();
       printf("23\r\n");
    }
}

void Talk2Cv(void) //定周期运行
{
	ProcessCommRead(recCvData, MC_CV_BUF_LENGTH);//读取视觉发来的数据，复制到recData中
    // printf("x y \r\n");
    CvDataRecognize();//指令识别
    
}

//接受视觉信息
void CvDataRecognize(void)
{
    // printf("start recognize");
    if(recCvData[0] == 'C' && recCvData[1] == 'V' && recCvData[MC_CV_BUF_LENGTH - 2] == '\r' && recCvData[MC_CV_BUF_LENGTH - 1] == '\n')
    {
        CvTalkOk = 1;
        gRobot.CVstate = recCvData[2];
        for(uint8_t i = 0; i < 4; i++)
        {
            cvReceiveMessage.data8[i] = recCvData[i+3];
        }
        SetDir(cvReceiveMessage.dataf);

        for(uint8_t i = 0; i < 4; i++)
        {
            cvReceiveMessage.data8[i] = recCvData[i+7];
        }
        SetDis(cvReceiveMessage.dataf);
    }     
}

void SetDir(float setValue)
{
	gRobot.cvDir = setValue;

    //将角度控制在0~180，-180~0
    if(gRobot.cvDir > 180)
    {
        gRobot.cvDir -= 360;
    }
    else if(gRobot.cvDir < -180)
    {
        gRobot.cvDir += 360;
    }
}

void SetDis(float setValue)
{
    gRobot.cvDis = setValue;
} 

/**将红蓝场、桶号、x、y、转盘角度传给视觉
  * 
  * 
  *
  */
void SendCVData(void)
{
    // printf("send111111111111111111111111");
    // gRobot.colorFlag = 1;//red
    // gRobot.attackPotID = 5;//2B
    union 
    {
        char dataC[SEND_CV_DATA-6];
        float dataF[(SEND_CV_DATA-6)/4];
    }sendData;
    
    char tdata[SEND_CV_DATA];
    tdata[0]='M';
    tdata[1]='C'; 
    tdata[SEND_CV_DATA-2]='\r';
    tdata[SEND_CV_DATA-1]='\n';

    tdata[2]=gRobot.colorFlag; 
    tdata[3]=gRobot.cvAttackPotID;
   
    sendData.dataF[0]=GetX();
    sendData.dataF[1]=GetY();
    sendData.dataF[2]=gRobot.turnAngle;


   
    memcpy(tdata+4,sendData.dataC,SEND_CV_DATA-6);//？
    

    // printf("send start!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    //进程通讯发送信息
    ProcessCommWrite(tdata,MC_CV_BUF_LENGTH);
    
}
