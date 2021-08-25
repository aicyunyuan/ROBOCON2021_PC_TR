#ifndef __VISION_H
#define __VISION_H

#include "stdint.h"
typedef union 
{ 
	uint8_t data8[4];
	int32_t data32;
	float dataf;
}transCVData_t;

#define SEND_COLOR_DATA (5)
#define SEND_POT_DATA (18)

#define SEND_CV_DATA (18)

//等待cv初始化
void WaitCvPrepare(void);

//定周期运行
void Talk2Cv(void);

//接受视觉信息
void CvDataRecognize(void);

//保存角度信息
void SetDir(float setValue);

//保存距离信息
void SetDis(float setValue);

//将红蓝场、桶号、x、y、转盘角度传给视觉
void SendCVData(void);


#endif 
