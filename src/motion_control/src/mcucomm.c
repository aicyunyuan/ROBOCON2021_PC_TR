#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <stdarg.h>
#include <pthread.h> 
#include "mcucomm.h"
#include "spi_init.h"
#include "moveBase.h"
#include "robot.h"
#include "timer.h"
#include "pps.h"

#include "ControlSPI.h"
#include "ErrorType.h"
extern FILE *fpWrite;
typedef union 
{
	uint8_t data8[4];
	int32_t data32;
	float dataf;
}transData_t;
//修改接收发送时要修改两处接收函数和一处发送函数
//向 mcu master发送消息  
uint8_t tx[MCU_SPI_LENGTH] = {0}; 
//接受 mcu master 的消息
uint8_t rx[MCU_SPI_LENGTH] = {0};

//去除起止符的消息
uint8_t txTransData[MCU_SPI_LENGTH - 5] = {0};
uint8_t rxTransData[MCU_SPI_LENGTH - 5] = {0};
//接收到用不到的数据时用blank接受
float blankFloat[8] = {0};
uint8_t blank7;
//检测SPI函数时间
struct timeval SPIstart, SPIendTime;
long long SPItotal_time,SPIstartTime;


static uint8_t mcuTalkOk = 0;
pthread_t spi_thread_id;
void WaitMcuPrepare(void)
{
    while(!mcuTalkOk);
    // while(!mcuTalkOk)
    // {
    //     usleep(5000);
    //     Communicate2Mcu();
    // }
}

void Communicate2Mcu(void)
{
    uint8_t spiShiftFlag = 0;
    uint8_t SPIIdentifierCorrect = 0; 
    int read_data_num  = 0;
    int32_t readRet;
    //一直检测是否收到数据
    readRet = VSI_SlaveReadBytes(VSI_USBSPI, 0, rx, &read_data_num, 100);
    if (readRet != ERR_SUCCESS)
    {   
        printf("Slave Read data error!!!\n");
    }
    else
    {
        if (read_data_num > 0) //受到数据
        {
            // printf("receive success!!!\n");
            int writeRet=0;
            tx[0] = 'H';
            tx[1] = 'D';
            tx[MCU_SPI_LENGTH - 3] = '\r';
            tx[MCU_SPI_LENGTH - 2] = '\n';
            tx[MCU_SPI_LENGTH - 1] = 0;
            gRobot.cvMcuDir = (int)(gRobot.cvDir * 10);
            WriteSpiData(MCU_SPI_UINT8_CNT,MCU_SPI_FLOAT_CNT,MCU_SPI_LENGTH-5,txTransData,\
                        gRobot.walkStatus,gRobot.pathPlanFlag,\
                        gRobot.archeryStart,gRobot.CVstate, blank7,blank7,blank7,blank7,\
                        -gRobot.wheelState.oneTarget.vel, -gRobot.wheelState.twoTarget.vel,-gRobot.wheelState.thrTarget.vel,\
                        gRobot.wheelState.oneTarget.direction,gRobot.wheelState.twoTarget.direction,gRobot.wheelState.thrTarget.direction,\
                        gRobot.cvDir, gRobot.cvDis,GetX(),GetY(),GetAngle());
            for(int i = 0; i < (MCU_SPI_LENGTH - 5); i ++)
            {
                tx[i+2] = txTransData[i];
            }
            writeRet = VSI_SlaveWriteBytes(VSI_USBSPI, 0, tx, MCU_SPI_LENGTH);
            if (writeRet != ERR_SUCCESS)
            {
                printf("Slave write data error!!!\n");
            }
            if(rx[0] == 'H' && rx[1] == 'M' && rx[MCU_SPI_LENGTH-3] == '\r' && rx[MCU_SPI_LENGTH-2] == '\n')
            {
                spiShiftFlag  = 0;
                for(int i = 0; i < (MCU_SPI_LENGTH - 5); i ++)
                {
                    rxTransData[i] = rx[i+2];
                }
                SPIIdentifierCorrect = 1; 
            }
            else if(rx[1] == 'H' && rx[2] == 'M' && rx[MCU_SPI_LENGTH-2] == '\r' && rx[MCU_SPI_LENGTH-1] == '\n')
            {
                spiShiftFlag  = 1;
                for(int i = 0; i < (MCU_SPI_LENGTH - 5); i ++)
                {
                    rxTransData[i] = rx[i+3];
                }
                SPIIdentifierCorrect = 1; 
            }
            if(SPIIdentifierCorrect == 1)
            {
                gRobot.mcuHeart = 0;
                mcuTalkOk = 1;//收到数据，mcu通过
                ReadSpiData(MCU_SPI_UINT8_CNT,MCU_SPI_FLOAT_CNT,MCU_SPI_LENGTH-5,rxTransData,&gRobot.robotMode,&gRobot.walkMode,&gRobot.shootArrowCnt,\
                        &gRobot.attackPotID, &gRobot.colorFlag,&gRobot.fetchReady,&gRobot.TRRetryFlag,&gRobot.writeFlag,\
                        &gRobot.turnAngle, &gRobot.autoShootVel, &gRobot.voltageTemp, &gRobot.autoShootKp,\
                        &gRobot.autoShootKi, &gRobot.autoPitchAngle, &blankFloat[6], &blankFloat[7],&blankFloat[7],&blankFloat[7],&blankFloat[7]);
            }
            // printf("r ");
            // for(int i = 0; i<3;i++ )
            // {
            //     printf("%d ",(int)rx[i]);
            // }
            // printf("\n");
        
        }
    }
    // //与mcu通信
    // //检测函数时间 
    // gettimeofday(&SPIstart, NULL);

    // SPIDataRW(0,tx,rx,MCU_SPI_LENGTH);
    
    // gettimeofday(&SPIendTime, NULL);  
	// SPItotal_time = (SPIendTime.tv_sec - SPIstart.tv_sec) * 1000000 + (SPIendTime.tv_usec - SPIstart.tv_usec);
}
/**
	* @brief	WriteSpiData 将各个变量存入spi需要给下层板发送的数据内容数组中
  * @note		None
  * @param	uint8DataCnt：u8类型的标志位个数 
	* @param	floatDataCnt：float类型的数据个数
	* @param	dataLength: rx数组长度 必须等于n1+n2*4！！！
	* @param	txData：需要存入的数组首地址
	* @param	...：n1个u8类型变量，n2个float类型变量，注意个数！注意只能是u8和float类型变量！！！
  * @retval	None
  */
void WriteSpiData(uint8_t uint8DataCnt, uint8_t floatDataCnt, uint8_t dataLength,char* txData, ...)
{
    transData_t spiSendMessage;
    va_list args;
    va_start(args, txData); 
    for(uint8_t i = 0; i < uint8DataCnt; i ++)
    {
        txData[i] = (char)va_arg(args, int);
    }
    for(uint8_t i = 0; i < floatDataCnt; i ++)
    {
        spiSendMessage.dataf = (float)va_arg(args, double);
        for(int j = 0; j < 4; j ++)
        {
            txData[uint8DataCnt + i*4 + j] =  spiSendMessage.data8[j];
        }
    }
    va_end(args);
}

/**
	* @brief	ReadData 将下层板数组内容解析给各个变量
  * @note		None
  * @param	uint8DataCnt：u8类型的标志位个数 
	* @param	floatDataCnt：float类型的数据个数
	* @param	dataLength: rx数组长度 必须等于n1+n2*4！！！
	* @param	rxData：需要解析的数组首地址
	* @param	...：n1个u8类型变量取址，n2个float类型变量取址，注意个数！注意只能是u8和float类型变量的地址！！！
  * @retval	
  */
void ReadSpiData(uint8_t uint8DataCnt, uint8_t floatDataCnt, uint8_t dataLength,char* rxData, ...)
{
    transData_t spiRecMessage;
    va_list args;
    va_start(args, rxData); 

    for(uint8_t i = 0; i < uint8DataCnt; i ++)
    {
        *va_arg(args, uint8_t*) = rxData[i];
    }
    for(uint8_t i = 0; i < floatDataCnt; i ++)
    { 
        for(int j = 0; j < 4; j ++)
        {
            spiRecMessage.data8[j] = rxData[uint8DataCnt + i*4 + j]; 
        }
        *(va_arg(args,float*)) = (float)spiRecMessage.data32/100.f;
    }
    va_end(args);
}

//新线程读取主机数据
void* SpiRun(void* p_comm)
{
    
    while(1)
    {
        Communicate2Mcu();
    }
    return (void*)0;
}

int SPIInit(void)
{
    int ret;
    VSI_INIT_CONFIG SPI_Config;
    VSI_BOARD_INFO BoardInfo;
    ret = VSI_ScanDevice(1);
    if (ret <= 0)
    {
        printf("No SPI device connect! \n");
        //return ret;
    }
    else
    {
        printf("%d SPI device connect! \n",ret);
    }
    // Open device
    ret = VSI_OpenDevice(VSI_USBSPI, 0, 0);

    printf("VSI_OpenDevice %d\n",ret);
    if (ret != ERR_SUCCESS)
    {
        printf("Open SPI device error! \n");
        //return ret;
    }
    else
    {
        printf("SPI device found! \n");
    }

    SPI_Config.ControlMode = 0;
    SPI_Config.MasterMode = 0;
    SPI_Config.ClockSpeed = 2250000;
    SPI_Config.CPHA = 0;
    SPI_Config.CPOL = 0;
    SPI_Config.LSBFirst = 0;
    SPI_Config.TranBits = 8;
    SPI_Config.SelPolarity = 0;
    ret = VSI_InitSPI(VSI_USBSPI, 0, &SPI_Config);
    if (ret != ERR_SUCCESS)
    {
        printf("Initialize SPI device error! \n");

        //return ret;
    }
    else
    {
        printf("Initialize SPI device done! \n");
    }
   
     
}
void NewSpiThread(void)
{
    int i, retp;
    retp = pthread_create(&spi_thread_id, NULL, SpiRun, NULL);
    if (retp != 0)
    {
        printf("new thread establish failed!\n");
    }
    else
    {
        printf("new thread established ! ID is %d \n", retp);
    }
}
