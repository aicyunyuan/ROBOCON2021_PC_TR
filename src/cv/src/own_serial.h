#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h> 
#include <signal.h>
#include <string>
#include <iostream>
#include <poll.h>

#include "kinect_init.h"



union dataUnion
{
	char dataChar[4]={0};
	float data;
};

 /* definition of signal handler */
void CopyData(char* origen, char* afterTreat, int size);
void CopyData_i(char* origen, char* afterTreat, int size);
void signal_handler_IO (int status);  




class own_serial
{
private:
    /* data */
public:
    own_serial(const char * port = "",
    uint32_t baudrate = B115200);
    ~own_serial();
    
    char encrypt(float angle, float distance, char state);
    int isopen(void);
    int init(int &fd1);
    int writeData(float distanceLeft = 0.0,float distanceRight = 0.0,float angel = 0.0,int status =0);//status为状态 0没有找到 1找到2条边 3找到左边的那条边 4找到右边的那条边
    int sendGesture(int Index);
    char decipher(void); 
    void McDataRecognize(void);
    void getMCmessage(void);


    char gestureNum[8]={'0','1','2','3','4','5','6','7'};
    //int signal_handler_IO(int stuas);
    int this_fd = 0;
    float numSend = 0.0;
    char cvData[18];
    char recCvData[MC_CV_BUF_LENGTH] = {0};

    float x = 0.f;
    float y = 0.f;
    float angle = 0.f;
    char color = 0;
    char barrel = 0;

private:
    struct termios termAttr;
    struct sigaction saio;
    int ifSerial = 0;
    const char * this_port;
    uint32_t this_baudrate = 9600;  
public:

};


 
