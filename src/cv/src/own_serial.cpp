#include "own_serial.h"
int fd = 0;
char sbuff[64];
char rbuff[20];
own_serial::own_serial(const char *port,
          uint32_t baudrat)
{
    std::cout<<"1\n";
    this_port = port; 
    this_baudrate = baudrat;
}
own_serial::~own_serial()
{
}

int own_serial::init(int &fd1)
{

    fd1 = open(this_port, O_RDWR| O_NOCTTY| O_NDELAY);
    
    while (fd1 == -1)
    {
        fd1 = open(this_port, O_RDWR| O_NOCTTY| O_NDELAY);
        perror("open_port: Unable to open serial\n");   
        std::cout<<"fd1="<<fd1<<std::endl;
    //return 0;
    }
    // saio.sa_handler = signal_handler_IO;
    // //signal_handler_IO(ifSerial);
    // saio.sa_flags = 0;
    // saio.sa_restorer = NULL; 
    // sigaction(SIGIO,&saio,NULL);
    // fcntl(fd1, F_SETFL, FNDELAY);
    // fcntl(fd1, F_SETOWN, getpid());
    // fcntl(fd1, F_SETFL, O_NDELAY| O_ASYNC ); 
    tcgetattr(fd1,&termAttr);
    //baudRate = B115200;          /* Not needed */
    cfsetispeed(&termAttr,this_baudrate);
    cfsetospeed(&termAttr,this_baudrate);
    termAttr.c_cflag &= ~PARENB;
    termAttr.c_cflag &= ~CSTOPB;
    termAttr.c_cflag &= ~CSIZE;
    termAttr.c_cflag |= CS8;
    termAttr.c_cflag |= (CLOCAL | CREAD);
    termAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    termAttr.c_iflag &= ~(IXON | IXOFF | IXANY);
    termAttr.c_oflag &= ~OPOST;
    termAttr.c_cc[VMIN] = 0;
    termAttr.c_cc[VTIME] = 1;
    tcflush(fd1,TCIOFLUSH);
    tcsetattr(fd1,TCSANOW,&termAttr);
    fd = fd1;
    std::cout<<"serial configured....\n";
}

int own_serial::isopen(void)
{
    return(ifSerial);
}
int own_serial::writeData(float distanceLeft,float distanceRight,float angel,int status)
{
    char buffData[17];
    dataUnion leftDisUnion;
    dataUnion rightDisUnion;
    dataUnion angleUnion;
    leftDisUnion.data = distanceLeft;
    rightDisUnion.data = distanceRight;
    angleUnion.data = angel;

    buffData[0] = 'A';
    buffData[1] = 'T';
    buffData[2] = status;

    CopyData(&leftDisUnion.dataChar[0],&buffData[3],4);
    CopyData(&rightDisUnion.dataChar[0],&buffData[7],4);
    CopyData(&angleUnion.dataChar[0],&buffData[11],4);

    buffData[15] = '\r';
    buffData[16] = '\n';

    int ifWrite =0;
    for (size_t i = 0; i < 17; i++)
    {
        std::cout<<(int)buffData[i]<<std::endl;
        ifWrite = write(fd,&buffData[i],1);
    }
    
}
int own_serial::sendGesture(int Index)
{
    char a = gestureNum[Index];
    char buffData[5];
    buffData[0] = 'G';
    buffData[1] = 'S';
    buffData[2] = a;
    buffData[3] = '\r';
    buffData[4] = '\n';
    int ifWrite =0;
    for (size_t i = 0; i < 5; i++)
    {
        ifWrite = write(fd,&buffData[i],1);
    }
}

char own_serial::encrypt(float angle, float distance, char state)
{
    // std::cout<<"angle = "<<angle<<std::endl;
    dataUnion angleUnion;
    dataUnion distanceUnion;
    angleUnion.data = (float)angle;
    distanceUnion.data = (float)distance;
    cvData[0] = 'C';
    cvData[1] = 'V';
    cvData[2] = state;
    // for(int i=0 ; i<4 ; i++)
    // {
    //     printf("singal = %d\n" , angleUnion.dataChar[i]);
    // }
    CopyData(&angleUnion.dataChar[0],&cvData[3],4);
    CopyData(&distanceUnion.dataChar[0],&cvData[7],4);
    cvData[11] = 'Z';
    cvData[12] = 'A';
    cvData[13] = 'W';
    cvData[14] = 'S';
    cvData[15] = 'B';
    cvData[16] = '\r';
    cvData[17] = '\n';
    ProcessCommWrite(cvData, MC_CV_BUF_LENGTH);
}

char own_serial::decipher(void)
{

}


static int cnt =0;
void signal_handler_IO(int status)
{
    char data;
    cnt++;
    //static int fd =status;
    int nread = read(fd, &rbuff, 4);
    std::cout <<" nread "<<nread<<std::endl;
    if (nread>0)
    {
        std::cout<<rbuff<<" "<<cnt<<std::endl;
    }
}

void CopyData(char* origen, char* afterTreat, int size)
{
	for (size_t i = 0; i < size; i++)
	{
		*afterTreat = *origen;
		afterTreat++;
		origen++;
	}
}

void CopyData_i(char* origen, char* afterTreat, int size)
{
	for (size_t i = 0; i < size; i++)
	{
		afterTreat[i] = origen[i];
		// afterTreat++;
		// origen++;
	}
}

void own_serial::getMCmessage(void) //定周期运行
{
	ProcessCommRead(recCvData, MC_CV_BUF_LENGTH);//读取运动控制发来的定位的数据，复制到recData中
    McDataRecognize();//指令识别
    
}

//接受运动控制信息
void own_serial::McDataRecognize(void)
{
    dataUnion xUnion;
    dataUnion yUnion;
    dataUnion angleUnion;
    if(recCvData[0] == 'M' && recCvData[1] == 'C' && recCvData[MC_CV_BUF_LENGTH - 2] == '\r' && recCvData[MC_CV_BUF_LENGTH - 1] == '\n')
    {
        color = recCvData[2];
        barrel = recCvData[3];
        for(uint8_t i = 0; i < 4; i++)
        {
            xUnion.dataChar[i] = recCvData[i+4];
        }
        for(uint8_t i = 0; i < 4; i++)
        {
             yUnion.dataChar[i] = recCvData[i+8];
        }
        for(uint8_t i = 0; i < 4; i++)
        {
             angleUnion.dataChar[i] = recCvData[i+12];
        }
        x = xUnion.data;
        y = yUnion.data;
        angle = angleUnion.data;
        std::cout << "颜色：" << (int)color << "  桶：" << barrel << "  x=" << xUnion.data << "  y=" <<  yUnion.data << "  angle=" << angleUnion.data << std::endl;
    }     
}


