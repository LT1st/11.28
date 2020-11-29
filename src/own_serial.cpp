#include "own_serial.h"
int fd = 0;
uint8_t sbuff[64];
uint8_t rbuff[20];
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
    unsigned char tbuf[4];
	unsigned char *p = (unsigned char*)&distanceLeft + 3;//指针p先指向float的最高字节
	float res;//验证float拆分为4个字节后，重组为float的结果
    uint8_t buffData[17];
    dataUnion leftDisUnion;
    dataUnion rightDisUnion;
    dataUnion angleUnion;
    leftDisUnion.data = distanceLeft;
    rightDisUnion.data = distanceRight;
    angleUnion.data = angel;

    buffData[0] = 'A';
    buffData[1] = 'T';
     switch (status)
     {
     case 0:
         buffData[2] = '0';
         break;
     case 1:
         buffData[2] = '1';
         break;
     case 2:
         buffData[2] = '2';
         break;
     case 3:
         buffData[2] = '3';
         break;
     case 4:
         buffData[2] = '4';
         break;
     case 5:
         buffData[2] = '5';
         break;
     default:
         break;
     }
    /* uint8_t statusChar = '0';
    buffData[2] = statusChar; */

    buffData[3] = *(p-3);
	buffData[4] = *(p-2);
	buffData[5] = *(p-1);
	buffData[6] = *p;

    for(int i=3;i<7;i++)
        printf("%x\t%d\t", buffData[i], (int)buffData[i]);
        //std::cout << "\t" << (char)buffData[i] <<"\t";

    test_float_to_4hex(angel);

    //CopyData(&leftDisUnion.dataChar[0],&buffData[3],4);
    CopyData(&rightDisUnion.dataChar[0],&buffData[7],4);
    CopyData(&angleUnion.dataChar[0],&buffData[11],4);

    buffData[15] = '\r';
    buffData[16] = '\n';

    int ifWrite =0;
    for (size_t i = 0; i < 17; i++)
    {
        std::cout  <<  (int)buffData[i]  <<  std::endl;
        ifWrite = write(fd,&buffData[i],1);
        printf("%c", buffData[i] );
    }
    
}

int own_serial::sendGesture(int Index)
{
    char a = gestureNum[Index];
    uint8_t buffData[5];
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
    std::cout<<"我找到你的手====>"<<Index<<std::endl;
}
static int cnt =0;
void signal_handler_IO(int status)
{
    uint8_t data;
    cnt++;
    //static int fd =status;
    int nread = read(fd, &rbuff, 4);
    std::cout <<" nread "<<nread<<std::endl;
    if (nread>0)
    {
        std::cout<<rbuff<<" "<<cnt<<std::endl;
    }
}

void CopyData(uint8_t* origen, uint8_t* afterTreat, int size)
{
	for (size_t i = 0; i < size; i++)
	{
		*afterTreat = *origen;
		afterTreat++;
		origen++;
	}
}

void test_float_to_4hex(float num)
{
	unsigned char tbuf[4];
    //指针p先指向float的最高字节
	unsigned char *p = (unsigned char*)&num + 3;
	float res;//验证float拆分为4个字节后，重组为float的结果

	//传入的float的值
	printf("\r\n传入的float的值：%f", num);

	//获取对应的4个字节，从低位到高位，这时就可以用于hex格式的数据传输了
	tbuf[0] = *(p-3);
	tbuf[1] = *(p-2);
	tbuf[2] = *(p-1);
	tbuf[3] = *p;

	//printf("\r\n查看float的每个字节内容(16进制):\r\n");
	printf("\t%x,%x,%x,%x\r\n", tbuf[0], tbuf[1], tbuf[2], tbuf[3]);

	//对拆分后的4个字节进行重组，模拟接收到hex后的数据还原过程
	unsigned char *pp = (unsigned char*)&res;
	pp[0] = tbuf[0];
	pp[1] = tbuf[1];  
	pp[2] = tbuf[2];
	pp[3] = tbuf[3];

	//printf("\r\n重组后的float的值：%f\r\n", res);
}