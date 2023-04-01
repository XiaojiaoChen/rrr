#include "kn800Interface.h"
#include <string.h>
namespace WISSON_ROBOTICS
{
KN800Arm::KN800Arm()
{
    status_ = ArmStatus_DISCONNECT;
   
}
KN800Arm::~KN800Arm()
{
    status_ = ArmStatus_DISCONNECT;
    close(fd_);

}

void KN800Arm::setName(const char *name)
{
    port_name_ = name;
}

int KN800Arm::connectArm()
{
    // const char *port = port_name;
    //printf("Connecting to kn800 at %s \n", port_name_);
    uint32_t baudrate = 115200;
    int ret = serialOpen(port_name_);
    if (ret < 0)
    {
         // 打开串口失败，抛出异常
        //throw std::runtime_error("Failed to open port");
        status_= ArmStatus_DISCONNECT;
        return ret;
    }

    this->serialInit(baudrate,0,8,1,'N');
    status_= ArmStatus_CONNECTED;
    return ret;
}
int KN800Arm::serialOpen(const char *port)
{

    fd_ = open(port, O_RDWR | O_NOCTTY | O_NDELAY); //O_RDWR|O_NDELAY
    if (fd_ < 0)
    {
        perror("Can't Open Serial Port");
        return -1;
    }
    else
    {
        // //恢复串口为阻塞状态
        // if (fcntl(devfd, F_SETFL, 0) < 0)
        // {
        //     printf("fcntl failed!\n");
        //     return -2;
        // }
        // else
        // {
        //     printf("fcntl=%d\n", fcntl(devfd, F_SETFL, 0));
        // }

        // if(fcntl(devfd,F_SETFL,FNDELAY) < 0)//非阻塞，覆盖前面open的属性
		// {   
		// 	printf("fcntl FNDELAY failed\n");   
		// }   

        //设置非阻塞
        int flag = fcntl(fd_,F_GETFL,0);
        if(flag < 0)
		{   
			printf("fcntl F_GETFL failed\n");   
            return -2;
		}

        flag |= O_NONBLOCK;
        if(fcntl(fd_,F_SETFL,flag) < 0)
		{   
			printf("fcntl F_SETFL failed\n");   
            return -3;
		}

        //测试是否为终端设备
        
        // if(0 == isatty(STDIN_FILENO))
        // {
        //     printf("standard input is not a terminal device\n");
        //     return -4;
        // }
        // else
        // {
        //     printf("isatty success!\n");
        // }
    }
    // printf("fd_->open=%d\n", fd_);

    // this->isOk = true;
    return 0;
}
int KN800Arm::serialInit(uint32_t ibaudRate, int flow_ctrl, int databits, int stopbits, int parity)
{
    uint16_t j;
    uint32_t speed_arr[10][2] = {{   300,   B300},
                                {  1200,  B1200},
                                {  2400,  B2400},
                                {  4800,  B4800},
                                {  9600,  B9600},
                                { 19200, B19200},
                                { 57600, B57600},
                                {115200,B115200},
                                {230400,B230400},
                                {921600,B921600}};

    struct termios options;
    /*tcgetattr(devfd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，
    该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.  */
    if(tcgetattr(fd_, &options) != 0)
    {
        perror("target port status is wrong!\n");
        return -1;
    }

    //设置串口输入波特率和输出波特率
    for(j = 0; j < sizeof(speed_arr)/(sizeof(int) * 2); j++)
    {
        if (ibaudRate == speed_arr[j][0])
        {
            cfsetispeed(&options, speed_arr[j][1]);
            cfsetospeed(&options, speed_arr[j][1]);
            break;
        }
    }

    if(j >= sizeof(speed_arr)/(sizeof(int) * 2))
    {
        perror("unsupport bit bound value!\n");
        return -2;
    }
    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch (flow_ctrl)
    {

    case 0: //不使用流控制
        options.c_cflag &= ~CRTSCTS;
        break;

    case 1: //使用硬件流控制
        options.c_cflag |= CRTSCTS;
        break;
    case 2: //使用软件流控制
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 5:
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr, "Unsupported data size\n");
        return -2;
    }
    //设置校验位
    switch (parity)
    {
    case 'n':
    case 'N': //无奇偶校验位。
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        break;
    case 'o':
    case 'O': //设置为奇校验
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E': //设置为偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
    case 's':
    case 'S': //设置为空格
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported parity\n");
        return -3;
    }
    // 设置停止位
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported stop bits\n");
        return -4;
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    //options.c_lflag &= ~(ISIG | ICANON);
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1;  /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd_, TCIFLUSH);
    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd_, TCSANOW, &options) != 0)
    {
        perror("com set error!\n");
        return -5;
    }

    //设置非阻塞
    {
        int flag;
        flag = fcntl(fd_,F_GETFL);
        //先获取原文件状态(假定函数执行成功返回文件状态)
        flag |= FNDELAY;//O_NONBLOCK;
        //去除非阻塞的flag
        fcntl(fd_,F_SETFL,flag);
    }
    return 0;
}

void KN800Arm::disconnectArm()
{
    if( status_ != ArmStatus_DISCONNECT)
    {
        status_ = ArmStatus_DISCONNECT;
        close(fd_);

    }
}


// 接收数据
int KN800Arm::recvData(char *buffer, size_t data_len) {
    int len, fs_sel;
    fd_set fs_read;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd_, &fs_read);

    time.tv_sec = 0;
    time.tv_usec = 1000;

    //使用select实现串口的多路通信
    fs_sel = select(fd_ + 1, &fs_read, NULL, NULL, &time);
    if(fs_sel)
    {
        len = read(fd_, buffer, data_len);
        return len;
    }
    else
    {
        return -1;
    }
}


void KN800Arm::recvSensor() 
{
    uint8_t circlyCount = 0,startFrameSuccer = 0,recvDataCnt = 0,frameLength = 0,startPos = 0;
    static uint8_t data_frame[64] = {0};
    uint16_t crcCheck = 0,recvCheck = 0;

    ssize_t len = recvData(reinterpret_cast<char*>(data_frame), sizeof(data_frame));
    if (len == -1) {
        return ;
        throw std::runtime_error("Failed to receive data frame");
    }else{
        for(circlyCount = 0 ; circlyCount < sizeof(data_frame) ; circlyCount++) 
        {
            if( startFrameSuccer == 0      && 
                data_frame[circlyCount -3]  == RECV_START_FRAME_HEAD &&
                data_frame[circlyCount -2]  == RECV_START_FRAME_SECOND &&
                data_frame[circlyCount -1]  == RECV_DATA_FRAME_ID ) 
            {
                startFrameSuccer = 1; 
                frameLength = data_frame[circlyCount]; 
                recvDataCnt = 1; 
                startPos = circlyCount -3;
                // printf("  recv_data succeed length %02x \n", frameLength);
                // for(int i = startPos;i < frameLength + 5;i++)
                //        {
                            
                //              printf(" %02x ", data_frame[i]);

                //        }  
            }

            if(startFrameSuccer == 1)
            {
               recvDataCnt++;
               if(recvDataCnt >= frameLength + 5)  
               {
                   crcCheck = crc16Updata(&data_frame[startPos],frameLength + 3);

                   recvCheck = (data_frame[frameLength + 4] *256) + data_frame[frameLength + 3];
                   if(crcCheck != recvCheck)
                   {
                      return;
                    //   throw std::runtime_error("crc check error");
                   }
                   else //data handler
                   {
                      memcpy(recvBuffer_,&data_frame[5],8); //copy target para to buff  
                        // int16_t elongate ;
                        // int16_t  bend_direction;
                        // int16_t bend;
                        // int16_t grasp;
                        // elongate = recvBuffer_[0];
                        // bend_direction = recvBuffer_[1];
                        // bend = recvBuffer_[2];
                        // grasp = recvBuffer_[3];

                   }
                   startFrameSuccer  = 0; //flag clear
                   recvDataCnt = 0;  //cnt clear
               }
            }
        }
    }
    
    
}

int KN800Arm::updateState( )
{
    int ret = 0;
    recvSensor(); 
 
    _state.elongation = recvBuffer_[0];
    _state.orientation = recvBuffer_[1];
    _state.bending = recvBuffer_[2];
    _state.grasp = recvBuffer_[3];
    
    return ret;
}

int KN800Arm::updateCommand(int16_t compositeCmd,int16_t elongate,int16_t  bend_direction,int16_t bend,int16_t grasp)
{
    uint8_t paraDataPack[USART_PARA_DATA_LEN] = {0};
    memcpy(paraDataPack,&compositeCmd,2);
    memcpy(&paraDataPack[2],&elongate,2);
    memcpy(&paraDataPack[4],&bend_direction,2);
    memcpy(&paraDataPack[6],&bend,2);
    memcpy(&paraDataPack[8],&grasp,2);

    sendCommand(paraDataPack,USART_PARA_DATA_LEN);
    // std::cout<<"robot arm start move"<<std::endl;

    return 0;
}



void KN800Arm::sendCommand(uint8_t *pdata,uint16_t paraDataLength) 
{
    uint8_t legal_data_byte_len = static_cast<uint8_t>(paraDataLength)+2;  
    uint8_t data_frame_all_length = paraDataLength + 7;
    uint8_t data_frame[100] = {SEND_START_FRAME_HEAD, SEND_START_FRAME_SECOND , SEND_DATA_FRAME_ID,legal_data_byte_len,SEND_DATA_FRAME_CMD};
    uint16_t crcCheck = 0;

    memcpy(&data_frame[5],pdata,paraDataLength);
    //check   
    crcCheck = crc16Updata(data_frame,data_frame_all_length-2);
    data_frame[data_frame_all_length - 2] = crcCheck;
    data_frame[data_frame_all_length - 1] = crcCheck >> 8;

    sendData(reinterpret_cast<char*>(data_frame),data_frame_all_length); //send data to usart
}


int KN800Arm::sendData(const char *data, size_t data_len) {

        int len = 0;

        len = write(fd_, data, data_len);
        tcflush(fd_, TCIOFLUSH);
        if (len == data_len)
        {
            // printf("send data is %s\n", data);
            // std::cout<<"=========== send_data send frame  =================="<<"len "<<len<<std::endl;
            // for(int i = 0;i < len;i++)
            // {
                
            //         printf(" %02x ", data[i]);

            // }  
            return len;
        }
        else
        {

            for(int i = 0;i < len;i++)
            tcflush(fd_, TCIOFLUSH);
            return -1;
        }
    }



uint16_t KN800Arm::crc16Updata(uint8_t *targetSrc,uint32_t lengthInBytes)
{
    uint8_t i, temp;
    uint16_t j, CRCode;
    CRCode = 0xFFFF;
    for (j = 0; j < lengthInBytes; j++)
    {
            CRCode = CRCode ^ targetSrc[j];
            for (i = 0; i < 8; i++)
            {
                    temp = CRCode & 0x0001;
                    CRCode = (CRCode >> 1);
                    if (temp == 1)
                            CRCode = (CRCode ^ 0xa001);
            }
    }
    return CRCode;
}
}