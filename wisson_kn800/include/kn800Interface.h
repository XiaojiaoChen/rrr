#ifndef __KN800_INTERFACE_H
#define __KN800_INTERFACE_H
#include <sys/timeb.h>
#include <queue>
#include <thread>
#include <string>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#define SEND_START_FRAME_HEAD   0XFF
#define SEND_START_FRAME_SECOND 0XFF
#define SEND_DATA_FRAME_ID  0XFD
#define SEND_DATA_FRAME_CMD 0X01
#define USART_PARA_DATA_LEN  10
// 接收数字数据
#define RECV_START_FRAME_HEAD 0XFF
#define RECV_START_FRAME_SECOND 0XFF
#define RECV_DATA_FRAME_ID 0XFD
namespace WISSON_ROBOTICS
{
struct kn800State
{
    int16_t elongation;
    int16_t orientation;
    int16_t bending;
    int16_t grasp;
};
 

class KN800Arm
{
public:
    enum ArmStatus
    {
        ArmStatus_DISCONNECT = 0,//断开连接
        ArmStatus_CONNECTFAILED,//连接失败
        ArmStatus_CONNECTED,//连接
        ArmStatus_STANDBY,//待机
        ArmStatus_MOVING,//正在移动
        ArmStatus_OVERRUN,//停止运动
        ArmStatus_REACHTARGET,//指令完成
    };

    KN800Arm();
    ~KN800Arm();
 
    int connectArm();
    void disconnectArm();
    ArmStatus getArmStatus() {  return status_;  }

    //serial Lowlevel
    void setName(const char *name);
    int serialOpen(const char *port);
    int serialInit(uint32_t ibaudRate, int flow_ctrl, int databits, int stopbits, int parity);
    int sendData(const char *data, size_t data_len) ;
    int recvData(char *buffer, size_t data_len) ;
    uint16_t crc16Updata(uint8_t *targetSrc,uint32_t lengthInBytes);

    void sendCommand(uint8_t *pdata,uint16_t paraDataLength);
    void recvSensor(); 
 
    int updateState();
    int updateCommand(int16_t compositeCmd,int16_t elongate,int16_t  bend_direction,int16_t bend,int16_t grasp);

    kn800State              _state;
    const char*             port_name_;
private:
    ArmStatus               status_;

    int                     current_status_;
    int                     pre_status_;
    bool                    is_frame_;
    bool                    isconnect;
    bool                    isStop;
    unsigned char           index_;
    int                      fd_; 
    uint16_t                recvBuffer_[4];
};
}
#endif
