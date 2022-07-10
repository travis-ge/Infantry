//
// Created by Quanyi on 2021/9/29.
//

#include "serial_port.h"
#include <iostream>
#include <unistd.h>          // Unix 标准函数定义
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>           // 文件控制定义
#include <termios.h>         // PPSIX 终端控制定义
#include <string.h>
#include "serial_port.h"
#include "base.h"
using namespace std;

std::mutex mtx_port;
extern SerialPort port;
extern long times;
extern Ptz_infor stm;
void infor_clear(){
    stm.bulletSpeed = stm.pitch = stm.yaw = 0;
}
/*************************************************
Function:       SerialPort
Description:    构造函数
Input:
Output:
Return:
Others:         初始化变量 创建接收线程
*************************************************/
SerialPort::SerialPort()
{

    for (int i = 0; i < COM_BUFF_LEN; i++)
    {
        buff_w_[i] = 0;
        buff_r_[i] = 0;
    }
}
/*************************************************
Function:       OpenDev
Description:    打开串口
Input:          device
Output:
Return:         fd or -1
Others:         LINUX中 open 函数作用：打开和创建文件
                O_RDONLY 只读打开  O_WRONLY 只写打开  O_RDWR 读，写打开
                对于串口的打开操作，必须使用O_NOCTTY参数，它表示打开的是一个终端设备，程序不会成为该端口的控制终端。如果不使用此标志，任务的一个输入(比如键盘终止信号等)都会影响进程
                O_NDELAY表示不关心DCD信号所处的状态（端口的另一端是否激活或者停止）
                O_NONBLOCK 设置为非阻塞模式，在read时不会阻塞住，在读的时候将read放在while循环中
*************************************************/
int SerialPort::OpenDev(const char *dev)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);//|O_NONBLOCK);
    if (fd == -1)
        cout << "串口打开失败" << endl;
    else
        tcflush(fd, TCIOFLUSH);   // 清空输入输出缓存

    return (fd);
}

/*************************************************
Function:       SetSpeed
Description:    设置波特率
Input:          fd speed
Output:
Return:         true or false
Others:
*************************************************/
unsigned int speed_arr[] = {B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
unsigned int name_arr[] = {115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300};
bool SerialPort::SetSpeed(int fd, int speed)
{
    int status;
    struct termios options;
    tcgetattr(fd, &options);
    for (int i= 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&options, speed_arr[i]);//输入波特率
            cfsetospeed(&options, speed_arr[i]);//输出波特率
            options.c_cflag |= (CLOCAL | CREAD);//本地控制模式 //保证程序不会成为端的占有者//使端口能读取输入的数据
            status = tcsetattr(fd, TCSANOW, &options);//设置串口文件
            if (status != 0)
            {
                cout << "波特率设置失败" << endl;
                return false;
            }
        }
    }
    return true;
}
/*************************************************
Function:       SetParity
Description:    设置串口数据位，效验和停止位
Input:          fd         类型 int  打开的串口文件句柄
                data_bits  类型 int  数据位 取值 为 7 或者 8
                parity     类型 char 效验类型 取值为 N, E, O, S
                stop_bits  类型 int  停止位 取值为 1 或者 2
Output:
Return:
Others:
*************************************************/
bool SerialPort::SetParity(int &fd, int data_bits, char parity, int stop_bits)
{
    struct termios options;
    /*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关错误信息*/
    if (tcgetattr(fd, &options) != 0)
    {
        cout << "串口设置失败" << endl;
        return false;
    }
    /*options.c_cflag &= ~ CSIZE;
    switch (data_bits)
    {
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            cout << "不支持的数据位" << endl;
            return false;
    }
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB; // Clear parity enable
            options.c_iflag &= ~INPCK; // Enable parity checking
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB); // 设置为奇效验
            options.c_iflag |= INPCK; // Disable parity checking
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB; // Enable parity
            options.c_cflag &= ~PARODD; // 转换为偶效验
            options.c_iflag |= INPCK; // Disable parity checking
            break;
        case 's':
        case 'S': // as no parity
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            cout << "不支持的校验类型" << endl;
            return false;
    }
    switch (stop_bits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            cout << "不支持的停止位" << endl;
            return false;
    }
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 设置为原始模式
    options.c_oflag &= ~ OPOST;*/
/*  直接设置
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_cflag = 2237; // 6322-115200 2237-9600
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0; // 等待时间
    options.c_cc[VMIN] = 0; // 最小接收字符
*/  //fcntl(fd,F_SETFL,FNDELAY);

    //直接设置
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    if (tcsetattr(fd, TCSANOW, &options) != 0) // Update the options and do it NOW
    {
        cout << "串口错误" << endl;
        return false;
    }

    cout << options.c_iflag << endl;
    cout << options.c_oflag << endl;
    cout << options.c_cflag << endl;
    cout << options.c_lflag << endl;
    cout << options.c_ispeed << endl;
    cout << options.c_ospeed << endl;
    return true;
}
/*************************************************
Function:       PortInit
Description:    初始化串口
Input:          串口号 波特率
Output:
Return:         true or false
Others:         bool
*************************************************/
bool SerialPort::PortInit(int device, int speed)
{
    const char *dev;

    if (device == 0)
        dev = "/dev/stm"; // ttyUSB0 USB串口
    else
    {
        cout << "Open serial error!!!" << endl;
        return false;
    }

    fd_ = OpenDev(dev);
    if (fd_ == -1){
        return false;
    }

    if (!SetSpeed(fd_, speed)){
        return false;
    }

    // 数据位 8  校验 无  停止位 1
    if (!SetParity(fd_, 8, 'N', 1)){
        return false;
    }

    return true;
}
/*************************************************
Function:       Read
Description:    串口读数据
Input:          buff length
Output:
Return:         true or false
Others:         bool；read()默认为阻塞模式，没有读到数据会阻塞住；若在前面设置为非阻塞模式，没有读到数据会返回-1
*************************************************/
int SerialPort::Read(char *r_buff, size_t length)
{
    size_t byte_read = 0;
    byte_read = read(fd_, r_buff, length);

   // if(byte_read == 0){ PortInit(0, 115200);}
    tcflush(fd_, TCIFLUSH); // 清空输入队列  TCIFLUSH 输入队列  TCOFLUSH 输出队列  TCIOFLUSH 输入输出队列
    return byte_read;
}
/*************************************************
Function:       Write
Description:    串口写数据
Input:          buff length
Output:
Return:         true or false
Others:         bool
*************************************************/
bool SerialPort::Write(char *w_buff, size_t length)
{
    //cout << length << endl;
    write(fd_, w_buff, length);
    //tcflush(fd_, TCOFLUSH); // 清空输出队列  TCIFLUSH 输入队列  TCOFLUSH 输出队列  TCIOFLUSH 输入输出队列
    return true;
}
/*************************************************
Function:       ISO14443AAppendCRCA
Description:    CRC16 打包
Input:          buff byte_count
Output:
Return:
Others:
*************************************************/
void SerialPort::ISO14443AAppendCRCA(void* buffer, unsigned short byte_count)
{
    unsigned short check_sum = 0x6363;   //1101111011
    unsigned char* data = (unsigned char*)buffer;

    while (byte_count --)
    {
        unsigned char byte = *data++;

        byte ^= (unsigned char)(check_sum & 0x00ff);
        byte ^= byte << 4;

        check_sum = (check_sum >> 8) ^ ((unsigned short)byte << 8) ^ ((unsigned short)byte << 3) ^((unsigned short)byte >> 4);
    }
    *data++ = (check_sum >> 0) & 0x00ff;
    *data = (check_sum >> 8) & 0x00ff;
}
/*************************************************
Function:       ISO14443ACheckCRCA
Description:    CRC16 解包
Input:          buff byte_count
Output:
Return:
Others:
*************************************************/
unsigned char SerialPort::ISO14443ACheckCRCA(void* buffer, unsigned short byte_count)
{
    unsigned short check_sum = 0x6363;
    unsigned char* data = (unsigned char*)buffer;

    while (byte_count --)
    {
        unsigned char byte = *data++;

        byte ^= (unsigned char)(check_sum & 0x00ff);
        byte ^= byte << 4;

        check_sum = (check_sum >> 8) ^ ((unsigned short)byte << 8) ^ ((unsigned short)byte << 3) ^((unsigned short)byte >> 4);
    }
    //cout << ((data[0] == ((check_sum >> 0) & 0xff)) && (data[1] == ((check_sum >> 8) & 0xff))) << endl;
    return (data[0] == ((check_sum >> 0) & 0xff)) && (data[1] == ((check_sum >> 8) & 0xff));
}
/*************************************************
Function:       ISO14443AAppendCRCA
Description:    CRC16 数据长度检验
Input:          buff
Output:
Return:
Others:
*************************************************/
//bool SerialPort::ISO14443ACheckLen(unsigned  char* buffer)
//{
//    if ((buffer[0] + buffer[1] == 0x7f) && (buffer[0] < COMEMAND_BUFF_LEN -2))
//        return true;
//    else
//        return true;
//}
bool SerialPort::ISO14443ACheckLen(unsigned  char* buffer)
{
    if ((buffer[2] + buffer[3] == 0xff))
        return true;
    else
        return true;
}
/*************************************************
Function:       SendBuff
Description:    发送数据包
Input:          buff length
Output:
Return:         true or false
Others:         bool
*************************************************/
bool SerialPort::SendBuff(char command, char *data, unsigned short length)
{
    unsigned char buffer[COMEMAND_BUFF_LEN]; //50
    buffer[0] = 0x55; // 帧头
    buffer[1] = command; // 命令
    buffer[2] = length; // 数据长度
    buffer[3] = 0xff - length; // 数据长度取反
    memcpy(buffer + HEAD_LEN, data, DATA_LEN); // 拷贝数据   从data中复制DATA_LEN个字节到第一个参数
    ISO14443AAppendCRCA(buffer, length + HEAD_LEN); // CRC16 打包

    if (Write((char*)buffer, length + HEAD_LEN + 2))
        return true;
    else
        return false;
}
/*************************************************
Function:       ReceiveBuff
Description:    读取数据包
Input:          src_buff dst_buff
Output:
Return:         true or false
Others:         bool
*************************************************/
int SerialPort::ReceiveBuff(char *src_buff, char *dst_buff)
{
    //NULL;
    //usleep(20000);
    size_t read_length = Read(src_buff, COM_BUFF_LEN);
   // std::clog<<"llllllllllength "<<read_length<<std::endl;
    if (read_length == 0){return -1;}
    //cout << src_buff[0] <<endl;
    if (src_buff[0] == 0x55)
    {
        if(ISO14443ACheckLen((unsigned char *)(src_buff + HEAD_LEN - 2))){
            //cout<<"receive success"<<endl;
            if (ISO14443ACheckCRCA(src_buff, (unsigned short)(src_buff[2] + HEAD_LEN))){
                //cout<<"receive success"<<endl;
                //cout << (int8_t *)src_buff <<endl;
                mtx_port.lock();
                for (int i = 0; i < DATA_LEN; i++) {
                    dst_buff[i] = src_buff[i];
                }
                mtx_port.unlock();
                int16_t buff[4];

                buff[0] = ((uint16_t)dst_buff[4]<<8)|uint8_t (dst_buff[5]);
                buff[1] = ((uint16_t)dst_buff[6]<<8)|uint8_t (dst_buff[7]);

                double pitch_ptz = double(buff[0]) /1000;
                double yaw_ptz = double (buff[1]) /1000;

                buff[2] =  ((uint16_t)dst_buff[8]<<8)| uint8_t (dst_buff[9]);
                buff[3] = ((uint16_t)dst_buff[10]<<8)| uint8_t (dst_buff[11]);
                double pitch_w = double(buff[2]) / 1000 ;
                double yaw_w = double(buff[3]) / 1000 ;
                uint16_t speed_d =  uint16_t (dst_buff[12])<<8 | uint8_t (dst_buff[13]);

                //std::cout<<"bit8 "<<hex<<uint8_t(dst_buff[8])<<" "<<"bit9"<<hex<<uint8_t(dst_buff[9])<<std::endl;
                receive[0] = 0;
                receive[1] = dst_buff[1];
                receive[2] = pitch_ptz;
                receive[3] = yaw_ptz;
                receive[4] = speed_d;  //50
                receive[5] = pitch_w;
                receive[6] = yaw_w;
//                cout<<"port buff: "
//                    <<" pitch_ptz: "<<pitch_ptz
//                    <<", yaw_ptz: "<<yaw_ptz
//                    <<", speed_d: "<<speed_d
//                    <<", pitch_w: "<<pitch_w  //抬头负，低头正
//                    <<", yaw_w: "<<yaw_w
//                    <<endl;  //顺时针负，逆时针正
                stm.pitch = receive[2];
                stm.yaw = receive[3];
                stm.yaw_w = receive[6];
                stm.pitch_w = receive[5];
                if(receive[4] > 10000.0){
                    stm.bulletSpeed = (receive[4]-10000.0)/100.0;
                }else{
                    stm.bulletSpeed = receive[4]/100.0;
                }
                return 1;
            }else{
                for (int i = 0; i < DATA_LEN; i++) {
                    dst_buff[i] = 0;
                }
                //infor_clear();
                return 0;
            }

        }else{
            for (int i = 0; i < DATA_LEN; i++) {
                dst_buff[i] = 0;
                infor_clear();
            }
            return 0;
        }
    }
    else
    {
        //cout << "SERIAL error1" << endl;
        for (int i = 0; i < DATA_LEN; i++){
            dst_buff[i] = 0;
            //receive[i] = 0;
        }
       // infor_clear();
        return 0;
    }
}


/**********************************
 * @descrip 接收线程
 * @param pFrame
 * @return
 * @date
 */
void port_receive(void){
    std::cout<<"创建线程成功:"<<std::endl;
    //while(!port.PortInit(0, 115200));
    int status = -2;
    while(1){
        status = port.ReceiveBuff(port.buff_l_, port.buff_r_);
        //std::cout<<"ssssssssssss"<<status<<std::endl;
        if(status == -1){
            while(!port.PortInit(0, 115200));
        }
    }
}
