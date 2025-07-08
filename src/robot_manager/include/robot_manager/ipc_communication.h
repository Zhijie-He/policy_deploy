#pragma once

#include <iostream>
#include <string>
#include <cstring>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <functional>
#include <thread> 


/*******************************************************************************
 * 枚举定义
 ******************************************************************************/
enum class DATA_TYPE {
    UNKNOWN = 0,
    MOTOR_CMD_TYPE,
    MOTOR_STATE_TYPE,
    IMU_DATA_TYPE,
    BMS_STATE,
    CONTROL_DATA,
    ARM_LEFT_CMD,
    ARM_RIGHT_CMD,
    ARM_LEFT_STATE,
    ARM_RIGHT_STATE,
    MOTOR_ABNORMAL_TYPE,
    MOTOR_INFO_TYPE,
    JOINT_DATA_TYPE,
    MAX_NUM
};

enum class IpcType {
    IPC_UNIX = 0,
    IPC_UDP
};



/*******************************************************************************
 * Class definition
 ******************************************************************************/
class IpcCommunication {
public:		//公开成员
    // 构造函数，传递回调函数并创建线程
    IpcCommunication() = default;

    ~IpcCommunication();

    void init(enum IpcType type, unsigned int src_port, const char * src_path);
    void initDestAddress(IpcType type, void* address, const std::string& dest_ip, unsigned int dest_port, const std::string& dest_path);
    void sendData(IpcType type, void* dest_addr, unsigned char* data_buf, unsigned int data_len);

private:	//私有成员，只能在类的内部访问

    struct sockaddr_in udp_local_addr;
    struct sockaddr_un unix_local_addr;

protected:	//受保护成员，只能在类或派生类调用

    int ipc_udp_fd = -1, ipc_unix_fd = -1;
	
    virtual void IpcReceiveDispose(void);
};

