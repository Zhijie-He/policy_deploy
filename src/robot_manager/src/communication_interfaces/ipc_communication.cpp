#include "ipc_communication.h"

/**
  * @brief  全局电机命令变量初始化函数
  * @param  type  
  * @retval null
  * @usage  
  */
void IpcCommunication::init(enum IpcType type, unsigned int src_port, const char * src_path){
    if(type == IpcType::IPC_UDP)
    {
        struct sockaddr_in * in_addr = (struct sockaddr_in *)&udp_local_addr;

        // 创建UDP数据报套接字
        if ((ipc_udp_fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
            perror("socket");
            exit(EXIT_FAILURE);
        }

        std::memset(in_addr, 0, sizeof(struct sockaddr_in));
        in_addr->sin_family = AF_INET;
        in_addr->sin_port = htons(src_port);
        in_addr->sin_addr.s_addr = INADDR_ANY;

        //绑定地址
        if (bind(ipc_udp_fd, (const struct sockaddr*)in_addr, sizeof(struct sockaddr_in)) < 0) {
            perror("bind failed");
            close(ipc_udp_fd);
            exit(EXIT_FAILURE);
        }
    }
    else if(type == IpcType::IPC_UNIX)
    {
        struct sockaddr_un * un_addr = (struct sockaddr_un *)&unix_local_addr;

        /* 创建UNIX域socket,使用数据报传输 */
        if ((ipc_unix_fd = socket(AF_UNIX, SOCK_DGRAM, 0)) == -1) {
            perror("socket error");
            pthread_exit(NULL);
        }

        /* 设置套接字地址 */
        std::memset(un_addr, 0, sizeof(struct sockaddr_un));
        un_addr->sun_family = AF_UNIX;
        std::strncpy(un_addr->sun_path, src_path, sizeof(un_addr->sun_path) - 1);

        /* 删除原有套接字文件 */
        unlink(src_path);

        /* 绑定套接字 */
        if (bind(ipc_unix_fd, (struct sockaddr *)un_addr, sizeof(struct sockaddr_un)) == -1) {
            perror("bind error");
            close(ipc_unix_fd);
            pthread_exit(NULL);
        }

    }
}

void IpcCommunication::initDestAddress(IpcType type, void* address, const std::string& dest_ip, unsigned int dest_port, const std::string& dest_path) {
    if (type == IpcType::IPC_UDP) {
        auto* udp_addr = reinterpret_cast<struct sockaddr_in*>(address);
        std::memset(udp_addr, 0, sizeof(struct sockaddr_in));
        udp_addr->sin_family = AF_INET;
        udp_addr->sin_port = htons(dest_port);
        udp_addr->sin_addr.s_addr = inet_addr(dest_ip.c_str());
    } else if (type == IpcType::IPC_UNIX) {
        auto* unix_addr = reinterpret_cast<struct sockaddr_un*>(address);
        std::memset(unix_addr, 0, sizeof(struct sockaddr_un));
        unix_addr->sun_family = AF_UNIX;
        std::strncpy(unix_addr->sun_path, dest_path.c_str(), sizeof(unix_addr->sun_path) - 1);
    }
}

void IpcCommunication::sendData(IpcType type, void* dest_addr, unsigned char* data_buf, unsigned int data_len) {
    unsigned int addr_len;

    if (type == IpcType::IPC_UDP) {
        addr_len = sizeof(struct sockaddr_in);

        // 发送数据
        if (sendto(ipc_udp_fd, data_buf, data_len, 0, reinterpret_cast<struct sockaddr*>(dest_addr), addr_len) == -1) {
            perror("sendto");
        }

    } else if (type == IpcType::IPC_UNIX) {
        addr_len = sizeof(struct sockaddr_un);

        // 发送数据
        if (sendto(ipc_unix_fd, data_buf, data_len, 0, reinterpret_cast<struct sockaddr*>(dest_addr), addr_len) == -1) {
            perror("sendto");
        }
    }

}

void IpcCommunication::IpcReceiveDispose(void) {
    unsigned char buffer[1500];

    while (true) {
        ssize_t recv_len = recvfrom(ipc_unix_fd, buffer, sizeof(buffer), 0, NULL, NULL);

        if(recv_len < 0)
        {
            ;;
        }

    }

    pthread_exit(nullptr);
}


IpcCommunication::~IpcCommunication() {
    if (ipc_udp_fd != -1) {
        close(ipc_udp_fd);
    }

    if (ipc_unix_fd != -1) {
        close(ipc_unix_fd);
    }
}

