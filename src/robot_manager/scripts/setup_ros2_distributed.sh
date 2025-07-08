#!/bin/bash

# 设置 ROS2 分布式环境的脚本
# 使用方法：./setup_ros2_distributed.sh <ROS_DOMAIN_ID> <其他主机的IP1> <其他主机的IP2> ...

# 检查参数
if [ "$#" -lt 2 ]; then
    echo "使用方法: $0 <ROS_DOMAIN_ID> <其他主机的IP1> <其他主机的IP2> ..."
    echo "示例: $0 42 192.168.1.2 192.168.1.3"
    exit 1
fi

# 获取参数
ROS_DOMAIN_ID=$1
shift 1  # 移除第一个参数，剩下的都是其他主机的 IP 地址
OTHER_HOST_IPS=("$@")  # 将所有其他主机的 IP 地址存入数组

# 设置 ROS_DOMAIN_ID
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
echo "已设置 ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# 生成 Fast DDS 配置文件
FASTDDS_CONFIG_FILE="/tmp/fastdds_config.xml"
cat <<EOL > $FASTDDS_CONFIG_FILE
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile">
        <rtps>
            <builtin>
                <initialPeersList>
EOL

# 添加其他主机的 IP 地址到配置文件中
for ip in "${OTHER_HOST_IPS[@]}"; do
    cat <<EOL >> $FASTDDS_CONFIG_FILE
                    <locator>
                        <udpv4>
                            <address>$ip</address>
                        </udpv4>
                    </locator>
EOL
done

cat <<EOL >> $FASTDDS_CONFIG_FILE
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>
EOL

echo "Fast DDS 配置文件已生成: $FASTDDS_CONFIG_FILE"

# 设置 Fast DDS 配置文件路径
export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_CONFIG_FILE
echo "已设置 FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_CONFIG_FILE"

# 提示完成
echo "环境变量和 Fast DDS 配置文件配置完成！"
echo "请确保所有主机在同一个网络中，并且可以相互访问。"
