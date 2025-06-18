# JointControlGroup.msg
uint16 handle              # 消息句柄
uint16 sequence            # 序列号
uint64 timestamp           # 相对时间戳，单位为10微秒
MotorCmd[14] cmd      # 14个关节的控制命令数据（数组形式）
