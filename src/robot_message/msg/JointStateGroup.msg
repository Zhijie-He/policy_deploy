# JointStateGroup.msg
uint16 handle              # 消息句柄
uint16 sequence            # 序列号
uint64 timestamp           # 相对时间戳，单位为10微秒
MotorState[14] state      # 14个关节状态数据（数组形式）
