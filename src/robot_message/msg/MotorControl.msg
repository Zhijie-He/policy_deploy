# MotorControl.msg
uint16 handle              # 消息句柄
uint16 sequence            # 序列号
MotorCmd[14] cmd      # 14个电机状态数据（数组形式）