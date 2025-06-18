# MotorCmd.msg
uint32 mode         # 工作模式
float32 pos         # 当前电机位置
float32 w           # 当前实际电机速度
float32 t           # 当前实际电机输出力矩
float32 kp          # 位置刚度
float32 kd          # 速度刚度