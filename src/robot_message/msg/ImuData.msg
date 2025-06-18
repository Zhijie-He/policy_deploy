# ImuData.msg

uint16 header
uint16 sequence

float32[3] accelerometer   # 加速度 (x, y, z)（单位：m/s^2）
float32[3] gyroscope       # 陀螺仪 (x, y, z)（单位：rad/s）
float32[4] quaternion      # 四元数 (w, x, y, z)
float32[3] rpy             # 欧拉角 (roll, pitch, yaw)
int32 temperature          # IMU 温度
uint32 timestamp           # 相对时间戳，单位为10微秒
