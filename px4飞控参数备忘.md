# PX4飞控参数备忘
## 参考网站
https://micoair.cn/docs/

## 飞控、传感器型号与接口
### 型号信息
* 飞控：微空NxtPX4v2
* 光流+测距：MTF-01
* GPS：MG-F10
* 摄像头：Intel D340

### 接口对应关系
UARTn在板子上有标出
TELEM1 -> UART2  
TELEM2 -> UART4  
TELEM3-> UART7 (ESC Telemetry)  
TELEM4 -> UART8  
GPS1 -> UART1  
GPS2 -> UART3  
RC -> UART5  

## 参数调整
### 光流
MAV_1_CONFIG TELEM n  
MAV_1_MODE Normal  
SER_TELn_BAUD 115200 8N1  
EKF2_OF_CTRL Enabled  
EKF2_RNG_CTRL Enabled  
EKF2_HGT_REF Range sensor  
