buzzer_task 
烧录成功时响三声作为烧录成功标志

led_task 
常态下驱动流水灯，便于观察任务是否阻塞

laser_task 
在正常工作时，每毫秒将距离以单位cm存储到变量distance中，异常返回0xFFFF。

plotter_task
将需求的数据通过上位机打印。

uart_task 
创建和遥控器的通讯，创建和裁判系统通讯，进行huart的回调，为避免重定义，如果有其他task需要调用HAL_UART_ErrorCallback或HAL_UARTEx_RxEventCallback，向此文件中添加分支

imu_task
读取c板姿态数据，后续调用，只读参数参考bmi088.hpp及mahony.hpp
imu方向配置需要根据实际情况调整

can_task
创建了can线的实例，其中云台和底盘元件各自占用一条can线，向电机输出值的cmd和write指令需要在其他处完成。

一、底盘代码运行线路
DBUS遥控器
    ↓
遥控器状态机
    ↓
底盘速度 vx、vy、wz
    ↓
麦轮解算得到四个目标轮速
    ↓
PID速度控制
    ↓
功率限制
    ↓
CAN发送给四个M3508

二、底盘代码说明
1、文件职责
chassis_controller.cpp：底盘状态机、速度指令、轮速控制、PID和功率限制。
chassis_controller.hpp：底盘控制器对外提供的数据结构和接口。
chassis_task.cpp：连接DBUS、CAN反馈和底盘控制器，按任务周期运行控制流程。
轮速解算、反馈解析、PID计算和CAN数据处理均使用已有中间件。

2、状态机和安全处理
状态流程：DISABLED → ARMING → ENABLED。
遥控器掉线、右拨杆为DOWN、摇杆未回中或电机反馈超时，都不会输出有效力矩。
失能时清除PID状态，并将四个电机力矩清零。
UP对应自动模式。

3、主要参数
底盘几何参数、M3508 ID、减速比、摇杆死区、速度上限和轮速上限位于：
applications/chassis_controller.cpp
PID参数和输出限幅也在该文件中，需要结合实车调试。

4、PID调试
调试顺序为 P → I → D：
先确认四个电机反馈方向、目标轮速和实际轮速单位正确，再逐步调整 P。
出现持续振荡时降低 P；稳定但有较小速度误差时再少量增加 I；D项最后根据反馈噪声决定是否启用。
PID输出上限和积分输出上限应先保持保守，避免电机突然产生过大力矩。

5、功率限制
预测公式为：
Pin = ∑(τ × ω) + K1 × ∑(τ²) + K2 × ∑(ω²) + K3

当前使用输出轴参考系，减速比为13.7。功率参数暂定为：
K1 = 2.8；K2 = 0.01；K3 = 4.0；功率限制 = 80 W。
K1、K2、K3、功率上限以及参考系都需要结合实测确认。

若预测功率未超限，K = 1；超限时求满足功率限制的最大K，并限制在0到1之间，四个电机的力矩统一乘K后再发送。
功率限制代码位于 applications/chassis_controller.cpp。

云台代码进程：创建了云台电机，设置了位置环pid，以及角度限幅，紧急失能，设置target_angle.yaw即可输出

三、云台自动折叠
gimbal_fold_task.cpp：独立的云台折叠任务。
GY-56 通过 applications/laser_task.cpp 发布 distance，检测到前方距离小于阈值后开始折叠。
折叠任务只修改 gimbal::target_angle.fold，位置 PID 和 CAN 输出仍由 gimbal_task.cpp 完成。
折叠到位后读取 auto_straight 的已行驶距离，超过设定行程后复位云台。
auto_straight 只由 chassis_task.cpp 调用 update()，折叠任务只读取 active() 和 travelled_m()，避免重复累加行程。
折叠角度偏移、入口距离、复位行程和动作超时位于 applications/gimbal_fold_task.cpp，实机调试时按结构尺寸和反馈调整。
调试用的云台轨迹追踪任务，在使能情况下左拨杆上档触发。
