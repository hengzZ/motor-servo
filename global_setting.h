#ifndef GLOBAL_SETTING_H
#define GLOBAL_SETTING_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>


// 用于控制的信号编码
typedef enum{
    GRIGHT=1,
    GLEFT=2,
    GPOINT=4,
    GPST_CANCEL=8,
    GEMG=16,
    GSPEED=32,
    GACCE_TIME=64,
    GDECE_TIME=128,
    GMAX_POINT=256,
    GSTATUS=512,
    GCHECK=1024,
    GERROR_MSG=2048,
    GALARM_RST=4096
} GFLAGS;
// 控制信号的数据对象格式
typedef struct {
    GFLAGS cmd;
    double v[2];
}param;

// 运动方向
typedef enum{
    NOTMOVE=1,
    LEFTMOVE=2,
    RIGHTMOVE=3
}MovementStatus;

// 控制器状态
typedef enum{
    FREE=0,
    LOCATING=1,
    LEFTORRIGHT=2,
    FINISH=3,
    ERROOR=4,
}CtrlStatus;


// 更新/读取控制变量
void update_g_x(param x);
param get_g_x();
// 更新/读取控制状态
void update_g_ctrl_status(CtrlStatus status);
CtrlStatus get_g_ctrl_status();
// 设置退出主程序，释放伺服开关
void set_stop(int mode);
int get_stop();

// 启动角度、极限角度设定
void set_g_start_angle(double angle);
double get_g_start_angle();
void set_g_left_angle(double angle);
double get_g_left_angle();
void set_g_right_angle(double angle);
double get_g_right_angle();
// 更新/获取目标角度
void update_destination_angle(double angle);
double get_destination_angle();
// 伺服电机零速度更新/获取
void set_motor_zero_speed();
int get_motor_zero_speed();
// 用于运动方面的反转，安装时用于调整默认方向
void set_anticlockwise(int mode);
int get_anticlockwise();

#endif // GLOBAL_SETTING_H
