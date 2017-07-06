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
    GCHECK=1024
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
}Direction;


// 更新/读取控制变量
void update_g_x(param x);
param get_g_x();

// 启动角度、极限角度设定
void set_g_start_angle(double angle);
double get_g_start_angle();
void set_g_left_angle(double angle);
double get_g_left_angle();
void set_g_right_angle(double angle);
double get_g_right_angle();

#endif // GLOBAL_SETTING_H
