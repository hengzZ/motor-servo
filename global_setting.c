#include "global_setting.h"


// 全局控制参数变量
volatile param g_x;
// 全局控制状态变量
volatile CtrlStatus g_ctrl_status = FREE;

// 启动位置角度
volatile double g_start_angle;
// 极限位置角度
volatile double g_left_angle;
volatile double g_right_angle;


// 变量赋值时的互斥
pthread_mutex_t global_mutex = PTHREAD_MUTEX_INITIALIZER;


// 更新/读取控制变量
void update_g_x(param x)
{
    pthread_mutex_lock(&global_mutex);
    g_x = x;
    pthread_mutex_unlock(&global_mutex);
}
param get_g_x()
{
    pthread_mutex_lock(&global_mutex);
    param temp_x = g_x;
    pthread_mutex_unlock(&global_mutex);
    return temp_x;
}

// 更新/读取控制状态
void update_g_ctrl_status(CtrlStatus status)
{
    pthread_mutex_lock(&global_mutex);
    g_ctrl_status = status;
    pthread_mutex_unlock(&global_mutex);
}
CtrlStatus get_g_ctrl_status()
{
    pthread_mutex_lock(&global_mutex);
    CtrlStatus status = g_ctrl_status;
    pthread_mutex_unlock(&global_mutex);
    return status;
}

// 启动角度、极限角度设定
void set_g_start_angle(double angle)
{
    pthread_mutex_lock(&global_mutex);
    g_start_angle = angle;
    pthread_mutex_unlock(&global_mutex);
}
double get_g_start_angle()
{
    pthread_mutex_lock(&global_mutex);
    double angle = g_start_angle;
    pthread_mutex_unlock(&global_mutex);
    return angle;
}

void set_g_left_angle(double angle)
{
	if(angle < -90) angle = -90.0;
	if(angle > 90) angle = 90.0;
    pthread_mutex_lock(&global_mutex);
    g_left_angle = angle;
    pthread_mutex_unlock(&global_mutex);
}
double get_g_left_angle()
{
    pthread_mutex_lock(&global_mutex);
    double angle = g_left_angle;
    pthread_mutex_unlock(&global_mutex);
    return angle;
}

void set_g_right_angle(double angle)
{
	if(angle < -90) angle = -90.0;
	if(angle > 90) angle = 90.0;
    pthread_mutex_lock(&global_mutex);
    g_right_angle = angle;
    pthread_mutex_unlock(&global_mutex);
}
double get_g_right_angle()
{
    pthread_mutex_lock(&global_mutex);
    double angle = g_right_angle;
    pthread_mutex_unlock(&global_mutex);
    return angle;
}

