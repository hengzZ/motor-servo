#include "global_setting.h"


// 全局控制参数变量
volatile param g_x;
// 全局控制状态变量
volatile CtrlStatus g_ctrl_status = NONE;
// 退出主程序，释放伺服开/关
volatile int g_stop = 0;
// 用于运动方面的反转，安装时用于调整默认方向
volatile int anticlockwise;

// 启动位置角度
static double g_start_angle;
// 极限位置角度
static double g_left_angle;
static double g_right_angle;
// 目标角度
volatile double destination_angle = 0;
// 伺服电机零速度标志
volatile int motor_zero_speed=1;

// 变量赋值时的互斥
pthread_mutex_t global_mutex = PTHREAD_MUTEX_INITIALIZER;


// 伺服电机零速度更新/获取
void set_motor_zero_speed()
{
    pthread_mutex_lock(&global_mutex);
    motor_zero_speed = 1;
    pthread_mutex_unlock(&global_mutex);
}
int get_motor_zero_speed()
{
    pthread_mutex_lock(&global_mutex);
    int speed = motor_zero_speed;
    pthread_mutex_unlock(&global_mutex);
    return speed;
}

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

// 设置退出主程序，释放伺服开关
void set_stop(int mode)
{
    pthread_mutex_lock(&global_mutex);
    g_stop = mode;
    pthread_mutex_unlock(&global_mutex);
}
int get_stop()
{
    pthread_mutex_lock(&global_mutex);
    int mode = g_stop;
    pthread_mutex_unlock(&global_mutex);
    return mode;
}

// 启动角度、极限角度设定
void set_g_start_angle(double angle)
{
    //printf("set: angle %f\n", angle);
    g_start_angle = angle;
    //printf("set: g_start_angle %f\n", g_start_angle);
}
double get_g_start_angle()
{
    //printf("get: g_start_angle %f\n", g_start_angle);
    double angle = g_start_angle;
    //printf("get: angle %f\n", angle);
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

// 更新目标角度
void update_destination_angle(double angle)
{
    pthread_mutex_lock(&global_mutex);
    destination_angle = angle;
    pthread_mutex_unlock(&global_mutex);
}
// 获取目标角度
double get_destination_angle()
{
    pthread_mutex_lock(&global_mutex);
    double angle = destination_angle;
    pthread_mutex_unlock(&global_mutex);
    return angle;
}

// 获取当前的坐标方向标记
void set_anticlockwise(int mode)
{
    pthread_mutex_lock(&global_mutex);
    anticlockwise = mode;
    pthread_mutex_unlock(&global_mutex);
}
int get_anticlockwise()
{
    pthread_mutex_lock(&global_mutex);
    int mode = anticlockwise;
    pthread_mutex_unlock(&global_mutex);
    return mode;
}

