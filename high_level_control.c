#include "high_level_control.h"
#include "am335x_setting.h"


// 目标角度
volatile double destination_angle = 0;
// 用于变量赋值的互斥
pthread_mutex_t mutex_dst_angle = PTHREAD_MUTEX_INITIALIZER;


// 更新目标角度
void update_destination_angle(double angle)
{
    pthread_mutex_lock(&mutex_dst_angle);
    destination_angle = angle;
    pthread_mutex_unlock(&mutex_dst_angle);
}
// 获取目标角度
double get_destination_angle()
{
    pthread_mutex_lock(&mutex_dst_angle);
    double angle = destination_angle;
    pthread_mutex_unlock(&mutex_dst_angle);
    return angle;
}


// 取消当前任务
int task_cancel()
{
    int ret;

    ret = positioning_cancel_on();
    if(-1 == ret) return -1;
    ret = positioning_cancel_off();
    if(-1 == ret) return -1;

    return 0;
}

// 急停
int force_stop()
{
    int ret;

    ret = forced_stop_on();
    if(-1 == ret) return -1;
    ret = forced_stop_off();
    if(-1 == ret) return -1;

    return 0;
}

// 转到指定角度
int goto_point(double angle)
{
    //// debug语句
    //printf("goto_point %f\n", angle);
    //double test_angle = get_encoder_angle();
    //printf("get encoder angle %f\n", test_angle);
    //double test_start = get_g_start_angle();
    //printf("zero start angle %d\n", test_start);

    if(angle < get_g_left_angle()) angle = get_g_left_angle();
    if(angle > get_g_right_angle()) angle = get_g_right_angle();

    int ret;
    
    //更新目标角度
    update_destination_angle(angle);

    double actual_angle = angle - get_encoder_angle();
    actual_angle *= TRANSMISSION_RATIO;
    ret = run_inc_angle(actual_angle);

    return ret;
}

// 左转
int goto_left()
{
    int ret;

    //更新目标角度
    double angle = get_g_left_angle();
    update_destination_angle(angle);

    double actual_angle = angle - get_encoder_angle();
    actual_angle *= TRANSMISSION_RATIO;
    ret = run_inc_angle(actual_angle);

    return ret;
}

// 右转
int goto_right()
{
    int ret;

    //更新目标角度
    double angle = get_g_right_angle();
    update_destination_angle(angle);
    
    double actual_angle = angle - get_encoder_angle();
    actual_angle *= TRANSMISSION_RATIO;
    ret = run_inc_angle(actual_angle);

    return ret;
}

// 速度设定
// 输入: 度/秒
int set_speed_value(double speed)
{
	int ret;

	uint32_t actual_speed = speed*60*100*TRANSMISSION_RATIO/360;
	set_cruise_speed(actual_speed);
	ret = task_cancel();
	if(-1 == ret) return -1;
	ret = send_cruise_speed();

	return ret;
}
// 加速时间设定
// 输入: 0.1ms
int set_acce_value(double acce_time)
{
	int ret;

	set_imme_acceleration_time(acce_time);
	ret = task_cancel();
	if(-1 == ret) return -1;
	ret = send_imme_acceleration_time();

	return ret;
}
// 减速时间设定
// 输入: 0.1ms
int set_dece_value(double dece_time)
{
	int ret;

	set_imme_deceleration_time(dece_time);
	ret = task_cancel();
	if(-1 == ret) return -1;
	ret = send_imme_deceleration_time();

	return ret;
}

//// 自检操作
int check_motion()
{
    int ret;

    // 速度设定
    ret = task_cancel();
    if(-1 == ret) return -1;
    ret = send_check_speed();
    if(-1 == ret) return -1;
    ret = send_check_acce_time();
    if(-1 == ret) return -1;
    ret = send_check_dece_time();
    if(-1 == ret) return -1;

    ret = goto_point(0);
    if(-1 == ret) return -1;
    while(!is_INP()){
	usleep(10000); // 10ms
    }

    ret = goto_left();
    if(-1 == ret) return -1;
    while(!is_INP()){
	usleep(10000); // 10ms
    }

    ret = goto_right();
    if(-1 == ret) return -1;
    while(!is_INP()){
	usleep(10000); // 10ms
    }

    ret = goto_point(0);
    if(-1 == ret) return -1;

    //速度复原
    ret = send_cruise_speed();
    if(-1 == ret) return -1;
    ret = send_imme_acceleration_time();
    if(-1 == ret) return -1;
    ret = send_imme_deceleration_time();
    if(-1 == ret) return -1;

    return ret;
}

