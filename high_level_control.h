#ifndef HIGH_LEVEL_CONTROL_H
#define HIGH_LEVEL_CONTROL_H

#include "alpha_motion_control.h"


// 更新/获取目标角度
void update_destination_angle(double angle);
double get_destination_angle();


// 取消当前任务
int task_cancel();
// 急停
int force_stop();
// 转到指定角度
int goto_point(double angle);
// 左转
int goto_left();
// 右转
int goto_right();
// 速度设定
// 输入: 度/秒
int set_speed_value(double speed);
// 加速时间设定
// 输入: 0.1ms
int set_acce_value(double acce_time);
// 减速时间设定
// 输入: 0.1ms
int set_dece_value(double dece_time);
//// 自检操作
int check_motion();


#endif // HIGH_LEVEL_CONTROL_H
