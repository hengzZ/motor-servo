# motor-servo

## module

- alpha parameter configure
    * alpha_setting.h
    * alpha_setting.c
- alpha motion control
    * alpha_motion_control.h
    * alpha_motion_control.c
    * alpha_motion_test.c
- encoder listening
    * am335x_setting.h
    * am335x_setting.c
- console listening
    * cmdparser.c

## API
|           API         |         note       |
|-----------------------|--------------------|
| open_dev()            | 读取参数，确认配置 |
| close_dev()           |                    |
| set_acce_time()       |    (ms)            |
| set_dece_time()       |    (ms)            |
| set_speed()           |    (degree/s)      |
| set_max_left_point()  |    (degree)        |
| set_max_right_point() |    (degree)        |
| run_to_angle()        |    (degree)        |
| run_to_direction()    |    left/right      |
| stop_run()            |                    |
| cancel_task()         |                    |
| check()               |    开机检验        |
| get_status()          |    [角度][状态]    |

note: 1. 超时时间设置
      2. 电机 - positions/circle
      3. 编码器 - positions/circle
      4. 转速比 - motor/encoder
      5. 状态: 最左、最右、In Position、运行中、停止中、故障
                                                                                        
## 
