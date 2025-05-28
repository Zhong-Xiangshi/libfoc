#ifndef __LIBFOC_H__
#define __LIBFOC_H__
#include "stdint.h"
#include "libfoc_interface.h"

typedef struct{
    float x;
    float y;
}  vector;

typedef struct{
    //缩放倍数
    float scale;
    float input; //输入值
    float alpha; //低通滤波系数
    float kp;
    float ki;
    float kd;
    float i_max;    //积分限幅
    float i_min;    //积分限幅
    float error;
    float i;
    float last_error;

}  pid_param;

typedef enum{
    FOC_MODE_CURRENT = 0, //力矩(电流)模式
    FOC_MODE_SPEED = 1,   //速度模式
    FOC_MODE_POSITION = 2, //位置模式（三环）
    FOC_MODE_POSITION_TWO = 3, //位置模式（二环）
    FOC_MODE_MAX
} foc_mode;

//电机句柄
typedef struct{

    foc_interface_t driver; // 驱动函数接口
    // 是否已经初始化
    uint8_t init_already;
    // 电机极对数，用来计算电角度，电角度=机械角度*极对数。
    uint8_t pole_pairs;
    // 电机最大占空比
    uint16_t motor_pwm_max;
    // 控制模式
    foc_mode mode;

    // 三相PWM值
    int16_t phase_a, phase_b, phase_c;

    
    //机械角度,初始机械角度,上次机械角度
    float mech_angle,mech_angle_zero,mech_angle_last;
    //电角度（单位弧度）
    float elec_angle_rad;

    //电流环
    float target_iq;
    float iq,id;
    float vq,vd;
    float phase_a_current, phase_b_current, phase_c_current; //三相电流
    pid_param pid_iq,pid_id;
    vector parker_x,parker_y;
    vector current_vec;

    //速度环
    float target_speed;
    float speed;
    pid_param pid_speed;

    //位置环
    float target_position;
    float position_last; // 上次位置
    float position;
    pid_param pid_position;

}  motor;

//foc初始化
int foc_init(motor *motor,foc_interface_t driver,uint8_t pole_pairs,uint16_t motor_pwm_max);

//设置电流环PID
void foc_current_set_pid_param(motor *motor,float scale,float iq_kp,float iq_ki,float id_kp,float id_ki);
//电流环更新，参考调用频率4khz
void foc_current_update(motor *motor);

//设置速度环PID
void foc_speed_set_pid_param(motor *motor,float scale,float alpha,float kp,float ki,float i_max);
//速度环更新，参考调用频率1khz
void foc_speed_update(motor *motor,float interval);

//设置位置环PID
void foc_position_set_pid_param(motor *motor,float scale,float alpha,float kp,float ki,float kd,float imax);
//位置环更新，参考调用频率1khz
void foc_position_update(motor *motor);
void foc_position_update_two(motor *motor);

//设置控制模式
void foc_set_mode(motor *motor,foc_mode mode);
//设置电机控制目标值
void foc_set_target(motor *motor,float target);

/*
    演示函数，用于调试
*/
void foc_demo_1(motor *motor);
void foc_demo_2(motor *motor);
void foc_demo_31(motor *motor);
void foc_demo_32(motor *motor,uint8_t motor_en);
void foc_demo_4(motor *motor,uint8_t motor_en);
void foc_demo_5(motor *motor,uint8_t motor_en,float target_iq);
void foc_demo_6(motor *motor,uint8_t motor_en);

#endif
