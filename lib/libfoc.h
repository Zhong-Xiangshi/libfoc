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
void foc_position_update(motor *motor); //三环版本
void foc_position_update_two(motor *motor); //双环版本

//设置控制模式
void foc_set_mode(motor *motor,foc_mode mode);
//设置电机控制目标值
void foc_set_target(motor *motor,float target);

//获取电机力矩（IQ，和力矩成正比，需要手动乘以常数）
float foc_get_torque(motor *motor);
//获取电机速度
float foc_get_speed(motor *motor);
//获取电机位置
float foc_get_position(motor *motor);

/*
    调试函数
    debug functions
*/

/*
    Obtain motor pole pairs. Output at pwm_max (Note: control this value to prevent motor overheating or damage), 
    fixed at 0 electrical angle. Manually rotate the motor one full circle, the number of intervals required is the motor's pole pairs.
    获得电机极对数。以pwm_max输出（注意控制此值，否则电机发烫或烧坏电机），固定为0电角度，此时手动转动电机，转一圈需要用的间隔数即电机的极对数
*/
void foc_demo_0(motor *motor);  

/*
    Rotate 90 electrical degrees clockwise every 1s at pwm_max. Swap phase sequence if counterclockwise rotation is needed.
    每隔1s以pwm_max顺时针转动90电角度。如果为逆时针需要换线序
*/
void foc_demo_1(motor *motor);  

/*
    Verify pole pair correctness and angle increment direction. Depends on angle sensor/pole pairs.
    Continuous clockwise rotation at fixed 90° voltage vector. Rotation fails if pole pairs incorrect. Angle increases clockwise.
    用于测试极对数是否正确，角度增量方向是否正常。依赖角度传感器、极对数。以90°固定电压矢量连续顺时针转动。极对数不对则无法连续转动。顺时针角度增加。
*/
void foc_demo_2(motor *motor);

/*
    Debug current direction. Energize ABC phases clockwise every 1s and display three-phase currents.
    Normal: A phase current positive when A energized, same for B/C.
    用于调试电流方向。每隔1s顺时针通电ABC三相,并显示三相电流。正常情况：A通电A相电流正，B通电B相电流正，C通电C相电流正
*/
void foc_demo_31(motor *motor); 

/*
    Control motor clockwise at fixed 90° voltage vector. Depends on angle sensor/current sensor/pole pairs. Displays three-phase currents.
    依赖角度传感器、电流传感器、极对数。以90°固定电压矢量控制电机顺时针转动,显示三相电流
*/
void foc_demo_32(motor *motor,uint8_t motor_en);    

/*
    Control motor at fixed 90° voltage vector. Displays IQ and ID currents. 
    Normal: ID near 0 when stalled. Depends on angle/current sensors/pole pairs.
    依赖角度传感器、电流传感器、极对数。以90°固定电压矢量控制电机，显示IQ和ID电流。正常情况堵转ID接近0
*/
void foc_demo_4(motor *motor,uint8_t motor_en); 

/*
    Torque control. Displays IQ, ID, VQ, VD. Depends on angle/current sensors/pole pairs.
    依赖角度传感器、电流传感器、极对数。力矩控制，显示IQ、ID、VQ、VD
*/
void foc_demo_5(motor *motor,uint8_t motor_en,float target_iq); 

/*
    Based on demo4 with IQ/ID low-pass filtering added. Displays IQ and ID currents.
    demo4为原型添加IQ、ID低通滤波，显示IQ和ID电流
*/
void foc_demo_6(motor *motor,uint8_t motor_en); 

#endif
