# libfoc  
An embedded FOC library  
嵌入式 FOC 库  

## ⚠️ 安全警告 / Safety Warnings
电机控制可能造成人身伤害或设备损坏  
Motor control may cause personal injury or equipment damage

使用前必须进行安全风险评估  
Safety risk assessment must be performed before use

严禁用于生命支持系统等关键场景  
Strictly prohibited for use in life-support systems or other critical applications

## Driver functions to be implemented (需要实现的驱动函数)  
```C
typedef struct{
    /// @brief Initialize PWM, encoder, current sensing  
    /// @brief 初始化PWM、编码器、电流传感  
    void (*foc_driver_init)(void);

    /// @brief Set duty cycle for three-phase PWM waveforms. Recommended PWM frequency: 20kHZ-30kHZ  
    /// @brief 设置电机三个相PWM波形的占空比，建议PWM频率：20kHZ-30kHZ  
    void (*foc_motor_set_phase)(uint16_t phase_a, uint16_t phase_b, uint16_t phase_c);

    /// @brief Motor enable function  
    /// @brief 电机使能函数  
    void (*foc_motor_enable)(uint8_t enable);

    /// @brief Get mechanical angle  
    /// @brief 获得机械角度  
    void (*foc_get_mech_angle)(float *angle);

    /// @brief Get three-phase current values in Amperes. Positive when outputting (flowing to motor)  
    /// @brief 获得3个相的电流，单位1A，输出(流向电机)为正  
    void (*foc_get_phase_current)(float *phase_a, float *phase_b, float *phase_c);
} foc_interface_t;
```

## How to Use (如何使用)  
### Torque Control Example (力矩控制示例)  
```C
#include "libfoc.h"
motor ms;
// Implement motor driver functions  
// 实现电机驱动函数  
foc_interface_t foc_driver = {
    .foc_driver_init = foc_driver_init,
    .foc_motor_set_phase = foc_motor_set_phase,
    .foc_motor_enable = foc_motor_enable,
    .foc_get_mech_angle = foc_get_mech_angle,
    .foc_get_phase_current = foc_get_phase_current,
};

int main(void){
    ...
    // 1. Initialize motor driver, pole pairs, PWM_MAX  
    // 1.初始化电机驱动、极对数、PWM_MAX  
    foc_init(&ms,foc_driver,7,100);
    
    // 2. Set current loop PID parameters: Iq_P, Iq_I, Id_P, Id_I  
    // 2.设置电流环PID参数：Iq_P、Iq_I、Id_P、Id_I  
    foc_current_set_pid_param(&ms,1,25, 7, 25, 7);
    
    // 3. Set mode to current loop  
    // 3.设置模式为电流环  
    foc_set_mode(&ms,FOC_MODE_CURRENT);
    
    // 4. Set target value to 0.2A  
    // 4.设置目标0.2A  
    foc_set_target(&ms,0.2f);
    ...
    while(1){
        // 5. Update current loop  
        // 5.更新电流环，建议在定时器中断调用  
        foc_current_update(&ms);
    }
}
```