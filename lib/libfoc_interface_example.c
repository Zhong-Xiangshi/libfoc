#include "libfoc_interface.h"

// --------------------------------用户实现 start---------------------------------------------

#include "tim.h"
#include "main.h"
#include "driver_as5600_basic.h"
#include "stdarg.h"
#include "adc.h"
#include "i2c.h"

uint16_t adc_value[3]; // ADC采样值
uint16_t adc_value_init[3];
extern uint16_t adc_value_init_count;



/// @brief 初始化PWM、编码器、电流传感
/// @param  
void foc_driver_init(void)
{
    
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);

    as5600_basic_init();

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, 3); // 启动ADC采样
    HAL_TIM_Base_Start_IT(&htim4); // 启动速度环和位置环定时器中断
    while(adc_value_init_count!=0)HAL_Delay(10); // 等待ADC采样中位值结束
    
}
/// @brief 电机使能函数
/// @param enable 1使能，0禁用
void foc_motor_enable(uint8_t enable){
    if(enable){
        HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET); // 使能电机驱动
    }else{
        HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET); // 禁用电机驱动
    }
}

/// @brief 设置电机三个相PWM波形的占空比，输入范围0~motor->motor_pwm_max
/// @param phase_a 
/// @param phase_b 
/// @param phase_c 
void foc_motor_set_phase(uint16_t phase_a, uint16_t phase_b, uint16_t phase_c)
{
    
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, phase_a);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, phase_b);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, phase_c);
    
}

// uint8_t as5600_buf[2];

/// @brief 获得机械角度。
/// @param angle 角度，顺时针增加，单位1°
void foc_get_mech_angle(float *angle)
{
    
    //读取上次角度，并开始异步读取下次角度，这样有利于增加电流环频率。调试时建议改为同步读取。
    uint16_t angle_raw = (uint16_t)(((as5600_buf[0] >> 0) & 0xF) << 8) | as5600_buf[1];
    *angle = (float)(angle_raw) * (360.0f / 4096.0f);
    HAL_I2C_Mem_Read_IT(&hi2c1, 0x6C, 0x0C, I2C_MEMADD_SIZE_8BIT, as5600_buf, 2);
    
}

/// @brief 获得3个相电流，单位安培，输出(流向电机)为正
/// @param phase_a
/// @param phase_b 
/// @param phase_c 
void foc_get_phase_current(float *phase_a, float *phase_b, float *phase_c){
    
    const float scale_factor =3.3f/4096.0f*1000.0f/40.0f/20.0f;
    *phase_b=(adc_value[0]-adc_value_init[0])*scale_factor;
    *phase_c=(adc_value[1]-adc_value_init[1])*scale_factor;
    *phase_a=-*phase_b-*phase_c;
    
}

foc_interface_t foc_driver = {
    .foc_driver_init = foc_driver_init,
    .foc_motor_set_phase = foc_motor_set_phase,
    .foc_motor_enable = foc_motor_enable,
    .foc_get_mech_angle = foc_get_mech_angle,
    .foc_get_phase_current = foc_get_phase_current,
};

/// @brief 调试打印
/// @param fmt 
/// @param  
void foc_debug_printf(const char *const fmt, ...){
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

/// @brief 毫秒延时
/// @param ms 
void foc_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
// --------------------------------用户实现 end---------------------------------------------
