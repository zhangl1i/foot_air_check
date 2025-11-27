/*********************************************************************************
 * 文件名  ：main.c
 * 描述    ：        
 * 硬件连接：
 * 4针OLED：GND -> GND; VCC -> 3.3V; SCL -> PB6; SDA -> PB7;
 * 气压传感器：VCC -> 5V; GND -> GND; AO -> PA1; 
 * USB转串口模块：GND -> GND; RX -> PA9; TX -> PA10; 
 * 功能描述：测量气压值OLED液晶屏显示；
             串口接收测量所得的气压值（波特率9600）；
   使用时先进行零点和满量程校准，依据校准值调整 Voltage_0   Voltage_40
**********************************************************************************/
#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"
#include "adc.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

int ADC;
float Pressure_V=0.0;
long pressure=0;

char V_disbuff[5]={0}; 
char P_disbuff[6]={0};

const float VCC =3300;         // ADC参考电压为mV
float map_voltage(float input_voltage, float in_min, float in_max, float out_min, float out_max);

int main(void)
{
    delay_init();
    NVIC_Configuration();  // NVIC 中断配置
    uart_init(9600);       // 串口初始化 9600
    Adc_Init();

    while (1)
    {
    // 先声明所有变量
    int ADC_local;
    float Pressure_V_local = 0.0f;
    float sensor_voltage;
    long pressure_local;

    ADC_local = Get_Adc_Average(1, 10); // 取 10 次 ADC 平均值
    Pressure_V_local = (ADC_local * VCC) / 4095.0f;   // VCC单位是mV，这里计算出来也是mV
//    sensor_voltage = map_voltage(Pressure_V_local , 0, 3300, 500, 4500); // 转换成伏特后映射
    pressure_local = (long)((Pressure_V_local - 2500) / 0.02);
//	printf("111");
//    printf("Pressure_V_local: %f mV\r\n", Pressure_V_local);
//		printf("sensor_voltage: %f mV\r\n", sensor_voltage);
//    printf("Pressure: %ld pa\r\n", pressure_local);
		printf("%ld\r\n", pressure_local);

    delay_ms(500);
    }
}

float map_voltage(float input_voltage, float in_min, float in_max, float out_min, float out_max)
{
    if(input_voltage < in_min) input_voltage = in_min;
    if(input_voltage > in_max) input_voltage = in_max;
    return out_min + (input_voltage - in_min) * (out_max - out_min) / (in_max - in_min);;
}


