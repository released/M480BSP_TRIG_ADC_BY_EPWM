# M480BSP_TRIG_ADC_BY_EPWM
 M480BSP_TRIG_ADC_BY_EPWM


udpate @ 2024/07/05

1. trigger ADC by EPWRM 

ADC channel :PB0/EADC0_CH0 , PB1/EADC0_CH1 , PB2/EADC0_CH2 , PB3/EADC0_CH3

EPWM : PA5/EPWM0_CH0

2. use PC.6 to measure float point calculation timing under EPWM IRQ


![image](https://github.com/released/M480BSP_TRIG_ADC_BY_EPWM/blob/main/scope_EPWM_ADC_GPIO_Toggle.jpg)



3. need to add f under each float point


![image](https://github.com/released/M480BSP_TRIG_ADC_BY_EPWM/blob/main/float.jpg)


5. below is timing difference between different optimization 


![image](https://github.com/released/M480BSP_TRIG_ADC_BY_EPWM/blob/main/detail1.jpg)


![image](https://github.com/released/M480BSP_TRIG_ADC_BY_EPWM/blob/main/detail2.jpg)


