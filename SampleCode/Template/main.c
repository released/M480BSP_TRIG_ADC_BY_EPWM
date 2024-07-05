/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "misc_config.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_REVERSE1                   			(flag_PROJ_CTL.bit1)
#define FLAG_PROJ_REVERSE2                 				(flag_PROJ_CTL.bit2)
#define FLAG_PROJ_REVERSE3                              (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)


/*_____ D E F I N I T I O N S ______________________________________________*/

volatile unsigned int counter_systick = 0;
volatile uint32_t counter_tick = 0;

uint32_t  aADCIRQConvertedData[3] = {0};

#define PWM_TST                                         (32767.0f)
float Math_Pa = 0.0f;
float Math_Pb = 0.0f;
float Math_Pc = 0.0f;

float U_Curr = 0.0f;
float V_Curr = 0.0f;
float W_Curr = 0.0f;

float Alpha = 0.0f;
float Beta = 0.0f;

float t1 = 0.0f;
float t2 = 0.0f;
float v_Alpha = 0.0f;
float v_Beta = 0.0f;
float Va = 0.0f;
float Vb = 0.0f;
float Vc = 0.0f;
float PWM_a = 0.0f;
float PWM_b = 0.0f;
float PWM_c = 0.0f;
float temp_sv1 = 0.0f;
float temp_sv2 = 0.0f;

float pid1_id = 0.0f;
float pid1_iq = 0.0f;
    
/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

unsigned int get_systick(void)
{
	return (counter_systick);
}

void set_systick(unsigned int t)
{
	counter_systick = t;
}

void systick_counter(void)
{
	counter_systick++;
}

void SysTick_Handler(void)
{

    systick_counter();

    if (get_systick() >= 0xFFFFFFFF)
    {
        set_systick(0);      
    }

    // if ((get_systick() % 1000) == 0)
    // {
       
    // }

    #if defined (ENABLE_TICK_EVENT)
    TickCheckTickEvent();
    #endif    
}

void SysTick_delay(unsigned int delay)
{  
    
    unsigned int tickstart = get_systick(); 
    unsigned int wait = delay; 

    while((get_systick() - tickstart) < wait) 
    { 
    } 

}

void SysTick_enable(unsigned int ticks_per_second)
{
    set_systick(0);
    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while (1);
    }

    #if defined (ENABLE_TICK_EVENT)
    TickInitTickEvent();
    #endif
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

void delay_ms(uint16_t ms)
{
	#if 1
    uint32_t tickstart = get_tick();
    uint32_t wait = ms;
	uint32_t tmp = 0;
	
    while (1)
    {
		if (get_tick() > tickstart)	// tickstart = 59000 , tick_counter = 60000
		{
			tmp = get_tick() - tickstart;
		}
		else // tickstart = 59000 , tick_counter = 2048
		{
			tmp = 60000 -  tickstart + get_tick();
		}		
		
		if (tmp > wait)
			break;
    }
	
	#else
	TIMER_Delay(TIMER0, 1000*ms);
	#endif
}

void EADC00_IRQHandler(void)
{
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
    aADCIRQConvertedData[0] = EADC_GET_CONV_DATA(EADC, 2);
    aADCIRQConvertedData[1] = EADC_GET_CONV_DATA(EADC, 1);
    aADCIRQConvertedData[2] = EADC_GET_CONV_DATA(EADC, 0);
}

void EADC_Init(void)
{
    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    /* Configure the sample module 0 for analog input channel 2 and enable EPWM0 trigger source */
    EADC_ConfigSampleModule(EADC, 0, EADC_EPWM0TG0_TRIGGER, 2);
    EADC_ConfigSampleModule(EADC, 1, EADC_EPWM0TG0_TRIGGER, 1);
    EADC_ConfigSampleModule(EADC, 2, EADC_EPWM0TG0_TRIGGER, 0);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* Enable the sample module 0 interrupt */

    EADC_ENABLE_INT(EADC, BIT0 | BIT1 | BIT2);//Enable sample module A/D ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT0);//Enable sample module 0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT1);//Enable sample module 0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT2);//Enable sample module 0 interrupt.

    NVIC_EnableIRQ(EADC00_IRQn);

}

void Zero_Curr(void)
{
    V_Curr = (float)(aADCIRQConvertedData[0]-1251);
    V_Curr *= 26.0f; 
    V_Curr /= 32767.0f;
    W_Curr = (float)(aADCIRQConvertedData[1]-1250); 
    W_Curr *= 26.0f; 
    W_Curr /= 32767.0f;
    U_Curr = (double) 0.0f - (W_Curr + V_Curr);
}

void Park(void)
{
    float Sin_Value = 0.0f,Cos_Value = 0.0f;
    // uint16_t Cos_deg = 0;

    Sin_Value = 0.123f;   //SinTable[ELEC_deg];///32767.0;
    // Cos_deg = 0.456;    //(ELEC_deg + 256)&0x3FF;
    Cos_Value = 0.789f;  //SinTable[Cos_deg];///32767.0;
    
    // pid1_id.term.Fbk = (Alpha * Cos_Value) + (Beta  * Sin_Value);
    // pid1_iq.term.Fbk = (Beta  * Cos_Value ) - (Alpha * Sin_Value);

    pid1_id = (Alpha * Cos_Value) + (Beta  * Sin_Value);
    pid1_iq = (Beta  * Cos_Value ) - (Alpha * Sin_Value);
}

void clarkel(void)//abc to Alpha/Beta
{
    Alpha = U_Curr;
    Beta = (double) (U_Curr+V_Curr+V_Curr)*0.57735026918963f; 
}


void IPARK_MACRO(void)
{
    float Sin_Value = 0.0f,Cos_Value = 0.0f;
    // uint16_t Cos_deg = 0;

    Sin_Value = 0.123f;  //SinTable[ELEC_deg];///32767.0;
    // Cos_deg = 0.456f;    //(ELEC_deg + 256) & 0x3FF;
    Cos_Value = 0.789f;  //SinTable[Cos_deg];///32767.0;

    // v_Alpha = pid1_id.term.Out * Cos_Value - pid1_iq.term.Out * Sin_Value;
    // v_Beta  = pid1_iq.term.Out * Cos_Value + pid1_id.term.Out * Sin_Value;

    v_Alpha = pid1_id * Cos_Value - pid1_iq * Sin_Value;
    v_Beta  = pid1_iq * pid1_id * Sin_Value;

}


void Calc_Time(void)
{
    float V1,V2;
    V1 = PWM_TST*t1;
    V2 = PWM_TST*t2;
    Math_Pc = (PWM_TST - V1 - V2) / 2.0f;
    Math_Pb = Math_Pc + V1;
    Math_Pa = Math_Pb + V2;
}
void SVM(void)
{
    Va= v_Beta;
    temp_sv1 = v_Beta / 2.0f;
    temp_sv2 = v_Alpha*0.8660254f; // 0.8660254 = sqrt(3)/2
    Va = v_Beta;               
    Vb = -temp_sv1 + temp_sv2; //           
    Vc = -temp_sv1 - temp_sv2; //

    if( (float) Va < 0.0f)
    {
        if(Vb < 0.0f)  //180-240(a<0 b<0 cx)
        {
            t1 = -Vb;      //                                   
            t2 = -Va;      //
            Calc_Time();     //                                    
            PWM_a = Math_Pc ;     //               
            PWM_b = Math_Pb;     // 
            PWM_c = Math_Pa;      // 
        }
    else
    {
        if(Vc < 0.0f) //300-360(a<0 b>0 c<0)
        {
            t1 = -Va;      //                                   
            t2 = -Vc;      //
            Calc_Time();     //                                    
            PWM_a = Math_Pa ;     //               
            PWM_b = Math_Pc;     // 
            PWM_c = Math_Pb;      // 
        }
        else   //240-300(a<0 b>0 c>0)
        {
            t1 = Vb;      //                                   
            t2 = Vc;      //
            Calc_Time();     //                                    
            PWM_a = Math_Pb ;     //               
            PWM_b = Math_Pc;     // 
            PWM_c = Math_Pa;      // 
        }
    }
    }
    else
    {
        if(Vb < 0.0f)
        {
            if(Vc < 0.0f) //60-120(a>0 b<0 c<0)
            {
                t1 = -Vc;      //                                   
                t2 = -Vb;      //
                Calc_Time();     //                                    
                PWM_a = Math_Pb ;     //               
                PWM_b = Math_Pa;     // 
                PWM_c = Math_Pc;      // 
            }
            else   //120-180(a>0 b>0 c>0)
            {
                t1 = Vc;      //                                   
                t2 = Va;      //
                Calc_Time();     //                                    
                PWM_a = Math_Pc ;     //               
                PWM_b = Math_Pa;     // 
                PWM_c = Math_Pb;      // 
            }
        }
        else    //0-60(a>0 b>0 cx)
        {
            t1 = Va;      //                                   
            t2 = Vb;      //
            Calc_Time();     //                                    
            PWM_a = Math_Pa ;     //               
            PWM_b = Math_Pb;     // 
            PWM_c = Math_Pc;      // 
        }
    }
}

void current_ctrl(void)
{
    Zero_Curr();
    clarkel();
    Park();
    IPARK_MACRO();
    SVM();
}

void EPWM0P0_IRQHandler(void)
{
    PC6 = 1;
    current_ctrl(); 
    PC6 = 0;

    // Clear channel 0 period interrupt flag
    EPWM_ClearPeriodIntFlag(EPWM0, 0);
}

void EPWM0_Init(void)
{
    #if 1
    EPWM_ConfigOutputChannel(EPWM0, 0, 20000, 50);

    #else
    /* Set EPWM0 timer clock prescaler */
    EPWM_SET_PRESCALER(EPWM0, 0, 10);

    /* Set up counter type */
    EPWM0->CTL1 &= ~EPWM_CTL1_CNTTYPE0_Msk;

    /* Set EPWM0 timer duty */
    EPWM_SET_CMR(EPWM0, 0, 1000);

    /* Set EPWM0 timer period */
    EPWM_SET_CNR(EPWM0, 0, 2000);
    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    EPWM_SET_OUTPUT_LEVEL(EPWM0, BIT0, EPWM_OUTPUT_HIGH, EPWM_OUTPUT_LOW, EPWM_OUTPUT_NOTHING, EPWM_OUTPUT_NOTHING);
    #endif

    /* EPWM period point trigger ADC enable */
    EPWM_EnableADCTrigger(EPWM0, 0, EPWM_TRG_ADC_EVEN_PERIOD);

    /* Enable output of EPWM0 channel 0 */
    EPWM_EnableOutput(EPWM0, EPWM_CH_0_MASK);

    EPWM_EnablePeriodInt(EPWM0, 0, 0);
    NVIC_EnableIRQ(EPWM0P0_IRQn);

    EPWM_Start(EPWM0, EPWM_CH_0_MASK); //EPWM0 channel 0 counter start running.
}

//
// check_reset_source
//
uint8_t check_reset_source(void)
{
    uint32_t src = SYS_GetResetSrc();

    if ((SYS->CSERVER & SYS_CSERVER_VERSION_Msk) == 0x1)    // M48xGCAE
    {
		printf("PN : M48xGCAE\r\n");
    }
    else    // M48xIDAE
    {
		printf("PN : M48xIDAE\r\n");
    }

    SYS->RSTSTS |= 0x1FF;
    printf("Reset Source <0x%08X>\r\n", src);

    #if 1   //DEBUG , list reset source
    if (src & BIT0)
    {
        printf("0)POR Reset Flag\r\n");       
    }
    if (src & BIT1)
    {
        printf("1)NRESET Pin Reset Flag\r\n");       
    }
    if (src & BIT2)
    {
        printf("2)WDT Reset Flag\r\n");       
    }
    if (src & BIT3)
    {
        printf("3)LVR Reset Flag\r\n");       
    }
    if (src & BIT4)
    {
        printf("4)BOD Reset Flag\r\n");       
    }
    if (src & BIT5)
    {
        printf("5)System Reset Flag \r\n");       
    }
    if (src & BIT6)
    {
        printf("6)HRESET Reset Flag \r\n");       
    }
    if (src & BIT7)
    {
        printf("7)CPU Reset Flag\r\n");       
    }
    if (src & BIT8)
    {
        printf("8)CPU Lockup Reset Flag\r\n");       
    }
    #endif
    
    if (src & SYS_RSTSTS_PORF_Msk) {
        SYS_ClearResetSrc(SYS_RSTSTS_PORF_Msk);
        
        printf("power on from POR\r\n");
        return FALSE;
    }    
    else if (src & SYS_RSTSTS_PINRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_PINRF_Msk);
        
        printf("power on from nRESET pin\r\n");
        return FALSE;
    } 
    else if (src & SYS_RSTSTS_WDTRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_WDTRF_Msk);
        
        printf("power on from WDT Reset\r\n");
        return FALSE;
    }    
    else if (src & SYS_RSTSTS_LVRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_LVRF_Msk);
        
        printf("power on from LVR Reset\r\n");
        return FALSE;
    }    
    else if (src & SYS_RSTSTS_BODRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_BODRF_Msk);
        
        printf("power on from BOD Reset\r\n");
        return FALSE;
    }    
    else if (src & SYS_RSTSTS_SYSRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_SYSRF_Msk);
        
        printf("power on from System Reset\r\n");
        return FALSE;
    } 
    else if (src & SYS_RSTSTS_CPURF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_CPURF_Msk);

        printf("power on from CPU reset\r\n");
        return FALSE;         
    }    
    else if (src & SYS_RSTSTS_CPULKRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_CPULKRF_Msk);
        
        printf("power on from CPU Lockup Reset\r\n");
        return FALSE;
    }   
    
    printf("power on from unhandle reset source\r\n");
    return FALSE;
}

void TMR1_IRQHandler(void)
{
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_1000MS = 1;//set_flag(flag_timer_period_1000ms ,ENABLE);
		}

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void loop(void)
{
	// static uint32_t LOG1 = 0;
	// static uint32_t LOG2 = 0;

    if ((get_systick() % 1000) == 0)
    {
        // printf("%s(systick) : %4d\r\n",__FUNCTION__,LOG2++);    
    }

    if (FLAG_PROJ_TIMER_PERIOD_1000MS)//(is_flag_set(flag_timer_period_1000ms))
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 0;//set_flag(flag_timer_period_1000ms ,DISABLE);

        // printf("%s(timer) : %4d\r\n",__FUNCTION__,LOG1++);
        PH0 ^= 1;             
    }

    printf("Math_Pa:%4.2f\r\n",Math_Pa);
    printf("Math_Pb:%4.2f\r\n",Math_Pb);
    printf("Math_Pc:%4.2f\r\n",Math_Pc);

    printf("U_Curr:%4.2f\r\n",U_Curr);
    printf("V_Curr:%4.2f\r\n",V_Curr);
    printf("W_Curr:%4.2f\r\n",W_Curr);
    printf("Alpha:%4.2f\r\n",Alpha);
    printf("Beta:%4.2f\r\n",Beta);
    printf("t1:%4.2f\r\n",t1);
    printf("t2:%4.2f\r\n",t2);
    printf("v_Alpha:%4.2f\r\n",v_Alpha);
    printf("v_Beta:%4.2f\r\n",v_Beta);
    printf("Va:%4.2f\r\n",Va);
    printf("Vb:%4.2f\r\n",Vb);
    printf("Vc:%4.2f\r\n",Vc);
    printf("PWM_a:%4.2f\r\n",PWM_a);
    printf("PWM_b:%4.2f\r\n",PWM_b);
    printf("PWM_c:%4.2f\r\n",PWM_c);
    printf("temp_sv1:%4.2f\r\n",temp_sv1);
    printf("temp_sv2:%4.2f\r\n",temp_sv2);
    printf("pid1_id:%4.2f\r\n",pid1_id);
    printf("pid1_iq:%4.2f\r\n",pid1_iq);
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		printf("press : %c\r\n" , res);
		switch(res)
		{
			case '1':
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
                SYS_UnlockReg();
				// NVIC_SystemReset();	// Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
                // SYS_ResetCPU();     // Not reset I/O and peripherals
                SYS_ResetChip();    // Reset I/O and peripherals ,  BS(FMC_ISPCTL[1]) reload from CONFIG setting (CBS)	
				break;
		}
	}
}

void UART0_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);

	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());    	

//    printf("Product ID 0x%8X\n", SYS->PDID);
	
	#endif	

    #if 0
    printf("FLAG_PROJ_TIMER_PERIOD_1000MS : 0x%2X\r\n",FLAG_PROJ_TIMER_PERIOD_1000MS);
    printf("FLAG_PROJ_REVERSE1 : 0x%2X\r\n",FLAG_PROJ_REVERSE1);
    printf("FLAG_PROJ_REVERSE2 : 0x%2X\r\n",FLAG_PROJ_REVERSE2);
    printf("FLAG_PROJ_REVERSE3 : 0x%2X\r\n",FLAG_PROJ_REVERSE3);
    printf("FLAG_PROJ_REVERSE4 : 0x%2X\r\n",FLAG_PROJ_REVERSE4);
    printf("FLAG_PROJ_REVERSE5 : 0x%2X\r\n",FLAG_PROJ_REVERSE5);
    printf("FLAG_PROJ_REVERSE6 : 0x%2X\r\n",FLAG_PROJ_REVERSE6);
    printf("FLAG_PROJ_REVERSE7 : 0x%2X\r\n",FLAG_PROJ_REVERSE7);
    #endif

}

void GPIO_Init (void)
{
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH0MFP_Msk)) | (SYS_GPH_MFPL_PH0MFP_GPIO);
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH1MFP_Msk)) | (SYS_GPH_MFPL_PH1MFP_GPIO);
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH2MFP_Msk)) | (SYS_GPH_MFPL_PH2MFP_GPIO);

	//EVM LED
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);

    
	SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC6MFP_Msk)) | (SYS_GPC_MFPL_PC6MFP_GPIO);
	GPIO_SetMode(PC,BIT6,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
//    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    /***********************************/
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
	
    /***********************************/
    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
	
    /***********************************/
    /* Enable EPWM0 module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);

    /* Select EPWM0 module clock source as PCLK0 */
    CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PCLK0, 0);

    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk));
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA5MFP_EPWM0_CH0;

    /***********************************/

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* EADC clock source is 96MHz, set divider to 8, EADC clock is 96/8 MHz */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

    /* Set PB.0 ~ PB.3 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk | GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    /* Configure the GPB0 - GPB3 ADC analog input pins.  */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk |
                       SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_EADC0_CH0 | SYS_GPB_MFPL_PB1MFP_EADC0_CH1 |
                      SYS_GPB_MFPL_PB2MFP_EADC0_CH2 | SYS_GPB_MFPL_PB3MFP_EADC0_CH3);

    /* Disable the GPB0 - GPB3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT3|BIT2|BIT1|BIT0);

    /***********************************/

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	GPIO_Init();
	UART0_Init();
	TIMER1_Init();
    check_reset_source();

    SysTick_enable(1000);
    #if defined (ENABLE_TICK_EVENT)
    TickSetTickEvent(1000, TickCallback_processA);  // 1000 ms
    TickSetTickEvent(5000, TickCallback_processB);  // 5000 ms
    #endif

    EADC_Init();
    EPWM0_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
