
#include "N76E003.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "Common.h"
#include "Delay.h"

void setTenLedLevel(void);

#define RF_MODE		0
#define CK_MODE		1

#define BUZZER_MODE	0
#define MOTOR_MODE	1

#define ON			1
#define OFF			0

/*	LED IO define	*/
#define	RF_SET_LED		P00_PushPull_Mode;P00 = 1;
#define RF_CLR_LED		P00_PushPull_Mode;P00 = 0;

#define	CK_SET_LED		P01_PushPull_Mode;P01 = 1;
#define CK_CLR_LED		P01_PushPull_Mode;P01 = 0;

/*	KEY	IO	define	*/
#define MODE_SWITCH_KEY_INIT	P06_Quasi_Mode;P06 = 1;
#define GET_MODE_SWITCH_KEY		P06

#define SIX_LED_KEY_INIT		P05_Quasi_Mode;P05 = 1;


#define KEY_LONG_PRESS			400		


#define MOTOR_SET_RUN	P10_PushPull_Mode;P10 = 0;
#define MOTOR_CLR_RUN	P10_PushPull_Mode;P10 = 1;

#define BUZZER_SET_RUN	P12_PushPull_Mode;P12 = 1;
#define BUZZER_CLR_RUN	P12_PushPull_Mode;P12 = 0;

#define CK_INPUT_INIT	P30_Input_Mode;
#define GET_CK			P30

#define RF_ADC_INPUT_INIT	Enable_ADC_AIN2;


/*	bits define		*/
bit is_5ms_Flag;
bit RF_CK_MODE;
bit RF_CK_ISLONGPRESS;
bit MOTOR_BUZZER_MODE;
bit SIX_LED_ON_OFF;
bit SIX_LED_ISLONGPRESS;
bit isSIX_LED_ENTER_FLASH_MODE;
bit Buzzer_Motor_isRun;
/*		system		*/
unsigned int systime_cnt;
/*		key			*/
unsigned int RF_CK_KEY_debounce;
unsigned char SIX_LED_KEY_debounce;

/*		six led pwm level		*/
unsigned char	SIX_LED_Freq;

unsigned char RF_CK_LED_Level;
unsigned char RF_CK_LED_display_step;

unsigned char RF_CK_RUN_CNT;
unsigned char RF_Level;
unsigned char RF_Level_add_cnt;
unsigned char RF_Level_del_cnt;
unsigned char RF_Level_Debounce_cnt;
/*			Timer 5ms			*/
/************************************************************************************************************
*    TIMER 0 interrupt subroutine
************************************************************************************************************/
#define TH0_INIT        (65536-6667)/256 //5.0ms@XTAL=12MHz, Period = (10.85/2) ms@XTAL=22.1184MHz
#define TL0_INIT        (65536-6667)%256
#define TH1_INIT        (65536-1333)/256 //2.5ms@XTAL=12MHz, Period = (5.425/2) ms@XTAL=22.1184MHz
#define TL1_INIT        (65536-1333)%256

void Timer0_ISR (void) interrupt 1          //interrupt address is 0x000B
{
    TH0 = TH0_INIT;
    TL0 = TL0_INIT;
	is_5ms_Flag = 1;
	systime_cnt++;
}

void Timer0_Init(void)
{
	TIMER0_MODE1_ENABLE;                        //Timer 0  mode configuration
    
	clr_T0M;
    
	TH0 = TH0_INIT;
	TL0 = TL0_INIT;
    
	set_ET0;                                    //enable Timer0 interrupt
	set_EA;                                     //enable interrupts
	
	set_TR0;                                    //Timer0 run
	
	setTenLedLevel();
}

void Timer1_ISR (void) interrupt 3
{
	TH1 = TH1_INIT;
    TL1 = TL1_INIT;
	setTenLedLevel();
}

void Timer1_Init(void)
{
	//for 1ms interrupt
	TIMER1_MODE1_ENABLE;                        //Timer 0  mode configuration
    
	clr_T1M;
    
	TH1 = TH1_INIT;
	TL1 = TL1_INIT;
    
	set_ET1;                                    //enable Timer0 interrupt
	set_EA;                                     //enable interrupts
	
	set_TR1;                                    //Timer0 run	
}

/*		battery power		*/
float  Bandgap_Voltage,VDD_Voltage;			//please always use "double" mode for this
int ADCsumH=0, ADCsumL=0;

void get_battery_power(void)
{
		float bgvalue;
		unsigned int i;
	
		UINT8 BandgapHigh,BandgapLow,BandgapMark;
		float Bandgap_Value,Bandgap_Voltage_Temp;
	
		set_IAPEN;
		IAPCN = READ_UID;
		IAPAL = 0x0d;
		IAPAH = 0x00;
		set_IAPGO;
		BandgapLow = IAPFD;
		BandgapMark = BandgapLow&0xF0;
			
		if (BandgapMark==0x80)
		{
				BandgapLow = BandgapLow&0x0F;
				IAPAL = 0x0C;
				IAPAH = 0x00;
				set_IAPGO;
				BandgapHigh = IAPFD;
				Bandgap_Value = (BandgapHigh<<4)+BandgapLow;
				Bandgap_Voltage_Temp = Bandgap_Value*3/4;
				Bandgap_Voltage = Bandgap_Voltage_Temp - 33;			//the actually banggap voltage value is similar this value.
		}
		if (BandgapMark==0x00)
		{
				BandgapLow = BandgapLow&0x0F;
				IAPAL = 0x0C;
				IAPAH = 0x00;
				set_IAPGO;
				BandgapHigh = IAPFD;
				Bandgap_Value = (BandgapHigh<<4)+BandgapLow;
				Bandgap_Voltage= Bandgap_Value*3/4;
		}
		if (BandgapMark==0x90)
		{
				IAPAL = 0x0E;
				IAPAH = 0x00;
				set_IAPGO;
				BandgapHigh = IAPFD;
				IAPAL = 0x0F;
				IAPAH = 0x00;
				set_IAPGO;
				BandgapLow = IAPFD;
				BandgapLow = BandgapLow&0x0F;
				Bandgap_Value = (BandgapHigh<<4)+BandgapLow;
				Bandgap_Voltage= Bandgap_Value*3/4;
		}
		clr_IAPEN;
		Enable_ADC_BandGap;
		ADCsumH = 0;
		ADCsumL = 0;
		CKDIV = 0x02;															// IMPORTANT!! Modify system clock to 4MHz ,then add the ADC sampling clock base to add the sampling timing.
		for(i=0;i<5;i++)													// All following ADC detect timing is 200uS run under 4MHz.
		{
			clr_ADCF;
			set_ADCS;
			while(ADCF == 0);
			if (i >= 2)
			{
				ADCsumH += ADCRH;
				ADCsumL += ADCRL;
			}
		}		
		CKDIV = 0x00;															// After ADC sampling, modify system clock back to 16MHz to run next code.
			
		ADCsumH = ADCsumH/3;
		ADCsumL = ADCsumL/3;
		bgvalue = (ADCsumH<<4) + ADCsumL;
		VDD_Voltage = (0x1000/bgvalue)*Bandgap_Voltage;
}


/*				KEY				*/
unsigned char get_RF_CK_Key(void)
{
	unsigned char keystatus;
	keystatus = 0;
	if (P06 == 0)
	{
		RF_CK_KEY_debounce++;
		if (RF_CK_KEY_debounce > KEY_LONG_PRESS)
		{
			RF_CK_KEY_debounce = KEY_LONG_PRESS;
			if (RF_CK_ISLONGPRESS == 0)
			{
				RF_CK_ISLONGPRESS = 1;
				keystatus = 0x02;
			}
		}
	}
	else
	{
		if (RF_CK_ISLONGPRESS == 0)
		{
			if (RF_CK_KEY_debounce > 20)
			{
				keystatus = 1;
			}
		}
		RF_CK_KEY_debounce = 0;
		RF_CK_ISLONGPRESS = 0;
	}
	return keystatus;
}


unsigned char get_SIX_LED_Key(void)
{
	unsigned char keystatus;
	keystatus = 0;
	if (P05 == 0)
	{
		SIX_LED_KEY_debounce++;
		if (SIX_LED_KEY_debounce > 110)
		{
			SIX_LED_KEY_debounce = 0;
			SIX_LED_ISLONGPRESS = 1;
			keystatus = 0x02;
		}
	}
	else
	{
		if (SIX_LED_ISLONGPRESS == 0)
		{
			if (SIX_LED_KEY_debounce > 20)
				keystatus = 1;
		}
		else if (SIX_LED_ISLONGPRESS)
		{
			keystatus = 4;
		}
		SIX_LED_ISLONGPRESS = 0;
		SIX_LED_KEY_debounce = 0;
	}
	return keystatus;
}

/*				SIX LED			*/
void PWM_ISR (void) interrupt 13
{
    clr_PWMF;               // clear PWM interrupt flag
}

void setSixLedFreq(unsigned char freq,unsigned char duty)
{
	unsigned int freqtemp;
	if (duty == 0)
	{
		//turn off six led
		//disable pwm P1.1
		PWM1_P11_OUTPUT_DISABLE;
		clr_PWMRUN;
		P11_PushPull_Mode;
		P11 = 0;
	}
	else if (duty == 100)
	{
		//turn on six led
		//disable pwm P1.1
		PWM1_P11_OUTPUT_DISABLE;
		clr_PWMRUN;
		P11_PushPull_Mode;
		P11 = 1;
	}
	else if (duty == 50)
	{
		if (freq > 10)
			freq = 10;
		if (freq & 0x01)
		{
			freq -= 1;
		}
		/*	set freqs	*/
		if (freq == 10)		//10HZ
		{
			freqtemp = 0x3080;
		}
		else if (freq == 8)	//7HZ
		{
			freqtemp = 0x45C0;
		}
		else if (freq == 6)	//5HZ
		{
			freqtemp = 0x6180;
		}
		else if (freq == 4)	//3HZ
		{
			freqtemp = 0xA200;
		}
		else if (freq == 2)	//2HZ
		{
			freqtemp = 0xF400;
		}
		else
			return;
		//turn on with pwm
		//enbale pwm P1.1
		PWM1_P11_OUTPUT_ENABLE;

//		PWM_IMDEPENDENT_MODE;
		
		PWM_INT_PWM2;
		PWM_FALLING_INT;
		
		PWM_CLOCK_FSYS;
		PWM_CLOCK_DIV_128;
		PWMPH = freqtemp>>8;
		PWMPL = freqtemp&0xFF;
		set_SFRPAGE;
		freqtemp = freqtemp>>1;
		freqtemp = freqtemp + 1;
		PWM1H = freqtemp>>8;
		PWM1L = freqtemp&0xFF;
		clr_SFRPAGE;
		//-------- PWM start run--------------
		set_LOAD;
		set_PWMRUN;			
	}
}


/*				TEN	LED			*/

void RF_CK_LED_LEVEL1(unsigned char step)
{
		#if 1	// 1
		P17_Quasi_Mode;
		P15_Quasi_Mode;
		P14_Quasi_Mode;
		P13_Quasi_Mode;
		if (step == 4)
		{
			P17_Input_Mode;
			P17 = 1;
			P15_Input_Mode;
			P15 = 1;
			P14_Input_Mode;
			P14 = 1;
			P13_Input_Mode;
			P13 = 1;
		}
		else if (step == 1)
		{
			P17_PushPull_Mode;
			P17 = 1;
			P15_PushPull_Mode;
			P15 = 1;
			P14_Input_Mode;
			P14 = 0;
			P13_Input_Mode;
			P13 = 0;
		}
		else if (step == 2)
		{
			P17_Input_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 1;
			P14_Input_Mode;
			P14 = 0;
			P13_Input_Mode;
			P13 = 0;
		}
		else if (step == 3)
		{
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 1;
			P14_Input_Mode;
			P14 = 0;
			P13_Input_Mode;
			P13 = 0;
		}
		
		#endif
}
void RF_CK_LED_LEVEL2(unsigned char step)
{
		#if 1	// 2
			
			P17_Quasi_Mode;
			P15_Quasi_Mode;
			P14_Quasi_Mode;
			P13_Quasi_Mode;
		if (step == 4)
		{
			step = 1;
			P17_PushPull_Mode;
			P17 = 1;
			P15_PushPull_Mode;
			P15 = 1;
			P14_PushPull_Mode;
			P14 = 1;
			P13_PushPull_Mode;
			P13 = 1;
		}
		else if (step == 1)
		{
			step = 2;
			P17_PushPull_Mode;
			P17 = 1;
			P15_PushPull_Mode;
			P15 = 1;
			P14_Input_Mode;
			P14 = 0;
			P13_Input_Mode;
			P13 = 0;
		}
		else if (step == 2)
		{
			step = 3;
			P17_Input_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 1;
			P14_Input_Mode;
			P14 = 0;
			P13_Input_Mode;
			P13 = 0;
		}
		else if (step == 3)
		{
			step = 0;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 1;
			P14_PushPull_Mode;
			P14 = 1;
			P13_Input_Mode;
			P13 = 0;
		}
		#endif
}
void RF_CK_LED_LEVEL3(unsigned char step)
{
		#if 1	//3
			
			P17_Quasi_Mode;
			P15_Quasi_Mode;
			P14_Quasi_Mode;
			P13_Quasi_Mode;
		if (step == 4)
		{
			step = 1;
			P15_PushPull_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 1;
			P14_PushPull_Mode;
			P14 = 1;
			P13_PushPull_Mode;
			P13 = 1;
		}
		else if (step == 1)
		{
			step = 2;
			P17_Input_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 1;
			P14_Input_Mode;
			P14 = 0;
			P13_Input_Mode;
			P13 = 0;
		}
		else if (step == 2)
		{
			step = 3;
			P17_Input_Mode;
			P17 = 0;
			P15_Input_Mode;
			P15 = 0;
			P14_Input_Mode;
			P14 = 0;
			P13_Input_Mode;
			P13 = 0;
		}
		else if (step == 3)
		{
			step = 0;
			P17_Input_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 0;
			P14_Input_Mode;
			P14 = 0;
			P13_PushPull_Mode;
			P13 = 0;
		}
		#endif
		
}
void RF_CK_LED_LEVEL4(unsigned char step)
{
		#if 1	//4
			
			P17_Quasi_Mode;
			P15_Quasi_Mode;
			P14_Quasi_Mode;
			P13_Quasi_Mode;
		if (step == 4)
		{
			step = 1;
			P17_PushPull_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 1;
			P14_PushPull_Mode;
			P14 = 1;
			P13_PushPull_Mode;
			P13 = 1;
		}
		else if (step == 1)
		{
			step = 2;
			P17_PushPull_Mode;
			P17 = 1;
			P15_PushPull_Mode;
			P15 = 0;
			P14_Input_Mode;
			P14 = 0;
			P13_Input_Mode;
			P13 = 0;
		}
		else if (step == 2)
		{
			step = 3;
			P17_Input_Mode;
			P17 = 0;
			P15_Input_Mode;
			P15 = 0;
			P14_Input_Mode;
			P14 = 0;
			P13_Input_Mode;
			P13 = 0;
		}
		else if (step == 3)
		{
			step = 0;
			P17_Input_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 0;
			P14_Input_Mode;
			P14 = 0;
			P13_PushPull_Mode;
			P13 = 0;
		}
		#endif
		
}
void RF_CK_LED_LEVEL5(unsigned char step)
{
		#if 1	//5
			
			P17_Quasi_Mode;
			P15_Quasi_Mode;
			P14_Quasi_Mode;
			P13_Quasi_Mode;
		if (step == 4)
		{
			step = 1;
			P17_PushPull_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 1;
			P14_PushPull_Mode;
			P14 = 1;
			P13_PushPull_Mode;
			P13 = 1;
		}
		else if (step == 1)
		{
			step = 2;
			P17_PushPull_Mode;
			P17 = 1;
			P15_PushPull_Mode;
			P15 = 0;
			P14_PushPull_Mode;
			P14 = 1;
			P13_Input_Mode;
			P13 = 0;
		}
		else if (step == 2)
		{
			step = 3;
			P17_PushPull_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 0;
			P14_PushPull_Mode;
			P14 = 0;
			P13_PushPull_Mode;
			P13 = 0;
		}
		else if (step == 3)
		{
			step = 0;
			P17_PushPull_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 0;
			P14_PushPull_Mode;
			P14 = 0;
			P13_PushPull_Mode;
			P13 = 0;
		}
		#endif
}
void RF_CK_LED_LEVEL6(unsigned char step)
{
		#if 1	// 6
			
			P17_Quasi_Mode;
			P15_Quasi_Mode;
			P14_Quasi_Mode;
			P13_Quasi_Mode;
		if (step == 4)
		{
			step = 1;
			P17_PushPull_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 1;
			P14_PushPull_Mode;
			P14 = 1;
			P13_PushPull_Mode;
			P13 = 1;
		}
		else if (step == 1)
		{
			step = 2;
			P17_PushPull_Mode;
			P17 = 1;
			P15_PushPull_Mode;
			P15 = 0;
			P14_PushPull_Mode;
			P14 = 1;
			P13_PushPull_Mode;
			P13 = 1;
		}
		else if (step == 2)
		{
			step = 3;
			P17_PushPull_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 0;
			P14_PushPull_Mode;
			P14 = 0;
			P13_PushPull_Mode;
			P13 = 0;
		}
		else if (step == 3)
		{
			step = 0;
			P17_PushPull_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 0;
			P14_PushPull_Mode;
			P14 = 0;
			P13_PushPull_Mode;
			P13 = 0;
		}
		#endif
		
}
void RF_CK_LED_LEVEL7(unsigned char step)
{
		#if 1	// 7
			
			P17_Input_Mode;
			P15_Input_Mode;
			P14_Input_Mode;
			P13_Input_Mode;
		if (step == 4)
		{
			step = 1;
			P17_PushPull_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 1;
			P14_PushPull_Mode;
			P14 = 1;
			P13_PushPull_Mode;
			P13 = 1;
		}
		else if (step == 1)
		{
			P14_Input_Mode;
			P14 = 1;
			step = 2;
			P17_PushPull_Mode;
			P17 = 1;
			P15_PushPull_Mode;
			P15 = 0;
			P14_PushPull_Mode;
			P14 = 1;
			P13_PushPull_Mode;
			P13 = 1;
		}
		else if (step == 2)
		{
			P14_Input_Mode;
			P14 = 1;
			step = 3;
			P17_PushPull_Mode;
			P17 = 1;
			P15_Input_Mode;
			P15 = 0;
			P14_PushPull_Mode;
			P14 = 0;
			P13_Input_Mode;
			P13 = 0;
		}
		else if (step == 3)
		{
			step = 0;
			P17_PushPull_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 0;
			P14_PushPull_Mode;
			P14 = 0;
			P13_PushPull_Mode;
			P13 = 0;
		}
		#endif
		
}
void RF_CK_LED_LEVEL8(unsigned char step)
{
		
		#if 1	//8
		if (step == 4)
		{
			P14_Input_Mode;
			P14 = 1;
			P15 = 0;
			step = 1;
			P17_PushPull_Mode;
			P17 = 1;
			P15_PushPull_Mode;
			P15 = 0;
			P14_PushPull_Mode;
			P14 = 1;
			P13_PushPull_Mode;
			P13 = 1;
		}
		else if (step == 1)
		{
			step = 2;
			P17_PushPull_Mode;
			P17 = 1;
			P15_PushPull_Mode;
			P15 = 1;
			P14_PushPull_Mode;
			P14 = 0;
			P13_Input_Mode;
			P13 = 0;
		}
		else if (step == 2)
		{
			step = 3;
			P17_PushPull_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 0;
			P14_Input_Mode;
			P14 = 1;
			P13_PushPull_Mode;
			P13 = 0;
		}
		else if (step == 3)
		{
			P17 = 1;
			step = 0;
			P17_PushPull_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 1;
			P14_PushPull_Mode;
			P14 = 1;
			P13_PushPull_Mode;
			P13 = 1;
		}
		#endif
}
void RF_CK_LED_LEVEL9(unsigned char step)
{
	#if 1	//9
		if (step == 4)
		{
			P13_PushPull_Mode;
			P13 = 0;
			P15_Input_Mode;
			P15 = 0;
			step = 1;
			P17_PushPull_Mode;
			P17 = 1;
			P15_PushPull_Mode;
			P15 = 0;
			P14_PushPull_Mode;
			P14 = 1;
			P13_PushPull_Mode;
			P13 = 1;
		}
		else if (step == 1)
		{
			step = 2;
			P17_PushPull_Mode;
			P17 = 1;
			P15_PushPull_Mode;
			P15 = 1;
			P14_PushPull_Mode;
			P14 = 0;
			P13_PushPull_Mode;
			P13 = 1;
		}
		else if (step == 2)
		{
			step = 3;
			P17_PushPull_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 1;
			P14_PushPull_Mode;
			P14 = 1;
			P13_PushPull_Mode;
			P13 = 1;
		}
		else if (step == 3)
		{
			step = 0;
			P17_PushPull_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 0;
			P14_PushPull_Mode;
			P14 = 0;
			P13_PushPull_Mode;
			P13 = 0;
		}
		#endif
		
}
void RF_CK_LED_LEVEL10(unsigned char step)
{
		#if 1	//10
			P17_Quasi_Mode;
			P15_Quasi_Mode;
			P14_Quasi_Mode;
			P13_Quasi_Mode;	
		if (step == 4)
		{
			P17 = 0;			
			P13_PushPull_Mode;
			P13 = 0;
			P15_Input_Mode;
			P15 = 0;
			P14_Input_Mode;
			P14 = 0;
			step = 1;
			P17_PushPull_Mode;
			P17 = 1;
			P15_PushPull_Mode;
			P15 = 0;
			P14_PushPull_Mode;
			P14 = 1;
			P13_PushPull_Mode;
			P13 = 1;
		}
		else if (step == 1)
		{
			P17 = 0;
			step = 2;
			P17_PushPull_Mode;
			P17 = 1;
			P15_PushPull_Mode;
			P15 = 1;
			P14_PushPull_Mode;
			P14 = 0;
			P13_PushPull_Mode;
			P13 = 1;
		}
		else if (step == 2)
		{
			step = 3;
			P17_PushPull_Mode;
			P17 = 1;
			P15_PushPull_Mode;
			P15 = 1;			
			P14_Input_Mode;
			P14 = 1;
			P13_PushPull_Mode;
			P13 = 0;
		}
		else if (step == 3)
		{
			step = 0;
			P17_PushPull_Mode;
			P17 = 0;
			P15_PushPull_Mode;
			P15 = 1;
			P14_PushPull_Mode;
			P14 = 1;
			P13_PushPull_Mode;
			P13 = 1;
		}
		#endif
		
}

void setTenLedLevel(void)
{
	if (RF_CK_LED_Level == 0)
	{
		P15 = 0;
		P14 = 0;
		P13 = 0;
		P17 = 0;
	}
	else if (RF_CK_LED_Level == 1)
	{
		RF_CK_LED_LEVEL1(RF_CK_LED_display_step);
	}
	else if (RF_CK_LED_Level == 2)
	{
		RF_CK_LED_LEVEL2(RF_CK_LED_display_step);
	}
	else if (RF_CK_LED_Level == 3)
	{
		RF_CK_LED_LEVEL3(RF_CK_LED_display_step);
	}
	else if (RF_CK_LED_Level == 4)
	{
		RF_CK_LED_LEVEL4(RF_CK_LED_display_step);
	}
	else if (RF_CK_LED_Level == 5)
	{
		RF_CK_LED_LEVEL5(RF_CK_LED_display_step);
	}
	else if (RF_CK_LED_Level == 6)
	{
		RF_CK_LED_LEVEL6(RF_CK_LED_display_step);
	}
	else if (RF_CK_LED_Level == 7)
	{
		RF_CK_LED_LEVEL7(RF_CK_LED_display_step);
	}
	else if (RF_CK_LED_Level == 8)
	{
		RF_CK_LED_LEVEL8(RF_CK_LED_display_step);
	}
	else if (RF_CK_LED_Level == 9)
	{
		RF_CK_LED_LEVEL9(RF_CK_LED_display_step);
	}
	else if (RF_CK_LED_Level == 10)
	{
		RF_CK_LED_LEVEL10(RF_CK_LED_display_step);
	}

	RF_CK_LED_display_step++;
	if (RF_CK_LED_display_step > 4)
	{
		RF_CK_LED_display_step = 1;
	}
}

/*				RF Level		*/
unsigned char getRFLevel(void)
{
	unsigned char level;
	float adctemp;
	CKDIV = 0x04;
	clr_ADCF;
	set_ADCS;
	while(ADCF == 0);
	CKDIV = 0x00;
	adctemp = (ADCRH<<4) + ADCRL;
	adctemp = adctemp/4096.0 * VDD_Voltage;
	/*if (adctemp > 2380)
	{
		return 24;
	}
	else if (adctemp > 2280)
	{
		return 23;
	}
	else if (adctemp > 2180)
	{
		return 22;
	}
	else if (adctemp > 2080)
	{
		return 21;
	}
	else if (adctemp > 1980)
	{
		return 20;
	}
	else if (adctemp > 1880)
	{
		return 19;
	}
	else if (adctemp > 1780)
	{
		return 18;
	}
	else if (adctemp > 1680)
	{
		return 17;
	}
	else if (adctemp > 1580)
	{
		return 16;
	}
	else if (adctemp > 1480)
	{
		return 15;
	}	
	else if (adctemp > 1380)
	{
		return 14;
	}	
	else if (adctemp > 1280)
	{
		return 13;
	}	
	else if (adctemp > 1180)
	{
		return 12;
	}	
	else if (adctemp > 1080)
	{
		return 11;
	}		
	else*/ if (adctemp > 1200)
	{
		return 10;
	}
	else if (adctemp > 1120)
	{
		return 9;
	}
	else if (adctemp > 1040)
	{
		return 8;
	}	
	else if (adctemp > 920)
	{
		return 7;
	}	
	else if (adctemp > 840)
	{
		return 6;
	}	
	else if (adctemp > 740)
	{
		return 5;
	}
	else if (adctemp > 640)
	{
		return 4;
	}	
	else if (adctemp > 540)
	{
		return 3;
	}	
	else if (adctemp > 480)
	{
		return 2;
	}	
	else if (adctemp > 80)
	{
		return 1;
	}
	return 0;
}

void checkbyself(void)
{
	RF_CK_LED_Level = 10;	//1,4,7,8
	RF_CK_LED_display_step = 1;
	RF_SET_LED
	CK_SET_LED
	MOTOR_SET_RUN
	BUZZER_CLR_RUN
	setSixLedFreq(1,100);
	setTenLedLevel();
	systime_cnt = 0;
	while(systime_cnt < 100);
	CK_CLR_LED;
	MOTOR_CLR_RUN
	BUZZER_CLR_RUN
	
	setSixLedFreq(0,0);
	RF_CK_LED_Level = 0;
	RF_CK_LED_display_step = 1;
	setTenLedLevel();
}

//========================================================================================================
void main(void)
{
	unsigned char valuetemp;
    Set_All_GPIO_Quasi_Mode;
//	InitialUART1_Timer3(115200);
//	TI_1 = 1;
	/*init 5ms timer	*/
	systime_cnt = 0;
	Timer0_Init();
	Timer1_Init();	
	P13_PushPull_Mode;
	P14_PushPull_Mode;
	P15_PushPull_Mode;
	P17_PushPull_Mode;	
	valuetemp = 0;
	systime_cnt = 0;
	get_battery_power();
	RF_ADC_INPUT_INIT;
	checkbyself();	
	MODE_SWITCH_KEY_INIT
	SIX_LED_KEY_INIT
	CK_INPUT_INIT
	RF_CK_MODE = RF_MODE;
	RF_CK_ISLONGPRESS = 0;
	MOTOR_BUZZER_MODE = BUZZER_MODE;
	SIX_LED_ON_OFF = OFF;
	SIX_LED_ISLONGPRESS = 0;
	SIX_LED_Freq = 0;
	isSIX_LED_ENTER_FLASH_MODE = 0;
	RF_CK_LED_Level = 0;
	RF_CK_LED_display_step = 1;
	RF_CK_RUN_CNT = 0;
	RF_Level_add_cnt = 0;
	RF_Level_del_cnt = 0;
	RF_Level = 0;
	RF_Level_Debounce_cnt = 1;
    while (1)
	{
		if (is_5ms_Flag)
		{
			is_5ms_Flag = 0;
			//check RF CK MODE KEY
			valuetemp = get_RF_CK_Key();
			if (valuetemp& 0x01)
			{
				/*	short key	*/
				if (MOTOR_BUZZER_MODE == BUZZER_MODE)
				{
					MOTOR_BUZZER_MODE = MOTOR_MODE;
					if (RF_CK_RUN_CNT > 0)
					{
						MOTOR_SET_RUN;
						BUZZER_CLR_RUN;
					}
				}
				else
				{
					MOTOR_BUZZER_MODE = BUZZER_MODE;
					if (RF_CK_RUN_CNT > 0)
					{
						MOTOR_CLR_RUN;
						BUZZER_SET_RUN;
					}
				}
			}
			else if (valuetemp & 0x02)
			{
				/*	long key	*/
				if (RF_CK_MODE == RF_MODE)
				{
					RF_CK_MODE = CK_MODE;
					CK_SET_LED;
					RF_CLR_LED;
				}
				else
				{
					RF_CK_MODE = RF_MODE;
					CK_CLR_LED;
					RF_SET_LED;
				}
				RF_Level_Debounce_cnt = 1;
				RF_Level = 0;
			}
			
			valuetemp = get_SIX_LED_Key();
			#if 0
			/*
			0: Normal off
			1: Normal on
			2: 1HZ flash
			3: 2hz flash
			4: 3hz flash
			5: 5hz flash
			6: 7hz flash
			7: 10hz flash
			*/
			if (valuetemp & 0x01)
			{
				//short press
				if (SIX_LED_ON_OFF == OFF)
				{
					SIX_LED_ON_OFF = ON;
					P11_PushPull_Mode;
					if (SIX_LED_Freq == 0)
					{
						SIX_LED_Freq = 1;//enter normal on
						
						PWM1_P11_OUTPUT_DISABLE;
						clr_PWMRUN;
						P11_PushPull_Mode;
						P11 = 1;
					}
					else if (SIX_LED_Freq >= 2)
					{
						P11 = 1;
					}
				}
				else
				{
					if (SIX_LED_Freq == 1)
					{
						SIX_LED_Freq = 0;//enter normal off
					}
					SIX_LED_ON_OFF = OFF;
					
					PWM1_P11_OUTPUT_DISABLE;
					clr_PWMRUN;
					P11_PushPull_Mode;
					P11 = 0;
				}
			}
			else if (valuetemp & 0x02)
			{
				if (SIX_LED_ON_OFF == ON)
				{
					if (SIX_LED_Freq == 1)
					{
						SIX_LED_Freq = 2;	//1hz start
						
						PWM1_P11_OUTPUT_DISABLE;
						clr_PWMRUN;
						P11_PushPull_Mode;
						P11 = 1;
					}
					else if (SIX_LED_Freq == 2)
					{
						SIX_LED_Freq = 3;	//1hz over
						
						PWM1_P11_OUTPUT_DISABLE;
						clr_PWMRUN;
						P11_PushPull_Mode;
						P11 = 1;
					}
					else if (SIX_LED_Freq == 3)
					{
						SIX_LED_Freq = 4;	//2hz start
					}
					else if (SIX_LED_Freq == 4)
					{
						SIX_LED_Freq = 5;	//5hz
					}
					else if (SIX_LED_Freq == 5)
					{
						SIX_LED_Freq = 6;	//7hz
					}
					else if (SIX_LED_Freq == 6)
					{
						SIX_LED_Freq = 7;	//10hz
					}
					else if (SIX_LED_Freq == 7)
					{
						SIX_LED_Freq = 1;
					}
					P11 = 1;
				}
			}
			else if (valuetemp & 0x04)
			{
				SIX_LED_ON_OFF = OFF;
				P11 = 0;
			}
			if (SIX_LED_ON_OFF == ON)
			{

			}
			#endif
			#if 1
			if (valuetemp & 0x01)
			{
				// six led turn on or turn off
				if (SIX_LED_ON_OFF == OFF)
				{
					SIX_LED_ON_OFF = ON;
					if (SIX_LED_Freq < 2)
					{
						SIX_LED_Freq = 0;
						setSixLedFreq(0,100);
						isSIX_LED_ENTER_FLASH_MODE = 0;
					}
					else
					{
						setSixLedFreq(SIX_LED_Freq,50);
						isSIX_LED_ENTER_FLASH_MODE = 1;
					}
				}
				else
				{
					SIX_LED_ON_OFF = OFF;
					isSIX_LED_ENTER_FLASH_MODE = 0;
					setSixLedFreq(0,0);
				}
			}
			else if (valuetemp & 0x02)
			{
				/*	long press:change freq level	*/
				if (SIX_LED_ON_OFF == ON)
				{
					isSIX_LED_ENTER_FLASH_MODE = 1;
					if (SIX_LED_Freq == 0)	//1HZ turn on
					{
						setSixLedFreq(0,100);
					}
					else if (SIX_LED_Freq == 1)
					{
						setSixLedFreq(0,0);
					}
					else if (SIX_LED_Freq >=2)
					{
						setSixLedFreq(SIX_LED_Freq,50);
					}
					SIX_LED_Freq += 1;
					if (SIX_LED_Freq > 12)
					{
						SIX_LED_Freq = 0;
						SIX_LED_ON_OFF = OFF;
						setSixLedFreq(0,100);
					}
				}
				else
				{
					if (isSIX_LED_ENTER_FLASH_MODE == 1)
					{
						SIX_LED_ON_OFF = ON;
						setSixLedFreq(SIX_LED_Freq,100);
					}
				}
			}
			else if (valuetemp & 0x04)
			{
				//long press release
				if (SIX_LED_Freq == 2)
				{
					SIX_LED_ON_OFF = OFF;
					SIX_LED_Freq = 0;
				}
				if (SIX_LED_ON_OFF == OFF)
				{
					isSIX_LED_ENTER_FLASH_MODE = 0;
				}
			}
			#endif
			
			
			if (RF_CK_MODE == RF_MODE)
			{
				/*		check rf level	*/
				valuetemp = getRFLevel();
				if (valuetemp >= 10)
					valuetemp = 10;
				if (valuetemp != RF_Level)
				{
					if (valuetemp > RF_Level)
					{
						RF_Level_add_cnt ++;
						RF_Level_del_cnt = 0;
						if (RF_Level_add_cnt > RF_Level_Debounce_cnt)	//2
						{
							RF_Level++;
							RF_Level_add_cnt = 0;
						}
					}
					else if (RF_Level > valuetemp)
					{
						RF_Level_del_cnt ++;
						RF_Level_add_cnt = 0;
						if (RF_Level_del_cnt > RF_Level_Debounce_cnt)	//2
						{
							RF_Level--;
							RF_Level_del_cnt = 0;
						}
					}
				
				}
				else
				{
//					if (RF_Level_add_cnt > 0)
//						RF_Level_add_cnt--;
//					if (RF_Level_del_cnt > 0)
//						RF_Level_del_cnt--;
					RF_Level_add_cnt = 0;
					RF_Level_del_cnt = 0;
					if (RF_Level < 10)
						RF_Level_Debounce_cnt = 1;
				}
			}
			else 
			{
				/*		check ck level	*/
				if (P30 == 1)
				{
					RF_Level = 10;
				}
				else 
					RF_Level = 0;
			}
			if (RF_CK_LED_Level != RF_Level)
			{
				RF_CK_LED_display_step = 1;
				if (RF_Level > 10)
					RF_CK_LED_Level = 10;
				else
					RF_CK_LED_Level = RF_Level;
			}
				if (RF_Level >= 7)
				{
					if (RF_CK_RUN_CNT < 20)
						RF_CK_RUN_CNT = 20;
					if (RF_Level >= 10)
					{
						RF_CK_RUN_CNT = 20 + (RF_Level - 7) * 10;
//						RF_Level_Debounce_cnt = RF_CK_RUN_CNT / (RF_Level-7);
					}
					if (Buzzer_Motor_isRun == 0)
					{
						if (MOTOR_BUZZER_MODE == BUZZER_MODE)
						{
							MOTOR_CLR_RUN;
							BUZZER_SET_RUN;
						}
						else
						{
							MOTOR_SET_RUN;
							BUZZER_CLR_RUN;
						}
					}
					Buzzer_Motor_isRun = 1;
				}
				else
				{
					Buzzer_Motor_isRun = 0;
					if (RF_CK_RUN_CNT > 20)
						RF_CK_RUN_CNT = 20;
				}			
			if (RF_CK_RUN_CNT > 0)
			{
				RF_CK_RUN_CNT -- ;
				if (RF_CK_RUN_CNT == 0)
				{
					MOTOR_CLR_RUN
					BUZZER_CLR_RUN
					Buzzer_Motor_isRun = 0;
				}
			}
		}
	}
}

