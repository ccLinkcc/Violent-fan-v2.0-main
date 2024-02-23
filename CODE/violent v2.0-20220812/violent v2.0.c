#include "STC8Hxxx.h"
#include "intrins.h"
//#include "stdio.h"

sbit PWM1   = P1^0;
sbit PWM1_L = P1^1;
sbit PWM2   = P1^2;
sbit PWM2_L = P1^3;
sbit PWM3   = P1^4;
sbit PWM3_L = P1^5;
/*************************************************************************/
//��ɫLED��������
#define	LED_R	P17
#define	LED_G	P54
#define	LED_B	P16
/*************************************************************************/
#define	VBUS_SENSE		P30		//����߽����⣬0������߽��룬1������߶Ͽ�
#define	POWER_SWITCH	P31		//mos����������Դ���أ�0���Ͽ���1����ͨ
/*************************************************************************/
#define	KEY1					P37
#define	KEY_SHORTPRESSED	0x01
#define	KEY_LONGPRESSED		0x02
#define	KEY_DOUBLEPRESSED	0x04
#define	KEY_LONGPRESS_TH		100			//����������ֵ= 100 * 10ms = 1s
#define	KEY_DOUBLEPRESS_TH	15			//����˫����ֵ= 15 * 10ms = 150ms�����ε���������ʱ��С��150ms����Ϊ��˫����
unsigned char data key_stage = 0;
unsigned char data key_pressed_time = 0;
unsigned char data Trg = 0;
/*************************************************************************/
#define	BAT_ADC_CH		10		//��ѹ���ʹ��ADCͨ��10
#define	BAT_AVG_SUM		16		//16������һ��ƽ��
#define	BAT_LOW_TH		6.4		//��ѹ������ѹֵ��6.4v
#define	V_DIV_SCALE		(470.0 / (330.0 + 470.0))		//��ص�ѹ��ѹ��
#define	ADC_VREF			5.0		//ADC�ο���ѹ��5.0v
#define	CODE_TO_V_SCALE	(ADC_VREF / 1024.0) / V_DIV_SCALE
#define	BATLOW_RECOVERY_TIME	100
unsigned int bat_code[BAT_AVG_SUM];
unsigned char bat_sample_cnt = 0;
float bat_v;		//��ص�ѹ����λ��v
unsigned int batlow_recovery_time = 0;
unsigned int batlow_cnt = 0;
/*************************************************************************/
#define	THROTTLE_SUM	4
unsigned char throttle_index = 0;
unsigned char code THROTTLE[4] = {0, 60, 150, 250};
/*************************************************************************/
#define	BEEBEE_VOLUME				40	//��ʾ������(0~250)
#define	PWM_START_VALUE			40	//��������(20~250)
#define	MOTOR_STUCK_TIMEOUT	80  //�����ת50*10ms=500ms�󣬹رյ��
#define	MOTOR_RUNTIME_TH		30000		//�������ʱ����ֵ30000*10ms = 5min�������ʱ��֮��û���κΰ����������Զ��ػ�
#define	MOTOR_IDLETIME_TH		3000		//�������ʱ����ֵ3000*10ms = 30sec��	�����ʱ��֮��û���κΰ����������Զ��ػ�
unsigned char	PWM_value;	//����PWMռ�ձȵ�ֵ
bit	B_start = 0;					//����ģʽ
bit	B_RUN = 0;						//���б�־
bit	B_Timer1_OF = 0;
unsigned int data PhaseTimeTmp[8];			//8������ʱ��, ��sum/16����30�ȵ�Ƕ�
unsigned char data step = 0;						//���ಽ��
unsigned char data demag_step = 0;			//0:�Ѿ�����, 1:��Ҫ����, 2:��������,
unsigned char data motor_stuck_time = 0;		//��ת��ʱ����
unsigned int motor_time = 0;						//�������ʱ��
/*************************************************************************/
bit	tick_10ms;		//10ms��ʱ��־

#define	STA_SLEEP		0x01
#define	STA_WAKING	0x02
#define	STA_RUNNING	0x04
#define	STA_BATLOW	0x08
#define	STA_CHARGING	0x10
unsigned char status = STA_WAKING;		//�ϵ��Ĭ��״̬�ǵ͹�������

void Delay1ms(unsigned char n)		//@35.00MHz
{
	unsigned char i, j;
	while(n--)
	{
		i = 35;
		j = 8;
		do
		{
			while (--j);
		} while (--i);
	}
}

void Delay1us(unsigned char us)		//@35.00MHz
{
	unsigned char i;
	do
	{
		_nop_();
		_nop_();
		i = 9;
		while (--i);
	}
	while(--us);
}
void Delay500ns(void)
{
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	_nop_();
}
unsigned int	ADC_Get10bitResult(unsigned char channel)	//channel = 0~15
{
	unsigned char i;
	ADC_CONTR = 0xC0 | channel; 
	//Delay500ns();
	//while((ADC_CONTR & ADC_FLAG) == 0)	;	//�ȴ�ADC����
	i = 255;
	while(i--)
	{
		if((ADC_CONTR & 0x20) != 0)
			break;	//�ȴ�ADC����
	}
	ADC_CONTR &= ~0x20;
	return	(((unsigned int)ADC_RES << 8) | ADC_RESL);
}
unsigned int bat_average(void)	//ȥ�������Сֵ����ƽ��
{
	unsigned char i;
	unsigned int min, max;
	unsigned int sum = 0;
	min = max = bat_code[0];
	for(i = 0; i < BAT_AVG_SUM; i++)
	{
		sum += bat_code[i];
		if(bat_code[i] >= max)
			max = bat_code[i];
		if(bat_code[i] <= min)
			min = bat_code[i];
	}
	sum -= (max + min);
	sum /= (BAT_AVG_SUM - 2);
	return sum;
}
void MotorStep(void) 	// �������к���
{
	switch(step)
	{
		case 0:  // AB  PWM1, PWM2_L=1
			PWMA_ENO = 0x00;	PWM1_L=0;	PWM3_L=0;
			Delay500ns();
			PWMA_ENO = 0x01;			// ��A��ĸ߶�PWM
			PWM2_L = 1;						// ��B��ĵͶ�
			ADC_CONTR = 0x80+13;	// ѡ��P0.2��ΪADC���� ��C���ѹ
			CMPCR1 = 0x8c + 0x10;	// �Ƚ����½����ж�
			break;
		case 1:  // AC  PWM1, PWM3_L=1
			PWMA_ENO = 0x01;	PWM1_L=0;	PWM2_L=0;	// ��A��ĸ߶�PWM
			Delay500ns();
			PWM3_L = 1;						// ��C��ĵͶ�
			ADC_CONTR = 0x80+12;	// ѡ��P0.1��ΪADC���� ��B���ѹ
			CMPCR1 = 0x8c + 0x20;	// �Ƚ����������ж�
			break;
		case 2:  // BC  PWM2, PWM3_L=1
			PWMA_ENO = 0x00;	PWM1_L=0;	PWM2_L=0;
			Delay500ns();
			PWMA_ENO = 0x04;			// ��B��ĸ߶�PWM
			PWM3_L = 1;						// ��C��ĵͶ�
			ADC_CONTR = 0x80+11;	// ѡ��P0.0��ΪADC���� ��A���ѹ
			CMPCR1 = 0x8c + 0x10;	// �Ƚ����½����ж�
			break;
		case 3:  // BA  PWM2, PWM1_L=1
			PWMA_ENO = 0x04;	PWM2_L=0;	PWM3_L=0;	// ��B��ĸ߶�PWM
			Delay500ns();
			PWM1_L = 1;						// ��C��ĵͶ�
			ADC_CONTR = 0x80+13;	// ѡ��P0.2��ΪADC���� ��C���ѹ
			CMPCR1 = 0x8c + 0x20;	// �Ƚ����������ж�
			break;
		case 4:  // CA  PWM3, PWM1_L=1
			PWMA_ENO = 0x00;	PWM2_L=0;	PWM3_L=0;
			Delay500ns();
			PWMA_ENO = 0x10;			// ��C��ĸ߶�PWM
			PWM1_L = 1;						// ��A��ĵͶ�
			bat_code[bat_sample_cnt] = ADC_Get10bitResult(BAT_ADC_CH);
			ADC_CONTR = 0x80+12;		// ѡ��P0.1��ΪADC���� ��B���ѹ
			CMPCR1 = 0x8c + 0x10;	//�Ƚ����½����ж�
			break;
		case 5:  // CB  PWM3, PWM2_L=1
			PWMA_ENO = 0x10;	PWM1_L=0;	PWM3_L=0;	// ��C��ĸ߶�PWM
			Delay500ns();
			PWM2_L = 1;						// ��B��ĵͶ�
			ADC_CONTR = 0x80+11;	// ѡ��P0.0��ΪADC���� ��A���ѹ
			CMPCR1 = 0x8c + 0x20;	// �Ƚ����������ж�
			break;
		default:
			break;
	}
	if(B_start)
		CMPCR1 = 0x8C;	// ����ʱ��ֹ�½��غ��������ж�               
}                                                                                                                                                                       
void MotorStart(void) // ǿ�Ƶ����������
{
	unsigned int timer,i;
	CMPCR1 = 0x8C;	// �رȽ����ж�
	PWM_value  = 10;	// ��ʼռ�ձ�, ���ݵ����������
	PWMA_CCR1L = PWM_value;
	PWMA_CCR2L = PWM_value;
	PWMA_CCR3L = PWM_value;
	step = 0;
	timer = 200;		//���ȵ������
	MotorStep();
	Delay1ms(100);	//Delay1ms(250);// ��ʼλ��	
	while(1)
	{
		for(i=0; i<timer; i++)//���ݵ����������, ���ת�ٵȵȵ������������ٶ�
		{
			Delay1us(100);
		}
		++step;
		step %= 6;
		PWM_value += 2;
		if(PWM_value > PWM_START_VALUE)
			PWM_value = PWM_START_VALUE;
		PWMA_CCR1L = PWM_value;
		PWMA_CCR2L = PWM_value;
		PWMA_CCR3L = PWM_value;	 
		MotorStep();
		timer -= timer/15+1;		
		if(timer < 25)
			return;	
	}
}
void MotorStop(void)
{
	B_RUN = 0;
	throttle_index = 0;
	PWM_value = 0;
	CMPCR1 = 0x8C;	// �رȽ����ж�
	PWMA_ENO  = 0;	// �ر�PWM���
	PWMA_CCR1L = 0;
	PWMA_CCR2L = 0;
	PWMA_CCR3L = 0;
	PWM1_L=0;
	PWM2_L=0;
	PWM3_L=0;
}

void KeyScan(void)
{
	switch(key_stage)
	{
		case 0:
		{
			if(KEY1 == 0)
				key_stage = 1;
			break;
		}
		case 1:
		{
			if(KEY1 == 0)
			{
				if(++key_pressed_time == KEY_LONGPRESS_TH)
				{
					Trg = KEY_LONGPRESSED;
					key_stage = 3;
				}
			}
			else
			{
				key_stage = 2;
				key_pressed_time = 0;
			}
			break;
		}
		case 2:
		{
			if(KEY1 == 1)
			{
				if(++key_pressed_time == KEY_DOUBLEPRESS_TH)
				{
					Trg = KEY_SHORTPRESSED;
					key_stage = 3;
				}
			}
			else
			{
				Trg = KEY_DOUBLEPRESSED;
				key_stage = 3;
			}
			break;
		}
		case 3:
		{
			if(KEY1 == 1)
			{
				key_stage = 0;
				key_pressed_time = 0;
			}
			break;
		}
		default:
			break;
	}
}
void GPIOInit(void)
{
	//P17:LED-R, P16:LED-B, P15:LIN3, P14:HIN3
	//P13:LIN2, P12:HIN2, P11:LIN1, P10:HIN1
	P1M0 = 0xFF;	
	P1M1 = 0x00;
	P1 = 0xC0;
	//P37:KEY1, P36:CMP-, P35:ADC13, P34:ADC12
	//P33:ADC11, P32:ADC10, P31:POWER_SWITCH, P30:VBUS_SENSE
	P3M0 = 0x00;	
	P3M1 = 0x7C;
	P3 = 0x81;
	//P54:LED-G
	P5M0 = 0x10;	
	P5M1 = 0x00;
	P5 = 0x10;
}
void ADCInit(void)	//ADC��ʼ������(Ϊ��ʹ��ADC��������Ƚ����ź�, ʵ��û������ADCת��)
{
	ADC_CONTR = 0x80 + 13;	//ADC PowerOn + channel
	ADCCFG = 0x21;
	P_SW2 |=  0x80;	//����XSFR
	ADCTIM = 0x20 + 20;
}
void PWMInit(void)
{
	P_SW2 |= 0x80;		//SFR enable   

	PWM1   = 0;
	PWM1_L = 0;
	PWM2   = 0;
	PWM2_L = 0;
	PWM3   = 0;
	PWM3_L = 0;

	PWMA_PSCR = 3;		// Ԥ��Ƶ�Ĵ���, ��Ƶ Fck_cnt = Fck_psc/(PSCR[15:0}+1), ���ض���PWMƵ�� = SYSclk/((PSCR+1)*(AAR+1)), �������PWMƵ�� = SYSclk/((PSCR+1)*(AAR+1)*2).
	PWMA_DTR  = 24;		// ����ʱ������, n=0~127: DTR= n T,   0x80 ~(0x80+n), n=0~63: DTR=(64+n)*2T,  
						//				0xc0 ~(0xc0+n), n=0~31: DTR=(32+n)*8T,   0xE0 ~(0xE0+n), n=0~31: DTR=(32+n)*16T,
	PWMA_ARR    = 255;	// �Զ���װ�ؼĴ���,  ����PWM����
	PWMA_CCER1  = 0;
	PWMA_CCER2  = 0;
	PWMA_SR1    = 0;
	PWMA_SR2    = 0;
	PWMA_ENO    = 0;
	PWMA_PS     = 0;
	PWMA_IER    = 0;
//	PWMA_ISR_En = 0;

	PWMA_CCMR1  = 0x68;		// ͨ��ģʽ����, PWMģʽ1, Ԥװ������
	PWMA_CCR1   = 0;			// �Ƚ�ֵ, ����ռ�ձ�(�ߵ�ƽʱ����)
	PWMA_CCER1 |= 0x05;		// �����Ƚ����, �ߵ�ƽ��Ч
	PWMA_PS    |= 0;		// ѡ��IO, 0:ѡ��P1.0 P1.1, 1:ѡ��P2.0 P2.1, 2:ѡ��P6.0 P6.1, 
//	PWMA_ENO   |= 0x01;		// IO�������,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P

	PWMA_CCMR2  = 0x68;		// ͨ��ģʽ����, PWMģʽ1, Ԥװ������
	PWMA_CCR2   = 0;			// �Ƚ�ֵ, ����ռ�ձ�(�ߵ�ƽʱ����)
	PWMA_CCER1 |= 0x50;		// �����Ƚ����, �ߵ�ƽ��Ч
	PWMA_PS    |= (0<<2);	// ѡ��IO, 0:ѡ��P1.2 P1.3, 1:ѡ��P2.2 P2.3, 2:ѡ��P6.2 P6.3, 
//	PWMA_ENO   |= 0x04;		// IO�������,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P

	PWMA_CCMR3  = 0x68;		// ͨ��ģʽ����, PWMģʽ1, Ԥװ������
	PWMA_CCR3   = 0;			// �Ƚ�ֵ, ����ռ�ձ�(�ߵ�ƽʱ����)
	PWMA_CCER2 |= 0x05;		// �����Ƚ����, �ߵ�ƽ��Ч
	PWMA_PS    |= (0<<4);	// ѡ��IO, 0:ѡ��P1.4 P1.5, 1:ѡ��P2.4 P2.5, 2:ѡ��P6.4 P6.5, 
//	PWMA_ENO   |= 0x10;		// IO�������,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P

	PWMA_BKR    = 0x80;		// �����ʹ�� �൱���ܿ���
	PWMA_CR1    = 0x81;		// ʹ�ܼ�����, �����Զ���װ�ؼĴ�������, ���ض���ģʽ, ���ϼ���,  bit7=1:д�Զ���װ�ؼĴ�������(�����ڲ��ᱻ����), =0:ֱ��д�Զ���װ�ؼĴ�����(���ڿ��ܻ��ҵ�)
	PWMA_EGR    = 0x01;		// ����һ�θ����¼�, ��������������Ƶ������, װ��Ԥ��Ƶ�Ĵ�����ֵ
}
void CMPInit(void)	//�Ƚ�����ʼ������
{
	CMPCR1 = 0x8C;	// 1000 1100 �򿪱Ƚ�����P3.6��Ϊ�Ƚ����ķ�������ˣ�ADC������Ϊ������� 
	CMPCR2 = 60;		// 60��ʱ���˲����ȽϽ���仯��ʱ������, 0~63
}
void Timer0Init(void)	//Timer0��ʼ�����������ڼ�⵽�����ʲôʱ�����Լ������ʲôʱ��ʼ������
{
	AUXR &= 0x7F;		//��ʱ��ʱ��12Tģʽ
	TMOD &= 0xF0;		//���ö�ʱ��ģʽ
	TL0 = 0x00;		//���ö�ʱ��ʼֵ
	TH0 = 0x00;		//���ö�ʱ��ʼֵ
	TF0 = 0;			//���TF0��־
	ET0 = 1; 			// ����ET0�ж�
	//TR0 = 1; // �򿪶�ʱ��0
}
void Timer1Init(void)	//timer1��ʼ������,���ڼ��㻻��֮���ʱ��
{
	AUXR &= 0xBF;		//��ʱ��ʱ��12Tģʽ
	TMOD &= 0x0F;		//���ö�ʱ��ģʽ
	TL1 = 0x00;		//���ö�ʱ��ʼֵ
	TH1 = 0x00;		//���ö�ʱ��ʼֵ
	TF1 = 0;		//���TF1��־
	ET1 = 1;		//������ж�
	TR1 = 1;		//��ʱ��1��ʼ��ʱ
}
void Timer2Init(void)		//10����@35.00MHz������10MSʱ���ź�
{
	AUXR &= 0xFB;		//��ʱ��ʱ��12Tģʽ
	T2L = 0x11;			//���ö�ʱ��ʼֵ
	T2H = 0x8E;			//���ö�ʱ��ʼֵ
	IE2 |= 0x04;    //ʹ�ܶ�ʱ���ж�
	AUXR |= 0x10;		//��ʱ��2��ʼ��ʱ
}
void TM0_Isr(void) interrupt 1	//timer0�жϺ���
{
	TR0 = 0;	// Timer0ֹͣ����
	if(demag_step == 1)		//�����Ҫ����. ÿ�μ�⵽��0�¼����һ���ж�Ϊ30�Ƚ���ʱ, ����������ʱ.
	{
		demag_step = 2;		//1:��Ҫ����, 2:��������, 0�Ѿ�����
		if(B_RUN)	//�����������
		{
			if(++step >= 6)
				step = 0;
			MotorStep();
		}
		//����ʱ��, �������Ȧ(���)������С��0�Ĺ�����, ���ַ��綯��, ����Խ������ʱ��Խ��, ��0���Ҫ�����ʱ��֮��
		//100%ռ�ձ�ʱʩ�ӽ��ظ���, �����������, ����ʾ����������ʱ��.
		//ʵ����, ֻҪ�ڻ������ʱ��ʮus�ż�����, �Ϳ�����
		TH0 = (unsigned char)((65536UL - 40*2) >> 8);	//װ��������ʱ
		TL0 = (unsigned char)(65536UL - 40*2);
		TR0 = 1;	//Timer0��ʼ����
	}
	else if(demag_step == 2)
	{
		demag_step = 0;		//1:��Ҫ����, 2:��������, 0�Ѿ�����
	}
}
void TM1_Isr(void) interrupt 3	//timer1�жϺ���
{
	B_Timer1_OF = 1;	//�����־
}

void TM2_Isr(void) interrupt 12	//Timer2�жϺ���
{
	tick_10ms = 1;	//10ms��ʱ��־
	KeyScan();
}
void CMP_Isr(void) interrupt 21	//�Ƚ����жϺ���, ��⵽���綯�ƹ�0�¼�
{
	static unsigned char data TimeIndex = 0;		// ����ʱ�䱣������
	unsigned int data PhaseTime;			// ����ʱ�����
	unsigned char data i;
	CMPCR1 &= ~0x40;		// �Ƚ����ж����������жϱ�־λ
	if(demag_step == 0)	// ���ź�ż���0�¼�,   demag_step=1:��Ҫ����, =2:��������, =0�Ѿ�����
	{
		TR1 = 0;				//Timer1ֹͣ����
		if(B_Timer1_OF)	//�л�ʱ����(Timer1)�����
		{
			B_Timer1_OF = 0;
			PhaseTime = 8000;	//����ʱ�����8ms, 2212���12V��ת�����130us�л�һ��(200RPS 12000RPM), 480mA
		}
		else
		{
			PhaseTime = (((unsigned int)TH1 << 8) + TL1) >> 1;	//��λΪ1us
			if(PhaseTime >= 8000)	//����ʱ�����8ms, 2212���12V��ת�����130us�л�һ��(200RPS 12000RPM), 480mA
				PhaseTime = 8000;	
		}
		TH1 = 0;	
		TL1 = 0;
		TR1 = 1;	//Timer1��ʼ����

		PhaseTimeTmp[TimeIndex] = PhaseTime;	//����һ�λ���ʱ��
		TimeIndex = (++TimeIndex)%8;		//�ۼ�8��
		for(PhaseTime=0, i=0; i<8; i++)	//��8�λ���ʱ���ۼӺ�
			PhaseTime += PhaseTimeTmp[i];	
		PhaseTime = PhaseTime >> 4;			//��8�λ���ʱ���ƽ��ֵ��һ��, ��30�ȵ�Ƕ�
		if((PhaseTime >= 40) && (PhaseTime <= 1000))
			motor_stuck_time = 0;
		if(PhaseTime >= 80)
			PhaseTime -= 40;							//���������˲�����������ͺ�ʱ��
		else
			PhaseTime  = 20;
		
		// PhaseTime = 20;	//ֻ��20us, �����ͺ�����, ���ڼ���˲�����������ͺ�ʱ��
		TR0 = 0;		//Timer0ֹͣ����
		PhaseTime  = PhaseTime << 1;	//2������1us
		PhaseTime = 0 - PhaseTime;
		TH0 = (unsigned char)(PhaseTime >> 8);		//װ��30�Ƚ���ʱ
		TL0 = (unsigned char)PhaseTime;
		TR0 = 1;					//Timer0��ʼ����
		demag_step = 1;		//1:��Ҫ����, 2:��������, 0�Ѿ�����
		//��һ�����ڶ�ʱ��0�жϺ����н��л���
	}
}
void INT3_Isr() interrupt 11 //�����ж�
{
}
void INT4_Isr() interrupt 16 //VBUS�����ж�
{
	//IAP_CONTR = 0x60;
	status |= STA_CHARGING;
	status |= STA_SLEEP;
	status &= ~STA_RUNNING;
}
void SelfCheckBeeBee(void)
{
	PWMA_PSCR = 16;
	
	PWMA_CCR1L = BEEBEE_VOLUME;
	PWM2_L = 1;
	PWMA_ENO = 0x01;
	Delay1ms(250);
	PWMA_CCR1L = 0;
	PWM2_L = 0;
	Delay1ms(50);
	
	PWMA_PSCR = 14;
	
	PWMA_CCR2L = BEEBEE_VOLUME;
	PWM3_L = 1;
	PWMA_ENO = 0x04;
	Delay1ms(250);
	PWMA_CCR2L = 0;
	PWM3_L = 0;
	Delay1ms(50);
	
	PWMA_PSCR = 12;
	
	PWMA_CCR3L = BEEBEE_VOLUME;
	PWM1_L = 1;
	PWMA_ENO = 0x10;
	Delay1ms(250);
	PWMA_CCR3L = 0;
	PWM1_L = 0;
	Delay1ms(50);
	
	PWMA_ENO = 0x00;
	PWMA_PSCR = 3;
}
void BatLowBeeBee()
{
	PWMA_PSCR = 16;
	
	PWMA_CCR1L = BEEBEE_VOLUME;
	PWM2_L = 1;
	PWMA_ENO = 0x01;
	Delay1ms(100);
	PWMA_ENO = 0x00;
	Delay1ms(100);
	PWMA_ENO = 0x01;
	Delay1ms(100);
	PWMA_ENO = 0x00;
	PWMA_CCR1L = 0;
	PWM2_L = 0;
	
	PWMA_PSCR = 3;
}
void KeyHandle(void)
{
	if(Trg == 0)
		return;
	motor_time = 0;
	switch(Trg)
	{
		case KEY_SHORTPRESSED:		//�̰��л���λ
		{
			if(status & STA_BATLOW)	//��ص�ѹ��ʱ����ֹ��������
			{
				BatLowBeeBee();		//���ŵ�ص�ѹ�͵���ʾ��
			}
			else
			{
				if(++throttle_index == 1)
					B_start = 1;
				if(throttle_index == 4)
				{
					throttle_index = 0;
					MotorStop();
				}
			}
			break;
		}
		case KEY_LONGPRESSED:			//���������ػ�
		{
			MotorStop();
			status &= ~STA_RUNNING;
			status |= STA_SLEEP;
			break;
		}
		case KEY_DOUBLEPRESSED:		//˫������ֹͣ���
		{
			MotorStop();
			break;
		}
		default:
			break;
	}
	Trg = 0;
}
void LEDControl(void)
{
	static unsigned int LED_cnt = 0;
	if(++LED_cnt == 100)
		LED_cnt = 0;
	if(status & STA_BATLOW)
	{
		LED_R = 1;
		LED_G = 1;
		LED_B = 0;
	}
	else if(status & STA_RUNNING)
	{
		if(throttle_index == 0)
		{
			LED_R = 0;
			LED_G = 1;
			LED_B = 1;
		}
		else
		{
			if(LED_cnt < (10 + throttle_index * 20))
			{
				LED_R = (LED_cnt / 10) % 2;
			}
			LED_G = 1;
			LED_B = 1;
		}
	}
}
void main(void)
{
	unsigned char i;
	GPIOInit();
	PWMInit();
	ADCInit();
	CMPInit();
	//UartInit();
	Timer0Init();	// Timer0��ʼ������
	Timer1Init();	// Timer1��ʼ������
	Timer2Init();	// Timer4��ʼ������
	INTCLKO |= 0x20;  //ʹ��INT3�½����ж�
	EA  = 1; 					//�����ж�
	while (1)
	{
		if(status & STA_RUNNING)	//����
		{
			if(tick_10ms)		//10msʱ϶
			{
				tick_10ms = 0;
				if(B_RUN)	//����������
				{
					PWM_value = (unsigned char)(PWM_value * 0.9 + THROTTLE[throttle_index] * 0.1);
					PWMA_CCR1L = PWM_value;
					PWMA_CCR2L = PWM_value;
					PWMA_CCR3L = PWM_value;
					if(++motor_stuck_time == MOTOR_STUCK_TIMEOUT)	//��ת��ʱ
						MotorStop();
				}
				else
				{	
					if(B_start)
					{
						B_start = 0;
						for(i=0; i<8; i++)
							PhaseTimeTmp[i] = 400;
						MotorStart();		// �������
						B_start = 0;
						demag_step = 0;		// ��ʼ����ʱ
						CMPCR1 &= ~0x40;	// ����жϱ�־λ
						if(step & 0x01)
							CMPCR1 = 0xAC;		//�������ж�
						else
							CMPCR1 = 0x9C;		//�½����ж�
						B_RUN = 1;
						Delay1ms(250);	//��ʱһ��, ����������
						Delay1ms(250);
						motor_stuck_time = 0;		//������ʱʱ�� 125*4 = 500ms
					}
					bat_code[bat_sample_cnt] = ADC_Get10bitResult(BAT_ADC_CH);
				}
				if(++motor_time == (throttle_index==0?MOTOR_IDLETIME_TH:MOTOR_RUNTIME_TH))
				{
					motor_time = 0;
					MotorStop();
					status &= ~STA_RUNNING;
					status |= STA_SLEEP;
				}
				if(++bat_sample_cnt == BAT_AVG_SUM)
				{
					bat_sample_cnt = 0;
					bat_v = bat_average() * CODE_TO_V_SCALE;	//����õ�ǰ��ѹֵ
					//printf("batv:%.2f\n", bat_v + 0.005);
					if((status & STA_BATLOW) == 0)
					{
						if(bat_v < BAT_LOW_TH)
						{
							if(++batlow_cnt == 64)
							{
								batlow_cnt = 0;
								LED_B = 0;
								MotorStop();
								status |= STA_BATLOW;
								batlow_recovery_time = 0;
							}
						}
						else
							batlow_cnt = 0;
					}
					else 
					{
						if(bat_v > BAT_LOW_TH)
						{
							if(++batlow_cnt == 32)
							{
								batlow_cnt = 0;
								status &= ~STA_BATLOW;
							}
						}
						else
							batlow_cnt = 0;
					}
				}
				LEDControl();
			}
			KeyHandle();
		}
		if(status & STA_SLEEP)		//����
		{
			LED_R = 1;
			LED_G = 1;
			LED_B = 1;
			if(KEY1 == 0)			//���������û�ɿ�������ʱ�ȴ��ɿ�
			{
				Delay1ms(200);
				Delay1ms(200);
				Delay1ms(200);
			}
			POWER_SWITCH = 0;	// �ر�MOS�������˵�Դ
			INTCLKO &= ~0x40;	// �ر�INT4�½����ж�
			ADC_CONTR = 0x00;	// �ر�Ƭ��ADC��Դ
			CMPCR1 = 0;				// �ر�Ƭ�ڱȽ���
			IE2 &= ~0x04;    //ʹ�ܶ�ʱ���ж�
			PCON = 0x02;			// MCU�������ģʽ
			//����������ߣ��жϻ��Ѻ�ִ�����жϺ�����Ҳ�ص���
			_nop_();
			_nop_();
			IAP_CONTR = 0x60;
		}
		if(status & STA_WAKING)		//����
		{
			unsigned char cnt = 0;
			while(KEY1 == 0)	
			{
				Delay1ms(200);
				LED_G = 0;
				Delay1ms(200);
				LED_G = 1;
				cnt++;
				if(cnt > 3)
					LED_R = 0;
			}
			if(cnt > 3)
			{
				status &= ~STA_SLEEP;
				status |= STA_WAKING;
				if(VBUS_SENSE == 0)		//������ڳ�磨����߽��룩
				{
					POWER_SWITCH = 0;
					status |= STA_CHARGING;
					status |= STA_SLEEP;		//��������
				}
				else									//δ�ڳ��							
				{		
					CMPCR1 = 0x8C;		//��ʼ���Ƚ���
					ADC_CONTR = 0x80;	//��ADC��Դ
					INTCLKO |= 0x40;	//ʹ��INT4�½����ж�
					POWER_SWITCH = 1;	//��MOS�������˵�Դ
					IE2 |= 0x04;    //ʹ�ܶ�ʱ���ж�
					LED_R = 0;
					key_stage = 0;
					key_pressed_time = 0;
					Trg = 0;
					SelfCheckBeeBee();	  	//����Լ�
					status |= STA_RUNNING;	//��������״̬
				}
			}
			else
			{
				status &= ~STA_WAKING;
				status |= STA_SLEEP;
			}
			status &= ~STA_WAKING;//�˳�����״̬
		}
	}
}


