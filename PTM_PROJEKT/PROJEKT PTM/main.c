#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_usart.h"

unsigned int u=0;
float ADC_Result;
float napiecie;
int zmienna=0x000;
int wlaczony=1;
unsigned int i=0;
unsigned int j=0;
unsigned int k=0;
unsigned int l=0;
int x;
int z=0;
u8 audio;
int a=0;
int b=0;
int dlugosc[2]={465658, 379448};

/*const u8 rawAudio[610676] = {
/* C:\Users\Przemys³aw\Desktop\Eddy Wata - La Bomba.wav (2017-03-20 17:19:54)
   StartOffset: 00000000, EndOffset: 00071AF9, D³ugoœæ: 00071AFA */

const u8 u1[465658];
const u8 u2[465658];
//SEGMENT: 1-PE7, 2-PE4, 3-PE5, 4-PE6,
//a- PE8, b-PE9 ,c-PE10 , d-PE11 , e-PE12 , f-PE13 , g-PE14 ,

//plytka:
//k3<poprzedni> - pb11, k2<pauza> - pb12, k1<play> - pb13, k0<nastepny> - pb14

void setNumber(int num, int poz)
{
	GPIO_SetBits(GPIOE, GPIO_Pin_8| GPIO_Pin_9| GPIO_Pin_10| GPIO_Pin_11| GPIO_Pin_12| GPIO_Pin_13 | GPIO_Pin_14);
	if(poz == 4) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
		GPIO_SetBits(GPIOE, GPIO_Pin_6);
	}
	if(poz == 3) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
		GPIO_SetBits(GPIOE, GPIO_Pin_5);
	}
	if(poz == 2) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
		GPIO_SetBits(GPIOE, GPIO_Pin_4);
	}
	if(poz == 1) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
		GPIO_SetBits(GPIOE, GPIO_Pin_7);
	}
	if(num == 0) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13); //wlaczenie 0
	} else if(num == 1) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_9 | GPIO_Pin_10);
	} else if(num == 2) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_14); //wlaczenie 2
	} else if(num == 3) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_14); //wlaczenie 3
	} else if(num == 4) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_13 | GPIO_Pin_14); //wlaczenie 4
	} else if(num == 5) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14); //wlaczenie 5
	} else if(num == 6) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14); //wlaczenie 6
	} else if(num == 7) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10); //wlaczenie 7
	} else if(num == 8) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14); //wlaczenie 8
	} else if(num == 9) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14); //wlaczenie 9
	}
}

int main(void)
{
	SystemInit();

	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE); // zegar dla portu GPIO z którego wykorzystany zostanie pin jako wejœcie ADC (PA1 )
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE); // zegar dla modu³u ADC1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC , ENABLE); // zegar dla modu³u DAC
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	//GPIO
	GPIO_InitTypeDef  GPIO_InitStructure;
	//diody uC
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//przyciski na plytce @@@@@@@@@@@DOZMIANY@@@@@@@@@@@@@@@@@@@@@@@
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//!!!!!!!!!
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//inicjalizacja wyjsc do wyswietlacza/diody
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8| GPIO_Pin_9| GPIO_Pin_10| GPIO_Pin_11| GPIO_Pin_12| GPIO_Pin_13| GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//inicjalizacja wejœcia ADC
	GPIO_InitStructure. GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure. GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure. GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//inicjalizacja wyjœcia DAC
	GPIO_InitStructure. GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure. GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure. GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure. GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//SPI
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// SCK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	// MISO
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	// MOSI
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);


	//ADC
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_CommonInitStructure. ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure. ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure. ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure. ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure. ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure. ADC_ScanConvMode = DISABLE;
	ADC_InitStructure. ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure. ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure. ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure. ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure. ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_84Cycles);
	ADC_Cmd(ADC1, ENABLE);

	//DAC
	DAC_InitTypeDef DAC_InitStructure;
	DAC_InitStructure. DAC_Trigger = DAC_Trigger_None;
	DAC_InitStructure. DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure. DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	DAC_InitStructure. DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_SetChannel1Data(DAC_Align_12b_R, 0x000);

	//SPI
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(SPI2, &SPI_InitStructure);

	SPI_Cmd(SPI2, ENABLE);

	//TIMER i NVIC
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 24; //999 TERAZ NA 16kHz , 9 , teraz 32khz, 1
	TIM_TimeBaseStructure.TIM_Prescaler = 209; //839 DZIELONE PRZZEZ 160 000 , 524,, 499
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 9999;
	TIM_TimeBaseStructure.TIM_Prescaler = 8399;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	NVIC_InitStructure. NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure. NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure. NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure. NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);

	GPIO_SetBits(GPIOD,GPIO_Pin_14);
	GPIO_ResetBits(GPIOE, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14);

	for(;;)
	{
	/*	SPI_I2S_SendData(SPI2, 0x03);
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		uint16_t receive = SPI_I2S_ReceiveData(SPI2);
		if(receive==0x03)
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
	*/	
		
		
		ADC_SoftwareStartConv(ADC1);
		while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
		ADC_Result = ADC_GetConversionValue(ADC1);
		napiecie = (ADC_Result*5)/4096;

		setNumber(i,4);
		setNumber(j,3);
		setNumber(k,2);
		setNumber(l,1);

		if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6)){ //POPRZEDNI
			GPIO_SetBits(GPIOD,GPIO_Pin_15);
			if(z==0){
				z=1;
				u=0;
				i=0;
				j=0;
				k=0;
				l=0;
				a=0;
			}else{
				z=0;
				u=0;
				i=0;
				j=0;
				k=0;
				l=0;
				a=0;
			}
			for(x=0;x<5999999;x++);
			GPIO_ResetBits(GPIOD,GPIO_Pin_15);
		}

		if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7)){ //PLAY
			if(wlaczony==0){
				GPIO_ResetBits(GPIOD,GPIO_Pin_12);
				GPIO_SetBits(GPIOD,GPIO_Pin_14);
				TIM_Cmd(TIM3, ENABLE);
				TIM_Cmd(TIM4, ENABLE);
				wlaczony=1;
			}
		}

		if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_8)){ //PAUSE
			if(wlaczony==1){
				TIM_Cmd(TIM3, DISABLE);
				TIM_Cmd(TIM4, DISABLE);
				GPIO_ResetBits(GPIOD,GPIO_Pin_14);
				GPIO_SetBits(GPIOD,GPIO_Pin_12);
				wlaczony=0;
			}
		}

		if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_9)){  //NASTEPNY
			GPIO_SetBits(GPIOD,GPIO_Pin_13);
			//zmienia......
			if(z==0){
				z=1;
				u=0;
				i=0;
				j=0;
				k=0;
				l=0;
				a=0;
			}else{
				z=0;
				u=0;
				i=0;
				j=0;
				k=0;
				l=0;
				a=0;
			}
		for(x=0;x<5999999;x++);
			GPIO_ResetBits(GPIOD,GPIO_Pin_13);

		}

	}

}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		if(z==0){
		if(u>dlugosc[z]){
			z=1;
			u=0;
			i=0;
			j=0;
			k=0;
			l=0;
			a=0;
		}
		}else {
			if(u>dlugosc[z]){
						z=0;
						u=0;
						i=0;
						j=0;
						k=0;
						l=0;
						a=0;
					}
		}

			// sprawdzenie flagi zwiazanej z odebraniem danych przez USART
		if(z==0){
		if(ADC_Result>2000)
		{
			DAC_SetChannel1Data(DAC_Align_12b_R, u1[u]*(2000/100));
		}
		else
		{
			DAC_SetChannel1Data(DAC_Align_12b_R, u1[u]*(ADC_Result/100));
		}
		u++;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		}
		else {
			if(ADC_Result>2000)
					{
						DAC_SetChannel1Data(DAC_Align_12b_R, u2[u]*(2000/100));
					}
					else
					{
						DAC_SetChannel1Data(DAC_Align_12b_R, u2[u]*(ADC_Result/100));
					}
					u++;
					TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		}
	}
}

void TIM4_IRQHandler(void)//sekundy, to i i j, k i l to minuty
{
	if (TIM_GetFlagStatus(TIM4, TIM_FLAG_Update )) {
		TIM_ClearFlag(TIM4, TIM_FLAG_Update );
		i++;
		if(i==10){
			i=0;
			j++;
			if(j>5){
				j=0;
				k++;
				if(k>9){
					k=0;
					l++;
					if(l>9)
						l=0;
							}
					}
				}
	}
}

/*void USART3_IRQHandler(void)
{
	// sprawdzenie flagi zwiazanej z odebraniem danych przez USART
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
		// odebrany bajt znajduje sie w rejestrze USART3->DR
		while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET);
		uint8_t receive = USART_ReceiveData(USART3);
		audio = receive;
		a++;
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
		// wyslanie danych
		USART_SendData(USART3, audio);
		// czekaj az dane zostana wyslane
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);

	}
}*/
/*
if(ADC_Result>2000)
		{
			DAC_SetChannel1Data(DAC_Align_12b_R, rawAudio[u]*(2000/100));
		}
		else
		{
			DAC_SetChannel1Data(DAC_Align_12b_R, rawAudio[u]*(ADC_Result/100));
		}*/
