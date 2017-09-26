#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "delay.h"
#include "diskio.h"
#include "ff.h"
#include "ffconf.h"
#include "list.h"
#include "misc.h"
#include "stdbool.h"
#include "display.h"


DIR Dir;
FIL file;
FILINFO fileInfo;
FATFS filesystem;
FRESULT fresult;

uint8_t DAC_Buff[512];
UINT b =0;
int8_t change = 0;
int8_t nr_switch=-1;
uint32_t nr_track=0;
bool stop = 0;
double time = 0;
float tp=0;
float volume=0;

struct List *first = 0, *last = 0, *current;

void Init_GPIO()
{
	//diody
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef  GPIO_InitStructure_Diodes;
	GPIO_InitStructure_Diodes.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure_Diodes.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_Diodes.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure_Diodes.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure_Diodes.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure_Diodes);

	//przyciski
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure_Buttons;
	GPIO_InitStructure_Buttons.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure_Buttons.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure_Buttons.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure_Buttons);

	//ADC1 PA1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	GPIO_InitTypeDef  GPIO_InitStructure_ADC;
	GPIO_InitStructure_ADC.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure_ADC.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure_ADC.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure_ADC);

	//DAC1 PA4
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure_DAC;
	GPIO_InitStructure_DAC.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure_DAC.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure_DAC.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure_DAC.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure_DAC);

	//wyswietlacz 7 segmentowy
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE , ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure_Display;
	GPIO_InitStructure_Display.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure_Display.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_Display.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure_Display);
}

//TIM7 wywietlacz multipleksowanie
void Init_TIM_display()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 8399;
	TIM_TimeBaseStructure.TIM_Prescaler = 20;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM7, ENABLE);

	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
}

//NVIC wywietlacz multipleksowanie
void Init_NVIC_display()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//TIM3 czas na wyswietlacz
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		time = scale_time(time);
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		Display_Real_Number(time);
		time = time + 0.01;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

//TIM5 zatrzymaj cofnij przewiñ do przodu
void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		if (nr_switch==5)//przewiñ o 1 do przodu
		{
			GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
			delay_ms(500);
			GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
			change=1;
			time=0;
		}
		else if (nr_switch==7)//zatrzymaj/wznow
		{
			if(stop==0)
			{
				stop=1;
				GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, DISABLE);
				TIM_Cmd(TIM3,DISABLE);
				NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT, ENABLE);
			}
			else
			{
				stop=0;
				GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
				TIM_Cmd(TIM3,ENABLE);
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
				NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT, DISABLE);
			}
		}
		else if (nr_switch==8)//cofnij 0 1
		{
			change=-1;
			time=0;
			GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
			delay_ms(500);
			GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
		}
		nr_switch=-1;
		TIM_Cmd(TIM5, DISABLE);
		TIM_SetCounter(TIM5, 0);
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
}

//TIM5 eliminacja wplywu drgan stykow przyciski PD12 PD14 PD15
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		nr_switch=5;
		TIM_Cmd(TIM5, ENABLE);
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
	else if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		nr_switch=7;
		TIM_Cmd(TIM5, ENABLE);
		EXTI_ClearITPendingBit(EXTI_Line7);
	}
	else if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		nr_switch=8;
		TIM_Cmd(TIM5, ENABLE);
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
}

//TIM5 eliminacja wplywu drgan stykow przyciski PD12 PD14 PD15
void Init_debouncer()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 8400-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 3000-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM5,DISABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
}

//TIM3 czas na wyswietlacz
void Init_NVIC_diodes()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 8399;
	TIM_TimeBaseStructure.TIM_Prescaler = 9999;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

//TIM5 eliminacja wplywu drgan stykow przyciski PD12 PD14 PD15
void Init_NVIC()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	EXTI_InitStructure.EXTI_Line = EXTI_Line5 | EXTI_Line7 | EXTI_Line8;
	NVIC_Init(&NVIC_InitStructure);
	EXTI_Init(&EXTI_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource5);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource7);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource8);
}

//DMA1 Channel7 Stream5  transfer DAC_Buff --> DAC_Channel_1
void Init_DMA(void)
{
	DMA_InitTypeDef dma_init;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Stream5);

	dma_init.DMA_Channel = DMA_Channel_7;
	dma_init.DMA_PeripheralBaseAddr = (uint32_t)(DAC_BASE + 0x10);
	dma_init.DMA_Memory0BaseAddr = (uint32_t)&DAC_Buff;
	dma_init.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	dma_init.DMA_BufferSize = 512;
	dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma_init.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	dma_init.DMA_Mode = DMA_Mode_Circular;
	dma_init.DMA_Priority = DMA_Priority_High;
	dma_init.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma_init.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	dma_init.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma_init.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(DMA1_Stream5, &dma_init);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_DMACmd(DAC_Channel_1, ENABLE);
}

void Init_DAC(void)
{
   	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE); //DAC
   	DAC_InitTypeDef DAC_InitStructure;
   	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
   	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
   	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
   	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
   	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
   	DAC_SetChannel1Data(DAC_Align_12b_R, 0x000);
   	DAC_Cmd(DAC_Channel_1, ENABLE);
}


//TIM6 wyzwalanie DAC (próbkowanie)
void Init_TimerForTriggerDAC(void)
{
	TIM_TimeBaseInitTypeDef tim_init;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	tim_init.TIM_CounterMode = TIM_CounterMode_Up;
	tim_init.TIM_Prescaler = 190;
	tim_init.TIM_Period = 10;
	TIM_TimeBaseInit(TIM6, &tim_init);
	TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);
	TIM_Cmd(TIM6, DISABLE);
}

//ADC1 regulacja g³osnosci
void Init_ADC()
{
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_84Cycles);
	ADC_Cmd(ADC1, ENABLE);
}

//odtwarzanie z danego pliku .WAV
void play(FRESULT fresult, struct List *track)
{
  UINT cnt;
  char *FileName;

  struct List *temp=track;
  FileName=temp->file.fname;

  fresult = f_open( &file, FileName, FA_READ );//otwarcie pliku .WAV do odczytu
  fresult = f_lseek(&file,44);//pominiêcie pierwszych 44 B pliku .WAV
  f_read (&file,&DAC_Buff[0],512,&cnt);//pierwszy transfer

  if(fresult == FR_OK){change=0;}
  else {time=0;}

  TIM_Cmd(TIM3, ENABLE);
  TIM_Cmd(TIM6, ENABLE);//w³¹czenie timera wyzwalaj¹cego DAC


  while(change == 0)//je¿eli nie wcisniêto przycisku przewiniêcia w przód/cofniêcia
  {
	 ADC_SoftwareStartConv(ADC1);
	 while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	 volume  = (float)ADC_GetConversionValue(ADC1)/4095.0;

	 volatile ITStatus it_st;
	 it_st = RESET;

	 while(it_st == RESET)//dopóki nie minie po³owa transferu DMA
	 {
		 it_st = DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_HTIF5);
	 }

	 f_read (&file,&DAC_Buff[0],256,&cnt);//wype³nienie 1 po³owy bufora

	 b=0;
	 while(b<256)
	 {
		 tp =volume*(float)DAC_Buff[b];//uzyskanie zadanej g³osnosci
		 DAC_Buff[b]=tp;
		 b++;
	 }

	 DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_HTIF5);//wyczyszczenie flagi miniêcia po³owy transferu

	 if(cnt<256){break;}

     it_st = RESET;

     while(it_st == RESET) //dopóki nie minie ca³y transfer DMA
     {
    	 it_st = DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_TCIF5);
     }

     f_read (&file,&DAC_Buff[256],256,&cnt);//wype³nienie 2 po³owy bufora

	 while(b<512)
	 {
		 tp =volume*(float)DAC_Buff[b];//uzyskanie zadanej g³osnosci
		 DAC_Buff[b]=tp;
		 b++;
	 }

     DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 );//wyczyszczenie flagi miniêcia ca³ego transferu

     if(cnt<256){break;}
  }

  TIM_Cmd(TIM6, DISABLE);//wy³¹czenie timera wyzwalaj¹cego DAC

  fresult = f_close(&file);//zamkniêcie pliku .WAV
  time=0;
}


int main( void )
{
	//Init
	SystemInit();
	Init_GPIO();
	Init_TIM_display();
	Init_NVIC_display();
	Init_ADC();
	Init_TimerForTriggerDAC();
	Init_DAC();
	Init_DMA();
	Init_debouncer();
	Init_NVIC();
	Init_NVIC_diodes();

	//inicjalizacja karty SD
	delay_init(80);
	SPI_SD_Init();
	disk_initialize(0);

	//odczyt plików opcjonalnie z katalogu
	fresult = f_mount( &filesystem, 1,1 );
	fresult = f_opendir(&Dir, "\\");

	if(fresult != FR_OK){return(fresult);}

	for(;;)
	{
		fresult = f_readdir(&Dir, &fileInfo);

		if(fresult != FR_OK){return(fresult);}

		if(!fileInfo.fname[0]){break;}

		if(nr_track==0)
		{
			first=last=add_last(last,fileInfo);
		}
		else
		{
			last=add_last(last,fileInfo);
		}
		nr_track = nr_track+1;

	}

	//ostatni utwór
	last->next=first;
	first->previous=last;
	current=first;

	for(;;)
	{
		play(fresult,current);
		if(change>=1)//przewiñ o 1 do przodu
		{
			if(nr_track>1)
			{
				current=current->next;
				time=0;
			}

		}
		else if (change==-1)//cofnij
		{
			if(nr_track>1)
			{
				current=current->previous;
			}
		}
	}
	return 0;
}

