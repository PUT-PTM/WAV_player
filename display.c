#include "display.h"
#include "stm32f4xx_tim.h"

//ktore segmenty wlaczyc
unsigned int cyfry[] = {0xC000, 0xF900, 0xA400, 0xB000, 0x9900, 0x9200, 0x8200, 0xF800, 0x8000, 0x9000,	//Cyfry 0-9
		0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00, //Cyfry A-F
		0xBF00,	 //16 - kreska
		0x7F00,  //17 - kropka
		0xFF00	 //18 - cyfra wylaczona
};

//ktorej cyfry na wyswietlaczu uzyc
unsigned int numery_cyfr[] = {0x0004, 0x0008, 0x0010, 0x0020};
unsigned int id = 0;

//jakie cyfry wyswietlic
unsigned int cyfry_wyswietl[] = {0xFF00, 0xFF00, 0xFF00, 0xFF00};

void PokazKreski(){
	cyfry_wyswietl[3] = cyfry[16];
	cyfry_wyswietl[2] = cyfry[16];
	cyfry_wyswietl[1] = cyfry[16];
	cyfry_wyswietl[0] = cyfry[16];
}

void WyswietlLiczbeCalkowita(int liczba){

	for(int i = 0; i < 4; i++) cyfry_wyswietl[i] = cyfry[18];

	if(liczba > 9999 || liczba < -999){

		PokazKreski();

	}else{

		int ujemna = (liczba < 0);
		liczba = abs(liczba);

		cyfry_wyswietl[3] = cyfry[liczba % 10];
		if(ujemna) cyfry_wyswietl[2] = cyfry[16];

		if(liczba >= 10){
			cyfry_wyswietl[2] = cyfry[(liczba/10) % 10];
			if(ujemna) cyfry_wyswietl[1] = cyfry[16];

		}if(liczba >= 100){
			cyfry_wyswietl[1] = cyfry[(liczba/100) % 10];
			if(ujemna) cyfry_wyswietl[0] = cyfry[16];

		}if(liczba >= 1000){
			cyfry_wyswietl[0] = cyfry[(liczba/1000) % 10];
		}

	}
}

void Display_Real_Number(double number){
	int l;
	int ujemna = (number < 0);

	if(number > 9999.0 || number < -999.0){

		PokazKreski();

	}else if(number > 999.0){

		l = (int)number;
		WyswietlLiczbeCalkowita(l);
		cyfry_wyswietl[3] &= cyfry[17];

	}else if(absf(number) >= 100.0){

		if(ujemna){
			l = (int)(number);
			WyswietlLiczbeCalkowita(l);
			cyfry_wyswietl[3] &= cyfry[17];
		}else{
			l = (int)(number*10);
			WyswietlLiczbeCalkowita(l);
			cyfry_wyswietl[2] &= cyfry[17];
		}

	}else if(absf(number) >= 10.0){

		if(ujemna){
			l = (int)(number*10);
			WyswietlLiczbeCalkowita(l);
			cyfry_wyswietl[2] &= cyfry[17];
		}else{
			l = (int)(number*100);
			WyswietlLiczbeCalkowita(l);
			cyfry_wyswietl[1] &= cyfry[17];
		}

	}else if(absf(number) >= 1.0){

		if(ujemna){
			l = (int)(number*100);
			WyswietlLiczbeCalkowita(l);
			cyfry_wyswietl[1] &= cyfry[17];
		}else{
			l = (int)(number*1000);
			WyswietlLiczbeCalkowita(l);
			cyfry_wyswietl[0] &= cyfry[17];
		}

	}else if(absf(number) >= 0.0){

		if(ujemna){
			l = (int)(number*100);
			WyswietlLiczbeCalkowita(l);
			if(cyfry_wyswietl[2] == cyfry[16]){
				cyfry_wyswietl[2] = cyfry[0];
			}
			cyfry_wyswietl[1] = cyfry[0] & cyfry[17];
			cyfry_wyswietl[0] = cyfry[16];
		}else{
			l = (int)(number*1000);
			WyswietlLiczbeCalkowita(l);
			if(number < 0.1){
				cyfry_wyswietl[1] = cyfry[0];
				if(number < 0.01){
					cyfry_wyswietl[2] = cyfry[0];
					if(number < 0.001){
						cyfry_wyswietl[3] = cyfry[0];
					}
				}
			}
			cyfry_wyswietl[0] = cyfry[0] & cyfry[17];
		}

	}

}

int abs(int l){ if(l < 0) return -l; else return l; }

float absf(float l){ if(l < 0) return -l; else return l; }

double scale_time(double time)
{
	if(time > 0.59 && time < 0.6)
	{
		time = 1.0;
	}
	if(time > 1.59 && time < 1.6)
	{
		time = 2.0;
	}
	if(time > 2.59 && time < 2.6)
	{
		time = 3.0;
	}
	if(time > 3.59 && time < 3.6)
	{
		time = 4.0;
	}
	if(time > 4.59 && time < 4.6)
	{
		time = 5.0;
	}
	if(time > 5.59 && time < 5.6)
	{
		time = 6.0;
	}
	if(time > 6.59 && time < 6.6)
	{
		time = 7.0;
	}
	if(time > 7.59 && time < 7.6)
	{
		time = 8.0;
	}
	if(time > 8.59 && time < 8.6)
	{
		time = 9.0;
	}
	if(time > 9.59 && time < 9.6)
	{
		time = 10.0;
	}
	return time;
}

void TIM7_IRQHandler ( void ){
	u16 ilosc = 20;
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET){

		GPIOE->ODR = cyfry_wyswietl[id] | numery_cyfr[id];
		if(++id > 3) id = 0;

		if(++ilosc > 1000){
			ilosc=0;
		}

		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);

	}
}
