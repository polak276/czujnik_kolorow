/*
 * czujnik_koloru.c
 *
 *  Created on: 2014-03-02
 *      Author: polak27
 */

#include <avr/io.h>
#include <stdlib.h>
#include "op.h"
#include "LCD/lcd44780.h"

void piny_kierunek_wyjsciowy(void)
{
	DDR(OE_PORT) |= (1<<OE);
	DDR(S0_PORT) |= (1<<S0);
	DDR(S1_PORT) |= (1<<S1);
	DDR(S2_PORT) |= (1<<S2);
	DDR(S3_PORT) |= (1<<S3);
}

void wybor_skladowej(enum rodzaj_skladowej skl)
{
	switch(skl)
	{
	case czerwona:
		LOW_S2;
		LOW_S3;
	break;

	case niebieska:
		LOW_S2;
		HIGH_S3;
	break;

	case zielona:
		HIGH_S2;
		HIGH_S3;
	break;

	}
}

float frequency_detect(int licznik_impulsow)
{
	float czestotliwosc=0;
	return czestotliwosc=(1000000.0)/licznik_impulsow;
}

int TCSMeasure()
{
	if((PIND & TCS_OUT_PIN) == 0)
	{
		while((PIND & TCS_OUT_PIN) == 0);  //jeœli trafiamy w stan niski czekamy a¿ siê zmieni na wysoki
	}

	while(PIND & TCS_OUT_PIN); //jesli mamy stan wysoki czekamy a¿ zmieni siê na niski

	TCNT1 = 0x0000; //zerujemy timer
	TCCR1B |= (1<<CS10); //wlaczamy timer

	while((PIND & TCS_OUT_PIN) == 0); //czekamy a¿ zakoñczy siê stan niski

	TCCR1B &= ~(1<<CS10); //wylaczamy timer
	return (TCNT1);
}

void wyswietl_pomiary_skladowych(int skladowa_r, int skladowa_b, int skladowa_g)
{
	lcd_cls();
	lcd_ftoa(r_b=((float)skladowa_r)/((float)skladowa_b));
	lcd_locate(0,5);
	lcd_ftoa(rg=((float)skladowa_r)/((float)skladowa_g));
	lcd_locate(0,10);
	lcd_ftoa(bg=((float)skladowa_b)/((float)skladowa_g));

	lcd_locate(1,0);
	lcd_int(skladowa_r);
	lcd_locate(1,5);
	lcd_int(skladowa_b);
	lcd_locate(1,10);
	lcd_int(skladowa_g);
}


int rozpoznaj_kolor(int skladowa_r, int skladowa_b, int skladowa_g)
{
	lcd_cls();

	float skl_r=skladowa_r/2.5;//potrzebne do wspolpracy z zegarem 20 MHz
	float skl_b=skladowa_b/2.5;
	float skl_g=skladowa_g/2.5;

	r_b=(float)skl_r/(float)skl_b;
	rg=(float)skl_r/(float)skl_g;
	bg=(float)skl_b/(float)skl_g;

	if(((r_b>0.8) && (r_b<1.42)) && ((rg>0.9) && (rg<1.35)) && ((bg>0.89) && (bg<1.24)) )
		{lcd_str("zielony"); return 1;} //detekcja zielonego

	else if(((r_b>0.52) && (r_b<0.72)) && ((rg>0.41) && (rg<1.58)) && ((bg>0.74) && (bg<0.84)) )
			{lcd_str("czerwony"); return 2;} //detekcja czerwonego

	else if(((r_b>1.16) && (r_b<2.55)) && ((rg>0.86) && (rg<1.58)) && ((bg>0.54) && (bg<0.73))
			&& ((skl_r>287) && (skl_r<511)))
				{lcd_str("niebieski"); return 3;} //detekcja niebieskiego

	else if(((r_b>0.52) && (r_b<0.87)) && ((rg>0.68) && (rg<0.89)) && ((bg>1.19) && (bg<1.5)) )
			{lcd_str("zolty"); return 4;} //detekcja zoltego

	else if(((r_b>1.04) && (r_b<1.52)) && ((rg>0.75) && (rg<0.99)) && ((bg>0.61) && (bg<0.8))
			&& ((skl_r>205) && (skl_r<290)) && ((skl_b>178) && (skl_b<226)))
				{lcd_str("fioletowy"); return 5;} //detekcja fioletowego

	else if(((r_b>1.12) && (r_b<1.52)) && ((rg>0.75) && (rg<0.86)) && ((bg>0.61) && (bg<0.8))
			&& ((skl_r>205) && (skl_r<430)) && ((skl_b>178) && (skl_b<380))
			&& ((skl_g>244) && (skl_g<560)))
				{lcd_str("fioletowy"); return 5;} //detekcja fioletowego wariant drugi

	else if(((r_b>1.09) && (r_b<1.33)) && ((rg>0.9) && (rg<1.07)) && ((bg>0.75) && (bg<0.87))
			&& ((skl_b>46) && (skl_b<187)) )
				{lcd_str("szary/bialy"); return 6;} //detekcja szary/bialy

	else if(((r_b>0.57) && (r_b<0.99)) && ((rg>0.64) && (rg<0.74)) && ((bg>0.89) && (bg<1.15)) )
		{lcd_str("pomaranczowy"); return 7;} //detekcja pomaranczowy

	else if(((r_b>0.59) && (r_b<1.06)) && ((rg>0.37) && (rg<0.72)) && ((bg>0.62) && (bg<0.7)) )
		{lcd_str("rozowy"); return 8;} //detekcja rozowy

	else if(((r_b>0.84) && (r_b<1.09)) && ((rg>0.70) && (rg<0.88)) &&
			((bg>0.76) && (bg<0.87)) && ((skl_r>277) && (skl_r<481))
			&& ((skl_b>313) && (skl_b<463)) )
		{lcd_str("brazowy"); return 9;} //detekcja brazowy

	else if(((r_b>1.14) && (r_b<1.22)) && ((bg>0.76) && (bg<0.83))
			&& ((skl_r>379) && (skl_r<481)) )
				{lcd_str("czarny"); return 10;} //detekcja czarny
	else
		lcd_str("kolor nieznany");

	return 0;
}



