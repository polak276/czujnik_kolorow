/*
 * opis_pinow.h
 *
 *  Created on: 2014-03-01
 *      Author: polak27
 */

#ifndef OPIS_PINOW_H_
#define OPIS_PINOW_H_

//***nazwy pinów***
#define OE 0
#define S0 1
#define S1 2
#define S2 3
#define S3 4

#define SW1 5
#define SW2 6
#define SW3 7

#define SW1_PIN (1<<SW1)
#define SW2_PIN (1<<SW2)
#define SW3_PIN (1<<SW3)

#define LED_PIN (1<<PD7)

//***kierunki pinów***
#define OE_PORT C
#define S0_PORT C
#define S1_PORT C
#define S2_PORT C
#define S3_PORT C

#define SW1_PORT C
#define SW2_PORT C
#define SW3_PORT C
#define LED_PORT D
#define TCS_OUT_PIN (1<<PD7)

//***PORT***
#define PORT(x) SPORT(x)
#define SPORT(x) (PORT##x)

//***PIN***
#define PIN(x) SPIN(x)
#define SPIN(x) (PIN##x)

//***DDR***
#define DDR(x) SDDR(x)
#define SDDR(x) (DDR##x)


//***stany H/L na danych pinach***
#define HIGH_OE PORT(OE_PORT) |= (1<<OE)
#define LOW_OE PORT(OE_PORT) &= ~(1<<OE)

#define HIGH_S0 PORT(S0_PORT) |= (1<<S0)
#define LOW_S0 PORT(S0_PORT) &= ~(1<<S0)

#define HIGH_S1 PORT(S1_PORT) |= (1<<S1)
#define LOW_S1 PORT(S1_PORT) &= ~(1<<S1)

#define HIGH_S2 PORT(S2_PORT) |= (1<<S2)
#define LOW_S2 PORT(S2_PORT) &= ~(1<<S2)

#define HIGH_S3 PORT(S3_PORT) |= (1<<S3)
#define LOW_S3 PORT(S3_PORT) &= ~(1<<S3)

#define SW1_DOWN !(PINC & SW1_PIN)
#define SW2_DOWN !(PINC & SW2_PIN)
#define SW3_DOWN !(PINC & SW3_PIN)

#define LED_ON PORT(LED_PORT) &= ~LED_PIN
#define LED_OFF PORT(LED_PORT) |= LED_PIN
#define LED_TOGGLE PORT(LED_PORT) ^=(LED_PIN)

enum rodzaj_skladowej {czerwona, niebieska, zielona};

extern int skladowa_r, skladowa_b, skladowa_g;
extern float r_b, rg, bg;

void wybor_skladowej(enum rodzaj_skladowej skl);
void timer_start();
float frequency_detect(int lcz);
void int0_on();
void int0_off();
void wyswietl_pomiary_skladowych(int, int, int);

#endif /* OPIS_PINOW_H_ */
