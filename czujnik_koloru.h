/*
 * czujnik_koloru.h
 *
 *  Created on: 8 mar 2018
 *      Author: polak27
 */

#ifndef CZUJNIK_KOLORU_H_
#define CZUJNIK_KOLORU_H_

void piny_kierunek_wyjsciowy(void);
void wybor_skladowej(enum rodzaj_skladowej skl);
float frequency_detect(int licznik_impulsow);
int TCSMeasure();
void wyswietl_pomiary_skladowych(int skladowa_r, int skladowa_b, int skladowa_g);
int rozpoznaj_kolor(int skladowa_r, int skladowa_b, int skladowa_g);

#endif /* CZUJNIK_KOLORU_H_ */
