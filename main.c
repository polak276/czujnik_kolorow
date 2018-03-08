/*
 *  * main.c
 *
 * F_CPU = 20MHz
 *
 * Detektor kolorów z opcj¹ odtwarzania dŸwiêków.
 * Odtwarzacz dŸwiêków na podstawie biblioteki M.Kardasia 2013
 */
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "op.h"
#include "PetitFS/diskio.h"
#include "PetitFS/pff.h"
#include "LCD/lcd44780.h"
#include <math.h>
#include "czujnik_koloru.h"

#define USE44KHZ 	0

#define SD_ON PORTB &= ~(1<<PB1)
#define SD_OFF PORTB |= (1<<PB1)
#define SD_SLOT_EMPTY (PINB & (1<<PB3))

#define TMR_START TCCR0 |= (1<<CS01)
#define TMR_STOP TCCR0 &= ~(1<<CS01)
#define TMR64_START TCCR0 |= (1<<CS01)|(1<<CS00)
#define TMR64_STOP TCCR0 &= ~((1<<CS01)|(1<<CS00))

#define SCK 	PB7
#define MOSI 	PB5
#define MISO 	PB6
#define CS 		PB4

#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)

#define PRZYCISK_PIN (1<<PD2)
#define PRZYCISK_ON !(PIND & PRZYCISK_PIN)

typedef struct {
	uint8_t stereo :1;
	uint8_t prescaler :1;
	uint8_t resolution;
	uint16_t khz;
} _FLAGS;

int kolor = 0;
int skladowa_r = 0, skladowa_b = 0, skladowa_g = 0;
float r_b = 0, rg = 0, bg = 0;
volatile _FLAGS FLAGS;	// definicja struktury
volatile uint8_t can_read;

FATFS Fs; /* File system object */
DIR Dir; /* Directory object */
FILINFO Fno; /* File information */
WORD rb;

static DWORD load_header(void);
static UINT play(const char *fn);
// ******************************************************

#define BUF_SIZE 512			// maksymalny rozmiar pojedynczego bufora

volatile uint16_t tick;			// czas utworu
volatile uint8_t slupek;		// wskaŸnik wysterowania
volatile int8_t sk1 = -1, sk2 = 4;	// zmienne pomocnicze do efektów dŸwiêkowych
uint8_t buf[2][BUF_SIZE];		// podwójny bufor do odczytu z karty SD
volatile uint8_t nr_buf;		// indeks aktywnego buforu
volatile int pom1 = 0;
uint8_t cplay[] = { 8, 12, 14, 15, 15, 14, 12, 8 };
uint8_t cnext[] = { 32, 20, 10, 5, 5, 10, 20, 32 };
char atnel[] = { 'a' - 5, 't' - 5, 'n' - 5, 'e' - 5, 'l' - 5, '.' - 5, 'p' - 5,
		'l' - 5 };
void sd_pwr(uint8_t OnOff);

BYTE rcv_spi(void) {
	SPDR = 0xFF;
	loop_until_bit_is_set(SPSR, SPIF);
	return SPDR;
}

//! **************** main() ***********************************************
int main(void) {

	PORTB = 0xff;
	DDRB |= (1 << PB1);
	PORTB |= (1 << PB0);
	DDRA |= (1 << PA7);
	PORTA |= (1 << PA7);
	lcd_init();

	lcd_defchar(0, cplay);	// wlasne znaki w LCD
	lcd_defchar(1, cnext);

	DDRB |= (1 << CS) | (1 << MOSI) | (1 << SCK) | (1 << CS);	// init SPI
	PORTB |= (1 << CS);
	SPCR |= (1 << SPE) | (1 << MSTR);
	SPSR |= (1 << SPI2X);
	TCCR1B = (1 << CS10);

	TCCR0 = (1 << WGM01);		// konfiguracja Timer0 (samplowanie)
	TIMSK = (1 << OCIE0);		//

	sei();

	DDR(OE_PORT) |= (1 << OE);
	DDR(S0_PORT) |= (1 << S0);
	DDR(S1_PORT) |= (1 << S1);
	DDR(S2_PORT) |= (1 << S2);
	DDR(S3_PORT) |= (1 << S3);

	LOW_OE;
	HIGH_S0;
	HIGH_S1;
	DDRA |= (1 << PA7);	//LCD
	PORTA |= (1 << PA7); // podswietlenie
	DDRD &= ~TCS_OUT_PIN; 	//pin wyjœcia detektora kolorów jako wejœcie
	PORTD |= TCS_OUT_PIN;

	lcd_cls();
	_delay_ms(1000);

	while (1) {
		sd_pwr(0);

		if ( SD_SLOT_EMPTY)
			continue;	// jeœli karta niewykryta powrót na pocz¹tek while(1)

		PORTD &= ~(1 << PD7); //led on
		sd_pwr(1);

		if (pf_mount(&Fs))
			continue; /* Initialize FS */

		while (1) {
			if (PRZYCISK_ON) {
				_delay_ms(400);
				if (PRZYCISK_ON) {
					wybor_skladowej(czerwona);
					skladowa_r = TCSMeasure();
					wybor_skladowej(niebieska);

					_delay_ms(100);

					skladowa_b = TCSMeasure();
					wybor_skladowej(zielona);

					_delay_ms(100);

					skladowa_g = TCSMeasure();
					kolor = rozpoznaj_kolor(skladowa_r, skladowa_b, skladowa_g);
					lcd_locate(1, 0);
					lcd_int(kolor);

					_delay_ms(250);

					TCNT1 = 0x0000;
					TCCR1A = (1 << WGM10) | (1 << COM1A1) | (0 << COM1A0)
							| (1 << COM1B1);
					TCCR1B = (1 << CS10);

					TCCR0 = (1 << WGM01);// konfiguracja Timer0 (samplowanie)
					TIMSK = (1 << OCIE0);

					if ( SD_SLOT_EMPTY)
						break;

					if (pf_opendir(&Dir, ""))
						break; /* Open sound file directory (root dir) */

					if (kolor == 1) {
						play("ZIELONY.WAV");
					} else if (kolor == 2) {
						play("CZERWONY.WAV");
					} else if (kolor == 3) {
						play("NIEB.WAV");
					} else if (kolor == 4) {
						play("ZOLTY.WAV");
					} else if (kolor == 5) {
						play("FIOLET.WAV");
					} else if (kolor == 6) {
						play("SZARBIEL.WAV");
					} else if (kolor == 7) {
						play("POMARAN.WAV");
					} else if (kolor == 8) {
						play("ROZOWY.WAV");
					} else if (kolor == 9) {
						play("BRAZOWY.WAV");
					} else if (kolor == 10) {
						play("CZARNY.WAV");
					}

					TCCR1A &= ~((1 << WGM10) | (1 << COM1A1) | (0 << COM1A0)
							| (1 << COM1B1)); // zerujemy rejestr
				}
			}
		}
	}
}

//***************** przerwanie TIMER0 - samplowanie ******************************************
ISR(TIMER0_COMP_vect) {

#if USE44KHZ == 1
	static uint16_t buf_idx;		// indeks w pojedynczym buforze
	static uint8_t v1, v2;// zmienne do przechowywania próbek
	buf_idx++;// pomijamy m³odszy bajt 16-bitowej próbki kana³ L
	v1 = buf[nr_buf][buf_idx++]-128;// -128 korekcja konwerji próbki 16-bitowej do 8-bitowej
	buf_idx++;// pomijamy m³odszy bajt 16-bitowej próbki kana³ R
	v2 = buf[nr_buf][buf_idx++]-128;// -128 korekcja konwerji próbki 16-bitowej do 8-bitowej
	OCR1A = v1;// próbka na wyjœcie PWM1, kana³ L
	OCR1B = v2;// próbka na wyjœcie PWM2, kana³ R

	buf_idx &= 0x01ff;
	if( !buf_idx ) {
		can_read = 1;
		nr_buf ^= 0x01;
	}
#endif

#if USE44KHZ == 0
	static uint16_t buf_idx;		// indeks w pojedynczym buforze
	static uint8_t v1, v2;			// zmienne do przechowywania próbek
	static uint8_t efekt_cnt;// zmienna pomocnicza dla uzyskiwania prostych efektów

	if (!efekt_cnt) {
		if (16 == FLAGS.resolution) {			// jeœli rozdzielczoœæ 16-bitów
			if (FLAGS.stereo) {					// jeœli próbki stereofoniczne
				buf_idx++;	// pomijamy m³odszy bajt 16-bitowej próbki kana³ L
				v1 = buf[nr_buf][buf_idx++] - 128;// -128 korekcja konwerji próbki 16-bitowej do 8-bitowej
				buf_idx++;	// pomijamy m³odszy bajt 16-bitowej próbki kana³ R
				v2 = buf[nr_buf][buf_idx++] - 128;// -128 korekcja konwerji próbki 16-bitowej do 8-bitowej
			} else {								// jeœli próbki monofoniczne
				buf_idx++;		// pomijamy m³odszy bajt 16-bitowej próbki MONO
				v1 = buf[nr_buf][buf_idx++] - 128;// -128 korekcja konwerji próbki 16-bitowej do 8-bitowej
				v2 = v1;						// to samo na dwa kana³y/wyjœcia
			}
		} else {								// jeœli rozdzielczoœæ 8-bitów
			if (!FLAGS.stereo) {					// jeœli próbki monofoniczne
				v1 = buf[nr_buf][buf_idx++];// pobieramy próbkê MONO do zmiennej v1
				v2 = v1;						// to samo na dwa kana³y/wyjœcia
			} else {							// jeœli próbki stereofoniczne
				v1 = buf[nr_buf][buf_idx++];		// pobieramy próbkê kana³ L
				v2 = buf[nr_buf][buf_idx++];		// pobieramy próbkê kana³ R
			}
		}

	}

	if (sk1 > -1) {
		if (efekt_cnt++ > sk1) {
			efekt_cnt = 0;
			buf_idx += sk2;
		}
	} else {
		efekt_cnt = 0;
		sk2 = 4;
	}

	OCR1A = v1;								// próbka na wyjœcie PWM1, kana³ L
	OCR1B = v2;								// próbka na wyjœcie PWM2, kana³ R

	if (buf_idx > BUF_SIZE - 1) {
		buf_idx = 0;								// reset indeksu bufora
		can_read = 1;							// flaga = 1
		nr_buf ^= 0x01;							// zmiana bufora na kolejny
	}

	tick++;										// podstawa czasu

	slupek = v1;				// zmienne na potrzeby wskaŸnika wysterowania

	pom1 = efekt_cnt;

#endif
}

/* 0:Invalid format, 1:I/O error, >1:Number of samples */
static DWORD load_header(void) {
	DWORD sz;
	uint8_t *wsk_buf = &buf[0][0];
	uint16_t ocrx;

	if (pf_read(wsk_buf, 12, &rb))
		return 1; /* Load file header (12 bytes) */

	if (rb != 12 || LD_DWORD(wsk_buf+8) != FCC('W', 'A', 'V', 'E'))
		return 0;

	for (;;) {
		pf_read(wsk_buf, 8, &rb); /* Get Chunk ID and size */
		if (rb != 8)
			return 0;
		sz = LD_DWORD(&wsk_buf[4]); /* Chunk size */

		switch (LD_DWORD(&wsk_buf[0])) { /* FCC */
		case FCC('f', 'm', 't', ' '): /* 'fmt ' chunk */
			if (sz > 100 || sz < 16)
				return 0; /* Check chunk size */

			pf_read(wsk_buf, sz, &rb); /* Get content */

			if (rb != sz)
				return 0;

			if (wsk_buf[0] != 1)
				return 0; /* Check coding type (1) */

			if (wsk_buf[2] != 1 && wsk_buf[2] != 2) /* Check channels (1/2) */
				return 0;

			FLAGS.stereo = wsk_buf[2] == 2; /* Get channel flag */
			if (wsk_buf[14] != 8 && wsk_buf[14] != 16) /* Check resolution (8/16) */
				return 0;

			FLAGS.resolution = wsk_buf[14];	// ustalamy jaka rozdzielczoœæ 16/8 - bitów
			FLAGS.prescaler = 0;

			// obliczmy preskaler Timera0 w zale¿noœci od czêstotliwoœci samplowania
			FLAGS.khz = LD_WORD(&wsk_buf[4]);

			ocrx = (uint16_t) (F_CPU / 8 / LD_WORD(&wsk_buf[4])) - 1;

			if (ocrx > 255) {
				ocrx = (uint16_t) (F_CPU / 64 / LD_WORD(&wsk_buf[4])) - 1;
				FLAGS.prescaler = 1;
			}
			OCR0 = (uint8_t) ocrx;					// obliczona wartoœæ OCR0

			break;

		case FCC('d', 'a', 't', 'a'): /* 'data' chunk (start to PLAY) */
			return sz;

		case FCC('L', 'I', 'S', 'T'): /* 'LIST' chunk (skip) */
		case FCC('f', 'a', 'c', 't'): /* 'fact' chunk (skip) */
			pf_lseek(Fs.fptr + sz);
			break;

		default: /* Unknown chunk (error) */
			return 0;
		}
	}

	return 0;
}

// ******************  funkcja  P L A Y  ********************************
static UINT play(const char *fn) {

	DWORD sz;
	FRESULT res;

	if ((res = pf_open(fn)) == FR_OK) {

		sz = load_header(); /* Load file header */
		if (sz < 256)
			return (UINT) sz;

		pf_lseek(0);

		pf_read(&buf[0][0], BUF_SIZE, &rb);	// za³aduj pierwsz¹ czêœæ bufora
		pf_read(&buf[1][0], BUF_SIZE, &rb);	// za³aduj drug¹ czêœæ bufora

		if (!FLAGS.prescaler)
			TMR_START;		// start Timera0 (samplowanie) z
		else
			TMR64_START;			// preskalerem zale¿nym od czêstotliwoœci

		DDRD |= (1 << PD5) | (1 << PD4);// ustaw piny PWM1 (OC1A) oraz PWM2 (OC1B) jako wyjœcia WA¯NE !!!

#if USE44KHZ == 0
		uint8_t ss = 0, mm = 0;
#endif

		while (1) {
			if (can_read) {		// jeœli flaga ustawiona w obs³udze przerwania

				pf_read(&buf[nr_buf ^ 0x01][0], BUF_SIZE, &rb);	// odczytaj kolejny bufor
				if (rb < BUF_SIZE)
					break;		// jeœli koniec pliku przerwij pêtlê while(1)

#if USE44KHZ == 0
				//PORTC = ~tab[slupek/33];		// obs³uga wsk. wysterowania

				// obliczanie czasu utworu
				if (tick > FLAGS.khz) {
					ss++;
					if (ss > 59) {
						ss = 0;
						mm++;
						if (mm > 59)
							mm = 0;
					}
					tick = 0;
				}

#endif
				can_read = 0;
			}
		}

		DDRD &= ~((1 << PD5) | (1 << PD4));	// ustaw piny PWM1 (OC1A) oraz PWM2 (OC1B) jako wejœcia WA¯NE !!!

		if (!FLAGS.prescaler)
			TMR_STOP;	// wy³¹czenie Timera0 (samplowania)
		else
			TMR64_STOP;

		_delay_ms(500);						// przerwa 0,5s
	}

	return res;
}

/*
 * 		FUNKCJA Programowego w³¹czania i wy³¹czania zasilania do karty
 * 		sd_pwr(1) - w³¹czenie zasilania
 * 		sd_pwr(0) - wy³¹czenie zasilania
 */
void sd_pwr(uint8_t OnOff) {
	if (OnOff) {
		SD_ON;
		SPCR |= (1 << SPE) | (1 << MSTR);
		_delay_ms(50);
		disk_initialize();
	} else {
		SPCR &= ~(1 << SPE);
		PORTB &= ~((1 << PB7) | (1 << PB6) | (1 << PB5));
		SD_OFF;
		_delay_ms(50);
	}
}
