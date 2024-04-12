/********************************************
 *
 *  Name: Ephraim Joseph De La Cruz
 *  Email: eedelacr@usc.edu
 *  Section: Fri 11:00a.m.
 *  Assignment: Lab 8
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "lcd.h"

void play_note(uint16_t);
void variable_delay_us(int16_t);
void timer2_init(void);

// Frequencies for natural notes from middle C (C4)
// up one octave to C5.
uint16_t frequency[8] =
   { 262, 294, 330, 349, 392, 440, 494, 523 };

volatile uint8_t new_state, old_state;
volatile uint8_t changed = 0;  // Flag for state change
volatile int16_t count = 0;		// Count to display
volatile uint8_t a, b;
volatile char x;
volatile char buf[11];
volatile int cycles;

int main(void) {

	// Initialize DDR and PORT registers and LCD
	DDRB |= (1 << PB3);
	PORTC |= (1 << PC4) | (1 << PC5);
	lcd_init();
	timer2_init();

	PCICR |= (1 << PCIE1);
	PCMSK1 |= (1 << PCINT12) | (1 << PCINT13);
	sei();

	// Write a splash screen to the LCD
	lcd_moveto(0, 0); lcd_stringout("EE 109 Lab 8");
	lcd_moveto(1,0);  lcd_stringout("Ephraim says hi");
	_delay_ms(2000);
	lcd_writecommand(0x01);
	
	// snprintf(buf, sizeof(buf), "%d", count);
	// lcd_stringout(buf);

	/* x = PINC;
	a = x & (1 << PC4) | (1 << PC5);

	if (!a)
		OCR2A = 51; // 20%
	else
		OCR2A = 204; // 80%

	count = OCR2A; */

	while (1) {                 // Loop forever
		/*if (changed) {
			OCR2A = count;
			lcd_moveto(1, 0);
			snprintf(buf, sizeof(buf), "%d", count);
			lcd_stringout(buf);
			changed = 0; // Reset flag
		} */
	 if(changed){
		OCR2A = count;
		char buf2[11];
		lcd_moveto(0, 0);
		snprintf(buf2, sizeof(buf2), "%d", count);
		lcd_stringout(buf2);
		changed = 0;
	 }
}
}

void timer2_init(void)
{
    TCCR2A |= (0b11 << WGM20);  // Fast PWM mode, modulus = 256
    TCCR2A |= (0b10 << COM2A0); // Turn D11 on at 0x00 and off at OCR2A
    OCR2A = 128;                // Initial pulse duty cycle of 50%
    TCCR2B |= (0b111 << CS20);  // Prescaler = 1024 for 16ms period
}

void play_note(uint16_t freq)
{
    cycles = 2 * freq;
    OCR1A = 16000000 / (2 * freq);
    TCCR1B |= (1 << CS10);
}

void variable_delay_us(int delay) {
	int i = (delay + 5) / 10;
	while (i--)
		_delay_us(10);
}

// Pin Change Interrupt Service Routine (ISR) to handle encoder input bits
ISR(PCINT1_vect)
{
	// In Task 6, add code to read the encoder inputs and determine the new
	// count value
	x = PINC;
	a = x & (1 << PC4);
	b = x & (1 << PC5);

	if (old_state == 0) {
		// Handle A and B inputs for state 0
		if (a) {
			new_state = 1;
			count++;
		}
		else if (b) {
			new_state = 3;
			count--;
		}
	}
	else if (old_state == 1) {
		// Handle A and B inputs for state 1
		if (!a) {
			new_state = 0;
			count--;
		}
		else if (b) {
			new_state = 2;
			count++;
		}
	}
	else if (old_state == 2) {
		// Handle A and B inputs for state 2
		if (!a) {
			new_state = 3;
			count++;
		}
		else if (!b) {
			new_state = 1;
			count--;
		}
	}
	else {   // old_state = 3
		// Handle A and B inputs for state 3
		if (a) {
			new_state = 2;
			count--;
		}
		else if (!b) {
			new_state = 0;
			count++;
		}
	}
	if(count > 255){
		count = 255;
	}
	else if (count < 0){
		count = 0;
	}
	if (new_state != old_state) {
		changed = 1;
		old_state = new_state;
	}
}
