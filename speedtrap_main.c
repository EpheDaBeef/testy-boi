/********************************************
 *
 *  Name: Ephraim Joseph De La Cruz
 *  Email: eedelacr@usc.edu
 *  Section: Fri 11:00a.m.
 *  Assignment: Final Project - Speed Trap
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "lcd.h"
// #include "adc.h"

void timer2_init(void);
void timer1_init(void);
void timer0_init(void);

volatile uint8_t new_state, old_state;
volatile uint8_t changed = 0;  // flag state change
volatile uint8_t sensor_changed = 0;  // flag sensor state change
volatile uint16_t encoder_count = 0; // count for encoder
volatile uint16_t buzzer_count = 0; // count for buzzer
volatile int16_t sensor_count = 0; // count for sensor
volatile int16_t speed, new_speed, decimal, expired; // speed values + expired flag
uint32_t distance_mm, time_ms, speed_mm_per_s, speed_cm_per_s;
volatile uint8_t a, b, c, d; // photosensor inputs + encoder inputs
volatile char x,y,z;
volatile uint16_t motor_width = 50; // halfway

enum state {START, STOP};
volatile int state = STOP;

int main(void) {

	sei(); // global interrupt
	lcd_init(); // lcd
	timer1_init();
	timer2_init();
	timer0_init();

	// splash screen
	lcd_moveto(0, 0); lcd_stringout("EE 109 Project");
	lcd_moveto(1,0);  lcd_stringout("Ephraim says hi");
	_delay_ms(2000);
	lcd_writecommand(0x01);

	/* PORTB, bit 3 (PB3)	PWM signal to servo motor
	PORTB, bit 4 (PB4)	Tri-state buffer enable
	PORTB, bit 5 (PB5)	LED for indicating timing in progress
	PORTC, bit 1 (PC1)	Buzzer
	PORTC, bit 2 (PC2)	RGB LED's red segment
	PORTC, bit 3 (PC3)	RGB LED's blue segment
	PORTC, bit 4 (PC4)	Start sensor
	PORTC, bit 5 (PC5)	Stop sensor
	PORTD, bit 0 (PD0)	RS-232 Serial Rx input
	PORTD, bit 1 (PD1)	RS-232 Serial Tx output
	PORTD, bit 2 (PD2)	Rotary encoder
	PORTD, bit 3 (PD3)	Rotary encoder */

	DDRB |= (1<< PB3) | (1 << PB4) | (1<< PB5);
	DDRC |= (1 << PC1) | (1 << PC2) | (1 << PC3);
	DDRC &= ~((1 << PC4) | (1 << PC5));
	DDRD |= (1 << PD1);
	DDRD &= ~((1 << PD0) | (1 << PD2) | (1 << PD3)); // Rx input + encoder
	PORTC |= (1 << PC4) | (1 << PC5); // sensor

	PCICR |= (1 << PCIE1) | (1 << PCIE2);
	PCMSK1 |= (1 << PCINT12) | (1 << PCINT13);
	PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);

	unsigned char eeprom_read;  
	eeprom_read = eeprom_read_byte((void*)200);    
	encoder_count = eeprom_read; 
	
	while (1) {
		if (sensor_changed) {
			// speed = (5400)/(sensor_count); // distance between sensors is 5.4cm
			// decimal = (5400)%(sensor_count);

			// uint32_t distance_mm = 38; // Distance between sensors in mm (1.5 inches * 25.4)
			// uint32_t time_ms = (tccnt * 1000) / (15625);
			// uint32_t speed_mm_per_s = (distance_mm * 1000) / time_ms;
			// uint32_t speed_cm_per_s = (distance_mm * 100) / time_ms;

			// char buf[13];
			// snprintf(buf, 13, "%3dms = %u.%u ", sensor_count, speed, decimal%10);
			// lcd_moveto(0,0);
			// lcd_stringout(buf);

			stop_time = 0;
			lcd_writecommand(0x01); 

			uint32_t distance_mm = 38; // Distance between sensors in mm (1.5 inches * 25.4)
			uint32_t time_ms = (tccnt * 1000) / (15625);
			uint32_t speed_mm_per_s = (distance_mm * 1000) / time_ms;
			uint32_t speed_cm_per_s = (distance_mm * 100) / time_ms;

			lcd_moveto(0, 0);
			lcd_stringout("time= ");
			lcd_moveto(0, 14);
			lcd_stringout("ms");

			unsigned char buf_time[20];
			snprintf(buf_time, 20, "%lu. ", time_ms);
			lcd_moveto(0, 6); 
			lcd_stringout(buf_time);

			lcd_moveto(1, 0);
			lcd_stringout("speed= "); 
			lcd_moveto(1, 12);
			lcd_stringout("cm/s");
			
			unsigned char buf_speed[20];
			snprintf(buf_speed, 20, "%lu", speed_cm_per_s); 
			lcd_moveto(1, 7);   
			lcd_stringout(buf_speed); 

			unsigned char speed_limit[20];
			snprintf(speed_limit, 20, "%u", encoder_count);
			lcd_moveto(0, 11);
			lcd_stringout(speed_limit);

		if (expired) {
			lcd_moveto(0,0);
			lcd_stringout("too late :(");
			PORTB &= ~(1 << PB5);
			state = STOP;
		}

		if (changed) {
			changed = 0; 
			lcd_moveto(0, 0);
			lcd_stringout("speedlimit -  "); 
			lcd_moveto(1, 12); 
			lcd_stringout("cm/s"); 
			lcd_moveto(0, 8);
			unsigned char buf2[20];
			snprintf(buf2, 20, "%u", encoder_count);
			lcd_moveto(1, 0);
			lcd_stringout("          ");
			lcd_moveto(1, 0);
			lcd_stringout(buf2);			
		}

		if (speed > encoder_count){ // change
			buzzer_count = 0; 
			TCCR0B |= ((1 << CS02)); // turn buzzer interrupt on
			}

			int y = 35 - ((speed*23)/100);

			if(y < 0){
				y = 0;
			}

			OCR2A = y;
			OCR1B = 0;
	}
}

void timer0_init(void) {
    TCCR0A |= (1 << WGM01);
	TCCR0B |= (1 << CS01) | (1 << CS00); // 64
	TIMSK0 |= (1 << OCIE0A);
	OCR0A = 255;
}

void timer1_init(void) {

	TCCR1A = 0; // clear
	TCCR1B |= (1 << WGM12); // ctc
    TCCR1B |= (1 << CS12) | (1 << CS10); // 1024
	TIMSK1 |= (1 << OCIE1A); // 16-bit encoder_counter ctc interrupt
	OCR1A = 62500; // overflow
	
}

void timer2_init(void)
{	
	TCCR2A = 0;
    TCCR2A |= (1 << COM2A1) | (1 << WGM20) | (1 << WGM21); 
    OCR2A = 23;                
    TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22); // 1024

}

// Pin Change Interrupt Service Routine (ISR) to handle sensor input bits
ISR(PCINT1_vect)
{
	x = PINC;
	a = x & (1 << PC4);
	b = x & (1 << PC5);

	if (!a) {
		state = START;
		TCNT1 = 0;
		TCCR1B |= (1 << CS12)|(1 << CS10); // start TIMER1
		PORTB |= (1 << PB5);
	}

	if (!b) {
		TCCR1B &= ~((1 << CS12) | (1 << CS10)); // stop TIMER1
		PORTB &= ~(1 << PB5);
		sensor_count = (TCNT1 / (62500/4000));
		state = STOP;
		sensor_changed = 1;
	}
}

// Pin Change Interrupt Service Routine (ISR) to handle encoder input bits
ISR(PCINT2_vect) {
	// In Task 6, add code to read the encoder inputs and determine the new
	// count value
	z = PIND;
	c = z & (1 << PD2);
	d = z & (1 << PD3);

	if (old_state == 0) {
		// Handle A and B inputs for state 0
		if (c) {
			new_state = 1;
			encoder_count++;
		}
		else if (d) {
			new_state = 3;
			encoder_count--;
		}
	}
	else if (old_state == 1) {
		// Handle A and B inputs for state 1
		if (!c) {
			new_state = 0;
			encoder_count--;
		}
		else if (d) {
			new_state = 2;
			encoder_count++;
		}
	}
	else if (old_state == 2) {
		// Handle A and B inputs for state 2
		if (!c) {
			new_state = 3;
			encoder_count++;
		}
		else if (!d) {
			new_state = 1;
			encoder_count--;
		}
	}
	else {   // old_state = 3
		// Handle A and B inputs for state 3
		if (c) {
			new_state = 2;
			encoder_count--;
		}
		else if (!d) {
			new_state = 0;
			encoder_count++;
		}
	}

	if(encoder_count > 99){
		encoder_count = 99;
	}
	else if (encoder_count < 0){
		encoder_count = 0;
	}

	if (new_state != old_state) {
		changed = 1;
		old_state = new_state;
	}

	eeprom_update_byte((void*)200, encoder_count);
}

ISR(TIMER1_COMPA_vect) { // past 4 seconds
	TCCR1B &= ~((1 << CS12)|(1 << CS11)|(1 << CS10)); // stop TIMER1
	expired = 1;
	state = STOP;
}

ISR(TIMER0_COMPA_vect) { // buzzer
	PORTC ^= (1 << PC1);
	buzzer_count++;

	if(buzzer_count >= OCR0A) {
		TCCR0B &= ~((1 << CS01) | (1 << CS00));
	}
}
