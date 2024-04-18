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

void play_note(uint16_t);
void variable_delay_us(int16_t);
void timer2_init(void);
void timer1_init(void);

// Frequencies for natural notes from middle C (C4)
// up one octave to C5.
/* uint16_t frequency[8] =
   { 262, 294, 330, 349, 392, 440, 494, 523 }; */

volatile uint8_t new_state, old_state;
volatile uint8_t changed = 0;  // Flag for state change
volatile int16_t count = 0;		// Count to display
volatile uint8_t a, b;
volatile char x;
volatile char buf[11];
volatile int cycles;
enum state {START, STOP};
volatile int state = STOP;
volatile uint16_t pwm_width_timer1 = 3000; // halfway
const uint16_t MIN_PWM_WIDTH = 1500; // min
const uint16_t MAX_PWM_WIDTH = 4500; // max

int main(void) {

	sei();
	lcd_init();
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

	PCICR |= (1 << PCIE1); // timer1 pin change interrupt
	PCMSK1 |= (1 << PCINT12) | (1 << PCINT13);

	
	while (1) {                 // Loop forever
		if (changed) {
			OCR2A = count;
			OCR1B = pwm_width_timer1;
			lcd_moveto(1, 0);
			snprintf(buf, sizeof(buf), "%u", count);
			lcd_stringout(buf);
			changed = 0; // reset flag

			/*if (changed) {
			OCR2A = count;
			lcd_moveto(1, 0);
			snprintf(buf, sizeof(buf), "%d", count);
			lcd_stringout(buf);
			changed = 0; // Reset flag
		} */
		}
	}
}

void timer0_init(void) {
    // wat 
}

void timer1_init(void) {

	TCCR1B |= (1 << WGM12); // ctc
    TCCR1B |= (1 << CS12) | (1 << CS10); // 1024
	TIMSK1 |= (1 << OCIE1A); // 16-bit counter
	OCR1A = 62499; // overflow
	
}

void timer2_init(void)
{
    TCCR2A |= (0b11 << WGM20); 
    TCCR2A |= (0b10 << COM2A0); 
    OCR2A = 128;                
    TCCR2B |= (0b111 << CS20); 
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
	x = PIND;
	a = x & (1 << PD2);
	b = x & (1 << PD3);

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

	if(count > 99){
		count = 99;
	}
	else if (count < 0){
		count = 0;
	}

	if (new_state != old_state) {
		changed = 1;
		old_state = new_state;
	}
}

// Pin Change Interrupt Service Routine (ISR) to handle encoder input bits
// ISR(PCINT1_vect)
// {
//     static uint16_t start_time = 0; // Variable to store start time
    
//     // Check if start sensor is triggered
//     if (!(PINC & (1 << PC4))) {
//         // Start sensor is blocked
//         start_time = 0; // Clear start time
//         TCNT1 = 0; // Reset TIMER1 count register to zero
//         TCCR1B |= (1 << CS11) | (1 << CS10); // Start TIMER1 with prescaler 64
//     }
    
//     // Check if stop sensor is triggered
//     if (!(PINC & (1 << PC5))) {
//         // Stop sensor is blocked
//         if (start_time != 0) { // Check if start sensor was triggered
//             uint16_t elapsed_time = TCNT1; // Read TIMER1 count register
//             TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); // Stop TIMER1
            
//             // Calculate elapsed time in milliseconds using fixed-point arithmetic
//             // Multiply elapsed_time by 1000 and divide by TIMER1's frequency (Hz)
//             uint32_t elapsed_time_ms = (elapsed_time * 1000UL) / (16000000UL / 64);
            
//             // Calculate speed using distance between sensors
//             // Assuming distance in millimeters and speed in mm/sec
//             // Speed = Distance / Time
//             // For example, if distance is 1000mm and time is 500ms, speed = 1000 / 0.5 = 2000 mm/sec
//             uint32_t distance_mm = /* Distance between start and stop sensors */; // Fill in distance
//             uint32_t speed_mm_per_sec = (distance_mm * 1000UL) / elapsed_time_ms;
            
//             // Use speed and elapsed time as needed for further processing or display
//             // For now, you can print them to the LCD or serial output
            
//             // Reset start_time
//             start_time = 0;
//         }
//     }
    
//     // Check if timing has exceeded 4 seconds
//     if (TCNT1 >= 62499) {
//         // Timing has exceeded 4 seconds, stop TIMER1
//         TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); // Stop TIMER1
        
//         // Print message on LCD or serial output indicating timing expired
//         // For example, lcd_stringout("Timing expired");
//     }
// }
