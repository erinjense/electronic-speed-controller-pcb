/**************************************
* RC Tank Electronic Speed Control
* Utilizes Timer/Counter Module for PWM
* on atmega328p.  TC0 and TC1 registers
*
* Multidisplinary Engineering Design
* Written by: Aaron Jense
*
**************************************/

// ----- PREAMBLE -------//
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "pinDefines.h"
// ------ FUNCTIONS -------//

void initPWM(void) {
    /* PWM Setup for TC0 */
    //  PWM_frequency = clock_speed/(2*Prescaler_value*TOP_value);
    TCCR0A |= (1 << WGM01) | (1 << WGM00);     /* Fast PWMmode, TOP =OCR0A, update OCR0x at BOTTOM*/
    TCCR0A &= ~(1 << COM0B1 | (1 << COM0B0)); /* Normal port operation, OC0B disconnected */
    TCCR0A &= ~(1 << COM0A1 | (1 << COM0A0)); /* Normal port operation, OC0A disconnected */
    TCCR0B |= (1 << CS01);                   /* set prescaler to 8 and starts PWM */
    /* PWM Setup for TC1 */
    TCCR1A |= (1 << WGM12) | (1 << WGM10);     /* Fast PWMmode, TOP = OCR1A, update OCR1x at BOTTOM*/
    TCCR1B |= (1 << CS11);                   /* set prescaler to 8 and starts PWM */
}

/* function names could be easily changed if motor output does not
 * correspond to desired direction or the motor leads could be swapped */

void forwardPWM(void){
  // Left motor, from perspective of battery terminal on top
  TCCR0A |= (1 << COM0B1);
  TCCR0A &= ~(1 << COM0B0); /* OC0B non-inverting mode */
  TCCR0A &= ~(1 << COM0A1 | (1 << COM0A0)); /* Normal port operation, OC0A disconnected */
  // Left motor, from perspective of battery terminal on top
  TCCR1A |= (1 << COM1B1);
  TCCR1A &= ~(1 << COM1B0); /* OC1B non-inverting mode */
  TCCR1A &= ~(1 << COM1A1 | (1 << COM1A0)); /* Normal port operation, OC1A disconnected */
  // enables for DRV8833
  PORTD &= ~(1 << PD5);
  PORTD |= (1 << PD6);
  PORTB &= ~(1 << PB2);
  PORTB |= (1 << PB1);
}

void reversePWM(void) {
  // Right motor, from perspective of battery terminal on top
  TCCR0A |= (1 << COM0A1); 
  TCCR0A &= ~(1 << COM0A0); /* OC0A non-inverting mode */
  TCCR0A &= ~(1 << COM0B1 | (1 << COM0B0)); /* Normal port operation, OC0B disconnected */
  PORTD &= ~(1<< PD6);
  PORTD |= (1 << PD5);
  // Left motor, from perspective of battery terminal on top
  TCCR1A |= (1 << COM1A1); 
  TCCR1A &= ~(1 << COM1A0); /* OC0A non-inverting mode */
  TCCR1A &= ~(1 << COM1B1 | (1 << COM1B0)); /* Normal port operation, OC1B disconnected */
  PORTB &= ~(1<< PB1);
  PORTB |= (1 << PB2);
}

void leftPWM(void) {
  // Right motor, from perspective of battery terminal on top
  TCCR0A |= (1 << COM0A1); 
  TCCR0A &= ~(1 << COM0A0); /* OC0A non-inverting mode */
  TCCR0A &= ~(1 << COM0B1 | (1 << COM0B0)); /* Normal port operation, OC0B disconnected */
  PORTD &= ~(1<< PD6);
  PORTD |= (1 << PD5);

  TCCR1A |= (1 << COM1B1);
  TCCR1A &= ~(1 << COM1B0); /* OC1B non-inverting mode */
  TCCR1A &= ~(1 << COM1A1 | (1 << COM1A0)); /* Normal port operation, OC1A disconnected */
  PORTB &= ~(1 << PB2);
  PORTB |= (1 << PB1);
}

void rightPWM(void) {
  // Left motor, from perspective of battery terminal on top
  TCCR1A |= (1 << COM1A1); 
  TCCR1A &= ~(1 << COM1A0); /* OC0A non-inverting mode */
  TCCR1A &= ~(1 << COM1B1 | (1 << COM1B0)); /* Normal port operation, OC1B disconnected */
  PORTB &= ~(1<< PB1);
  PORTB |= (1 << PB2);

  // Left motor, from perspective of battery terminal on top
  TCCR0A |= (1 << COM0B1);
  TCCR0A &= ~(1 << COM0B0); /* OC0B non-inverting mode */
  TCCR0A &= ~(1 << COM0A1 | (1 << COM0A0)); /* Normal port operation, OC0A disconnected */
  PORTD &= ~(1 << PD5);
  PORTD |= (1 << PD6);
}
