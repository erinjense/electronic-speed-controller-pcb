/* Direction and Speed Finite State Machine
 *
 * Designed for pwmCat-motorBoard
 *
 * Receives a Bluetooth Low Energy (BLE) signal
 * from an iPhone app or any other BLE capable device
 * connected and transmiting the correct ASCII letter/number.
 *
 * Code was designed for discrete signals. e.g. ( The user
 * presses a forward arrow on an app. The app sends 'F' to 
 * the BLE module on motor board. The atmega328p receives
 * signal from the BLE module via UART.
 * This code then drives motors to go in a forward direction.
 * The code will remain in forward direction unless the user
 * changes direction.)
 *
 * DIRECTION SIGNALS:
 *
 * forward: 'F'
 * backward: 'B'
 * right: 'R'
 * left: 'L'
 *
 * SPEED SIGNALS:
 *
 * stop : '0'
 * slow : '1'
 * mid : '2'
 * max : '3'
 *
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include "pinDefines.h"
#include <util/delay.h>
#include "USART.h"
#include "PWM.h"

#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define USART_BAUDRATE 9600

/* Delays so that the motors don't instantly go from max speed
 * to stop or change direction instatnly at high speeds.
 * This helps prevent current spikes
 */
#define MOTOR_SYNC_DELAY 5
#define SPEED_STEP_DELAY 2

#define MAX_SPEED 50
#define MED_SPEED 100
#define SLOW_SPEED 191
#define STOP_SPEED 255
#define SAFE_SPEED 150

/* Changes speed in steps with safety delays */
void change_speed(uint8_t new_speed) {

  while (OCR0A != new_speed) {
    if (OCR0A < new_speed) { 
      OCR0A++; 
      OCR0B = OCR0A; 
      OCR1A = OCR0A;
      OCR1B = OCR0A;
      _delay_ms(SPEED_STEP_DELAY);
    }
    else if (OCR0A > new_speed) { 
      OCR0A--; 
      OCR0B = OCR0A; 
      OCR1A = OCR0A;
      OCR1B = OCR0A;
      _delay_ms(SPEED_STEP_DELAY);
    }
  }

}
int main(void) {
  uint8_t rx_byte;
  uint8_t current_speed;
  uint8_t last_speed;
  /* Data Direction inits for PortB and PortD */
  DDRB |= ((1 << PB1) | (1 << PB2) | (1 << PB5));
  DDRD |= ((1 << PD2) | (1 << PD5) | (1 << PD6) | (1 << PD7));
  /* nSleep ON for DRV8833 = motor output ON */
  PORTD |= (1 << PD7);

  initPWM();
  initUSART();

  current_speed = 150;
  last_speed = current_speed;

  OCR0A = current_speed;
  OCR0B = OCR0A;
  OCR1A = OCR0A;
  OCR1B = OCR0A;

  printString("AT+NAMEpwmCat");

  while(1) {

    /* No user-signal, STOP the motors
     * This would be handy if user control app
     * continuously sends a signal if user
     * continuously holds down a button, but
     * the motors need to stop if user is not
     * sending a signal.
     */
   /* 
    if ( (UCSR0A & 0b10000000) == 0b00000000) {
      change_speed(STOP_SPEED);  
    }
    */

    // Wait forever until user-signal and get it
    rx_byte = receiveByte();

    switch (rx_byte) {

      // Forward
      case 'F':

        /* Always enable motor driver. In case user is
         * going from stop and then changed direction */
        PORTD |= (1 << PD7);

        if (current_speed == STOP_SPEED) {
          forwardPWM();
          current_speed = last_speed;
          change_speed(current_speed);
        }
        /* max speed is 0 and min speed is 255
         * This checks to see if current speed 
         * is higher than a safe speed.
         * The comparison operators are non-intuitive
         */
        else if (current_speed < SAFE_SPEED) {
          change_speed(SAFE_SPEED);
          forwardPWM();
          change_speed(current_speed);
        }
        else { forwardPWM(); }
        break;

      // Backward
      case 'B':
        PORTD |= (1 << PD7);
        if (current_speed == STOP_SPEED) {
          reversePWM();
          current_speed = last_speed;
          change_speed(current_speed);
        }
        else if (current_speed < SAFE_SPEED) {
          change_speed(SAFE_SPEED);
          reversePWM();
          change_speed(current_speed);
        }
        else { reversePWM(); }
        break;

      // Right
      case 'R':
        PORTD |= (1 << PD7);
        if (current_speed == STOP_SPEED) {
          rightPWM();
          current_speed = last_speed;
          change_speed(current_speed);
        }
        else if (current_speed < SAFE_SPEED) {
          change_speed(SAFE_SPEED);
          rightPWM();
          change_speed(current_speed);
        }
        else { rightPWM(); }
        break;
      // Left
      case 'L':
        PORTD |= (1 << PD7);
        if (current_speed == STOP_SPEED) {
          leftPWM();
          current_speed = last_speed;
          change_speed(current_speed);
        }
        else if (current_speed < SAFE_SPEED) {
          change_speed(SAFE_SPEED);
          leftPWM();
          change_speed(current_speed);
        }
        else { leftPWM(); }
        break;
    }

    // Change Speed State
    switch(rx_byte) {

      // Stop
      case '0':
        /* Disable motor driver output */
        PORTD &= ~(1 << PD7);
        /* keep track of last speed, so that
         * if user pressed stop and then changes
         * direction. Then last speed remains same.*/
        last_speed = current_speed;
        current_speed = STOP_SPEED;
        change_speed(current_speed); 
        break;
      // Slowest
      case '1':
        current_speed = SLOW_SPEED;
        change_speed(current_speed);
        break;
      // Medium
      case '2':
        current_speed = MED_SPEED;
        change_speed(current_speed); 
        break;
      // Fastest
      case '3':
        current_speed = MAX_SPEED;
        change_speed(current_speed); 
        break;
    }

  }
  /* Code never reaches here */
  return(0);
}
