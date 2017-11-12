/*
   C Program for kitchen counter light switch based gestures
   */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "uart.h"

#ifndef F_CPU
#error "F_CPU undefined, please define CPU frequency in Hz in Makefile"
#endif

#define UART_BAUD_RATE      115200      

static volatile int pulse = 0;
static volatile int ii = 0;
static volatile uint8_t timer2_overflow = 0;

uint16_t dist = 0;
uint16_t samples = 0;
uint16_t base = 0;
char showa[16];
uint8_t state = 0;
uint8_t rebase = 0;

void ping(void);
void debug(void);
void WDT_init(void);
void timer_init(void);

int main(void) {


    // init all ports
    DDRB = 0xFF;
    DDRC = 0xFF;
    DDRD = 0b11111000;

    //initialize a interrupts
    EICRA = 1;
    EIMSK |= (1<<INT0);

    //initialize a serial connection
    uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) ); 

    //enable interrupts
    sei();

    timer_init();

    ping(); // initial 
    base = dist;


    while(1) { 
        ping();
        if (rebase == 1) { // check if rebase is done and corrects a state
                state--;
                rebase = 0;
        }
        
        else if (abs(dist - base) > 20){ // if distance have changed, move to next state.
            if (state == 0) state = 1;
            else if (state == 2) state = 3;
            else if (state == 4) state = 5;
        }
        else {
            wdt_reset();
            
            if (state == 1) {
                PORTB |= (1 << PB5); //lights on
                state = 2;
            }
            else if (state == 3) { // off gesture initialized, start a timeout timer.
                TCNT2 = 0;
                TCCR2B |= (1 << CS22)|(1 << CS21)|(1 << CS20);
                state = 4;
            }
            else if (state == 5) { // off gesture occur
                PORTB &= !(1 << PB5); // lights off
                TCNT2 = 0;
                TCCR2B = 0;
                state = 0;
            }
        } 
        debug();
    }
    return 0;
}

void timer_init(void) {
    //initialize a timer1 for HC-SR04
    TCCR1A = 0;
   
    //initialize a timer2 for gesture
    TCNT2 = 0;
    TIMSK2 |= (1 << TOIE2);

    //disable interrupts
    cli();
    //Enable watchdog
    WDTCSR |= (_BV(WDCE) | _BV(WDE));
    WDTCSR =   _BV(WDIE) | _BV(WDP3);
    //Enable global watchdog
    sei();


}

void debug(void) { // some serial prints
    ltoa(dist,showa,10);
    uart_puts("Dist: ");
    uart_puts(showa);
    uart_puts("\t");

    ltoa(base,showa,10);
    uart_puts("Base: ");
    uart_puts(showa);
    uart_puts("\t");
    
    ltoa(state,showa,10);
    uart_puts("state: ");
    uart_puts(showa);
    uart_puts("\t");

    uart_puts("\n");
}

void ping(void) { // makes 5 pings and averages a value.
    uint8_t i = 0;
    for (i = 0; i < 5; i++){
        PORTD |= (1<<PD3);
        _delay_us(15);
        PORTD &= ~(1<<PD3);
        _delay_ms(8);
        samples += pulse;
    }
    dist = (samples / 5); 
    samples = 0;
}

ISR(TIMER2_OVF_vect) { // handles a timeout for gestures
    if (timer2_overflow < 122) timer2_overflow++;
    else {
        state = 2;
        TCNT2 = 0;
        TCCR2B = 0;
        timer2_overflow = 0;
    }
}

ISR(INT0_vect) { // handles to timing after ping is sent
    if (ii == 1) { // pulse ended, end timer and save timer value.
        TCCR1B = 0;
        pulse = TCNT1;
        TCNT1 = 0;
        ii = 0;
    }
    if (ii == 0) { // pulse is started, start the timer
        TCCR1B |= (1<<CS11);
        ii = 1;
    }
}

ISR(WDT_vect) { // handle watchdog interrupt to rebase
    uart_puts("rebase \n");
    base = dist;
    rebase = 1;
}

