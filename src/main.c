/*
   C Program for Distance Measurement using Ultrasonic Sensor and AVR Microocntroller
   */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "uart.h"

#ifndef F_CPU
#error "F_CPU undefined, please define CPU frequency in Hz in Makefile"
#endif

#define UART_BAUD_RATE      115200      

static volatile int pulse = 0;
static volatile int ii = 0;

uint16_t dist = 0;
uint16_t samples = 0;
char showa[16];
uint8_t rebase = 0;
uint16_t base = 0;


void ping(void);
void debug(void);

int main(void) {
    // init all ports
    DDRB = 0xFF;
    DDRC = 0xFF;
    DDRD = 0b11111000;
    _delay_ms(50);

    //initialize a interrupts
    EICRA = 1;
    EIMSK |= (1<<INT0);

    //initialize a timer
    TCCR1A = 0;

    //initialize a serial connection
    uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) ); 

    //enable interrupts
    sei();


    //_delay_ms(200);
    ping(); // initial 

    base = dist;
    debug();

    while(1){ 
        //_delay_ms(20);
        ping();
        debug();

        if (dist > (base - 4) && dist < (base + 4)){

        
        }
        else {
            rebase++;
            if (rebase == 50){
                uart_puts("NEB rebase\n");
                _delay_ms(500);
                rebase = 0;
                base = dist;
            }

        }

    } // while
}

void debug(void) {

    ltoa(dist,showa,10);
    uart_puts("Dist: ");
    uart_puts(showa);
    uart_puts("\t");

    ltoa(base,showa,10);
    uart_puts("base: ");
    uart_puts(showa);
    uart_puts("\n");


}

void ping(void) {

    uint8_t i = 0;
    for (i = 0; i < 5; i++){
        PORTD |= (1<<PD3);
        _delay_us(15);
        PORTD &= ~(1<<PD3);
        _delay_ms(8);
        samples += pulse;
    }
    dist = (samples / 5) / 58 / 2;
    samples = 0;
}

ISR(INT0_vect) // handles to timing after ping is sent
{
    if (ii == 1) // pulse ended, end timer and save timer value.
    {
        TCCR1B = 0;
        pulse = TCNT1;
        TCNT1 = 0;
        ii = 0;
    }
    if (ii == 0) // pulse is started, start the timer
    {
        TCCR1B |= (1<<CS11);
        ii = 1;
    }
}

