/*
 * fan running on 12V using push-pull MOSFET driver
 *
 *
 * 7 Watt ceilingfan bought in Phnom Penh, Cambodia for US$7.
 * Runs on 230 VAC. Has 4180 turns of 1x 0.2mm copperwire strand (740 Ohm)
 *
 *
 * Rewinded the coil:
 * ------------------
 * number of turns 0.4mm (diameter) copperwire: 210 
 * However it turns out that the necessary VDC must be 16.5V
 * on the DC supply side of the push-pull mosfet PCB.
 * 
 * For winding the coil: Take 2 strands of 0.4mm copperwire.
 * Length is 27meter. Wind bifilar on bobbin 210 turns.
 *
 */

#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_OSC F_CPU

#include "delay.h"


// Set fuses: 
FUSES =
{
   .high = 0xff,
   .low  = 0x7a,
};


/*
 * Octave:
 * > pwm=ceil(sin((0:180)*(2*pi/180))*220)
 * ~26Hz
 * 0.2A -> 3.3 Watt
 */
const uint8_t PM_lut_slow[] PROGMEM = {
//   0     1     2     3     4     5     6     7     8     9 
// Columns 1 through 10:
     0,    8,   16,   23,   31,   39,   46,   54,   61,   68,
// Columns 11 through 20:
    76,   83,   90,   97,  104,  110,  117,  124,  130,  136,
// Columns 21 through 30:
   142,  148,  153,  159,  164,  169,  174,  178,  183,  187,
// Columns 31 through 40:
   191,  195,  198,  201,  204,  207,  210,  212,  214,  216,
// Columns 41 through 50:
// 217   218   219   220   220   220   220   220   219   218
   217,  218,  219,  220,  230,  240,  230,  220,  219,  218, // edit
// Columns 51 through 60:
   217,  216,  214,  212,  210,  207,  204,  201,  198,  195,
// Columns 61 through 70:
   191,  187,  183,  178,  174,  169,  164,  159,  153,  148,
// Columns 71 through 80:
   142,  136,  130,  124,  117,  110,  104,   97,   90,   83,
// Columns 81 through 90:
    76,   68,   61,   54,   46,   39,   31,   23,   16,    8,
};

/*
 * Octave:
 * > pwm=ceil(sin((0:150)*(2*pi/150))*240)
 * ~31Hz
 * printf('%d, ',pwm(1:10)); printf('\n');
 *
 * 0.3A -> 5 Watt
 */
const uint8_t PM_lut_med[] PROGMEM = {
//   0     1     2     3     4     5     6     7     8     9  
// Columns 1 through 10:
     0,   11,   21,   31,   41,   50,   60,   70,   79,   89,
// Columns 11 through 20:
    98,  107,  116,  125,  133,  142,  150,  157,  165,  172,
// Columns 21 through 30:
   179,  185,  192,  198,  203,  208,  213,  218,  222,  225,
// Columns 31 through 40:
//   229,  232,  234,  236,  238,  239,  240,  240,  240,  240,
   229,  232,  234,  236,  238,  239,  240,  245,  245,  240, // edit
// Columns 41 through 50:
   239,  238,  236,  234,  232,  229,  225,  222,  218,  213,
// Columns 51 through 60:
   208,  203,  198,  192,  185,  179,  172,  165,  157,  150,
// Columns 61 through 70:
   142,  133,  125,  116,  107,   98,   89,   79,   70,   60,
// Columns 71 through 80:
    50,   41,   31,   21,   11,
};


/*
 * Octave:
 * > pwm=ceil(sin((0:124)*(2*pi/124))*255)
 * ~37Hz
 * 0.4A -> 6.6 Watt
 */
const uint8_t PM_lut_fast[] PROGMEM = {
//   0     1     2     3     4     5     6     7     8     9  
// Columns 1 through 10:
     0,   13,   26,   39,   52,   64,   77,   89,  101,  113,
// Columns 11 through 20:
   124,  135,  146,  157,  167,  176,  185,  194,  202,  210,
// Columns 21 through 30:
   217,  223,  229,  235,  240,  244,  247,  250,  253,  254,
// Columns 31 through 40:
   255,  255,  255,  254,  253,  250,  247,  244,  240,  235,
// Columns 41 through 50:
   229,  223,  217,  210,  202,  194,  185,  176,  167,  157,
// Columns 51 through 60:
   146,  135,  124,  113,  101,   89,   77,   64,   52,   39,
// Columns 61 through 70:
    26,   13,
};

/*
 * Octave:
 * > pwm=ceil(sin((0:114)*(2*pi/114))*255)
 * ~41Hz
 * 0.44A  -> 7 Watt
 */
const uint8_t PM_lut_faster[] PROGMEM = {
//   0     1     2     3     4     5     6     7     8     9  
// Columns 1 through 10:
     0,   15,   29,   42,   56,   70,   83,   96,  109,  122,
// Columns 11 through 20:
   134,  146,  157,  168,  178,  188,  197,  206,  214,  221,
// Columns 21 through 30:
   228,  234,  239,  244,  248,  251,  253,  255,  255,  255,
// Columns 31 through 40:
   255,  253,  251,  248,  244,  239,  234,  228,  221,  214,
// Columns 41 through 50:
   206,  197,  188,  178,  168,  157,  146,  134,  122,  109,
// Columns 51 through 60:
    96,   83,   70,   56,   42,   29,   15,
};



int button_is_pressed_and_released(void)
{
    if(bit_is_clear(PINB, PB3)) {
        delay_ms(25);
        if(bit_is_clear(PINB, PB3)) { // valid press now

            delay_ms(250);

            while(1) { // wait until release
                if(bit_is_set(PINB, PB3)) { // wait until release
                    delay_ms(25);
                    if(bit_is_set(PINB, PB3)) { // valid release
                        return 1;
                    }
                }
            }
        }
    }
    return 0;
}

volatile uint8_t half_period=0;
volatile uint8_t idx=0;
volatile uint8_t state=0;
volatile uint8_t button=0;
volatile uint8_t clkdiv=0;

int main(void) 
{
    uint8_t sreg;
    PORTB = 0xf; // PB0, PB1 to ON  (mosfets OFF) // PB3 is input button
    DDRB  = 0x33;

    TCCR0A = (1<<WGM01)|(1<<WGM00); // Fast PWM. // first normal port behv 0x0x
                         // in ISR then toggle OC0A  or  OC0A   0bAABBxxxx  (AA or BB  11 or 00)
    TCCR0B |= (1<<CS00); // clock/1 (no prescaling)
    OCR0A = 1; // follow LUT later
    OCR0B = 1; // follow LUT later
    TIMSK0 |= (1<<TOIE0); //Enable overflow interrupt.


    TCCR0A &= ~((1<<COM0B1)|(1<<COM0B0)); // disable port OC0B
    PORTB |= (1<<PB1);                    //    "     "    "

    TCCR0A |=  ((1<<COM0A1)|(1<<COM0A0)); // enable  port OC0A

    sei(); // enable interrupts

    while(1) {
        if(button_is_pressed_and_released()) {
            sreg=SREG; cli(); // atomic begin:
            state++;
            //state&=3;
            if(state>4) state=0;
            SREG=sreg; // sei() (atomic end)
        }
    }
}

#define PORT_A_disable() {TCCR0A &= ~((1<<COM0A1)|(1<<COM0A0)); PORTB |= (1<<PB0);}
#define PORT_A_enable()  {TCCR0A |=  ((1<<COM0A1)|(1<<COM0A0));}

#define PORT_B_disable() {TCCR0A &= ~((1<<COM0B1)|(1<<COM0B0)); PORTB |= (1<<PB1);}
#define PORT_B_enable()  {TCCR0A |=  ((1<<COM0B1)|(1<<COM0B0));}

// timer overflow ISR:
ISR(TIM0_OVF_vect) 
{
    // running this: 1200000/256=4687.5 per second
    // running this: 9600000/256=37500 per second
    uint8_t pwm,max_idx;

    // On CLK=9.6 MHz then using a clkdiv=8 here:
    clkdiv++;
    if(clkdiv < 8) return;
    clkdiv=0;


    if(state==1) {
        pwm=pgm_read_byte(&PM_lut_slow[idx]);
        max_idx=90;
    } else if(state==2) {
        pwm=pgm_read_byte(&PM_lut_med[idx]);
        max_idx=75;
    } else if(state==3) {
        pwm=pgm_read_byte(&PM_lut_fast[idx]);
        max_idx=62;
    } else if(state==4) {
        pwm=pgm_read_byte(&PM_lut_faster[idx]);
        max_idx=57;
    } else { // state==0: turn off
        idx=0;
        half_period=0;
        OCR0A = 0;
        OCR0B = 0;
        PORT_A_disable();
        PORT_B_disable();
        return;
    }
    idx++;
    if(idx>=max_idx) {
        idx=0;
        half_period++;
        half_period &=1;

        if(half_period==0) {
            PORT_B_disable();
            PORT_A_enable();
        } else {
            PORT_A_disable();
            PORT_B_enable();
        }
    }
    OCR0A = pwm;
    OCR0B = pwm;
}

