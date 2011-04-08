#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#define FRAMERATEADDR   9
#define FRAMERATEDFLT   16

#define INPUT_STATE_TIMED   0
#define INPUT_STATE_TIMING  1

#define LENGTH(array)   (sizeof(array) / sizeof(array[0]))
//#define PERIOD(f)       (250000/8/f) // XXX
#define PERIOD(f)       (250000/f)

#define BUTTON_DDR      DDRB
#define BUTTON_PIN      PINB
#define BUTTON_1        (1 << PB0)
#define BUTTON_2        (1 << PB1)
#define BUTTON_3        (1 << PB2)

#define CAMERA_DDR      DDRA
#define CAMERA_PIN      PINA
#define CAMERA_PORT     PORTA

#define LED_DDR         DDRD
#define LED_PIN         PIND
#define LED_PORT        PORTD
#define LED_GREEN       (1 << PD6)
#define LED_RED         (1 << PD7)

#define TRIGGER_DDR     DDRD
#define TRIGGER_PIN     PIND
#define TRIGGER_PORT    PORTD
#define TRIGGER_INT     INT0
#define TRIGGER_IN      (1 << PD2)
#define TRIGGER_OUT     (1 << PD5)

static volatile uint8_t multipliers[4] = {0, 0, 0, 0};
static volatile uint8_t time = 0;
static volatile uint8_t cameras = 0b10101010;
static volatile uint16_t framerate = 16;

ISR(INT0_vect) {
    CAMERA_PORT = cameras;
    _delay_us(32);
    CAMERA_PORT = 0;

    ++time;
    cameras = 0;
    for(uint8_t i = 0; i < 4; ++i)
        cameras |= (!(time % (8 >> multipliers[i])) << (2 * i + 1));

    TCCR2 |= (1 << CS21); // Start Timer2 with CK/8 prescaler (2 MHz).
}

ISR(TIMER0_COMP_vect) {
    TRIGGER_PORT &= ~TRIGGER_OUT;
    TCCR0 &= ~((1 << CS01) | (1 << CS00)); // Stop Timer0.
}

ISR(TIMER1_COMPA_vect) {
    // XXX
    CAMERA_PORT ^= cameras;
    //_delay_us(32);
    //CAMERA_PORT = 0;

    return;

    static uint8_t t = 0;

    TRIGGER_PORT |= TRIGGER_OUT;

    ++t;

    if(t % 8) {
        OCR0 = 1;
    } else {
        OCR0 = 32;
        //LED_PORT ^= LED_GREEN;
    }

    TCNT0  = 0;           // Clear Timer0.
    TCCR0 |= (1 << CS01) | (1 << CS00); // Start Timer0 with CK/8 prescaler (2 MHz).
}

ISR(TIMER2_COMP_vect) {
    if(TRIGGER_PIN & TRIGGER_IN) {
        time = 0;
    } else {
    }

    TCCR2 &= ~(1 << CS21); // Stop Timer2;
}

static void
camera_output_init(void) {
    CAMERA_DDR = 0b10101010;
}

static void
led_init(void) {
    LED_DDR |= LED_GREEN | LED_RED;
}

static void
trigger_input_init(void) {
    TRIGGER_DDR &= ~TRIGGER_IN;

    MCUCR |= (1 << ISC01) | (1 << ISC00); // Interrupt on rising edge.
    GICR  |= (1 << TRIGGER_INT); // Enable interrupt on trigger pin.

    /* Set up Timer2. */
    TCCR2 = (1 << WGM21);
    OCR2 = 16;
    TIMSK |= (1 << OCIE2); // Interrupt on compare match.
}

static void
trigger_output_init(uint16_t framerate) {
    TRIGGER_DDR |= TRIGGER_OUT;

    /*
     * Timer1 generates an interrupt at 8x the base framerate.
     * When the interrupt is handled, the output trigger pin is set high.
     */

    TCCR1B = (1 << WGM12)   // Clear timer on compare match.
           | (0 << CS12)    // CK/64 prescaler, 250 kHz.
           | (1 << CS11)
           | (1 << CS10);

    TIMSK |= (1 << OCIE1A); // Interrupt on compare match.
    OCR1A  = PERIOD(framerate);

    /*
     * Timer0 generates an interrupt after counting up to a predefined value.
     * This value is specified in the Timer1 Output Compare Match handler.
     *
     * When the Timer0 Output Compare Match interrupt is handled, the output
     * trigger pin is set low.
     */

    TCCR0  = (1 << WGM01); // Clear timer on compare match.
    TIMSK |= (1 << OCIE0); // Interrupt on compare match.
}

static void
button_handle_release(uint8_t button) {
}

static void
button_handle_press(uint8_t button) {
    switch(button) {
    case BUTTON_1:
        if(framerate <= 500)
            OCR1A = PERIOD(++framerate);
        break;
    case BUTTON_2:
        // Save framerate.
        LED_PORT |= LED_RED;
        eeprom_busy_wait();
        eeprom_write_byte((uint8_t *)FRAMERATEADDR, framerate);
        LED_PORT &= ~LED_RED;
        break;
    case BUTTON_3:
        if(framerate > 2)
            OCR1A = PERIOD(--framerate);
        break;
    default:
        break;
    }
}

static void
button_handle_hold(uint8_t button) {
    switch(button) {
    case BUTTON_1:
        if(framerate <= 500)
            OCR1A = PERIOD(++framerate);
        break;
    case BUTTON_3:
        if(framerate > 2)
            OCR1A = PERIOD(--framerate);
        break;
    default:
        break;
    }
}

int
main(void) {
    uint8_t button_jiffies[3] = {0, 0, 0};
    uint8_t button_held = 0b00000000;
    uint8_t button_state = 0b00000000;

    eeprom_busy_wait();
    framerate = eeprom_read_byte((const uint8_t *)FRAMERATEADDR);
    if(framerate == 0xff) {
        eeprom_busy_wait();
        eeprom_write_byte((uint8_t *)FRAMERATEADDR, FRAMERATEDFLT);
    }

    camera_output_init();

    led_init();

    trigger_input_init();
    trigger_output_init(framerate);

    LED_PORT ^= LED_RED;
    _delay_ms(100);
    LED_PORT ^= LED_RED;
    _delay_ms(100);
    LED_PORT ^= LED_RED;
    _delay_ms(100);
    LED_PORT ^= LED_RED;
    _delay_ms(100);
    LED_PORT ^= LED_RED;
    _delay_ms(100);
    LED_PORT ^= LED_RED;

    sei();

    for (;;) {
        _delay_ms(10);

        for(uint8_t i = 0; i < 3; ++i) {
            uint8_t button = (1 << i);

            if(BUTTON_PIN & button) {
                button_handle_release(button);
                button_state &= ~button;
                button_held &= ~button;
                button_jiffies[i] = 0;
            } else {
                ++button_jiffies[i];

                if(!(button_state & button)) {
                    button_handle_press(button);
                    button_state |= button;
                }

                if(button_jiffies[i] > 63)
                    button_held |= button;

                if(button_held & button)
                    button_handle_hold(button);
            }
        }
    }
}
