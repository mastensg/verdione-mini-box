/*
 * Copyright (c) 2010, 2011 Simula Research Laboratory AS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by Simula Research Laboratory AS.
 * 4. Neither the name of Simula Research Laboratory AS nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Simula Research Laboratory AS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Simula Research Laboratory AS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/twi.h>

#define FRAMERATEADDR   10 // 2 bytes
#define MULTIPLIERSADDR 12 // 4 bytes
#define BRIGHTNESSADDR  16 // 1 byte
#define CONTRASTADDR    17 // 1 byte

#define INPUT_STATE_TIMED   0
#define INPUT_STATE_TIMING  1

#define LENGTH(array)   (sizeof(array) / sizeof(array[0]))
#define PERIOD(f)       (250000/8/f)

#define BUTTON_DDR      DDRB
#define BUTTON_PIN      PINB
#define BUTTON_PORT     PORTB
#define BUTTON_1        (1 << PB0)
#define BUTTON_2        (1 << PB1)
#define BUTTON_3        (1 << PB2)

#define CAMERA_DDR      DDRA
#define CAMERA_PIN      PINA
#define CAMERA_PORT     PORTA
#define CAMERA_MASK     0b10101010

#define LCD_DISPLAY_ON      0x41
#define LCD_DISPLAY_OFF     0x42
#define LCD_CURSOR_SET      0x45
#define LCD_CURSOR_HOME     0x46
#define LCD_CURSOR_ON       0x47
#define LCD_CURSOR_OFF      0x48
#define LCD_CURSOR_LEFT     0x49
#define LCD_CURSOR_RIGHT    0x4a
#define LCD_BLINK_ON        0x4b
#define LCD_BLINK_OFF       0x4c
#define LCD_BACKSPACE       0x4e
#define LCD_CLEAR_SCREEN    0x51
#define LCD_CONTRAST_SET    0x52
#define LCD_BRIGHTNESS_SET  0x53
#define LCD_LOAD_CUSTOM     0x54
#define LCD_SCROLL_LEFT     0x55
#define LCD_SCROLL_RIGHT    0x56

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

#define TWI_SLA_DISP    0x50

static volatile uint8_t beats = 0;

static uint8_t cursor;
static uint16_t framerate;
static uint8_t multipliers[4];

ISR(INT0_vect) {
    static volatile uint8_t cameras = 0;

    LED_PORT ^= LED_RED;
    CAMERA_PORT = cameras;
    _delay_us(32);
    CAMERA_PORT = 0;

    ++beats;
    cameras = 0;
    for(uint8_t i = 0; i < 4; ++i)
        cameras |= (!(beats % (8 >> multipliers[i])) << (2 * i + 1));

    TCCR2 |= (1 << CS21); // Start Timer2 with CK/8 prescaler (2 MHz).
}

ISR(TIMER0_COMP_vect) {
    TRIGGER_PORT |= TRIGGER_OUT;

    /* Stop Timer0. */
    TCCR0 &= ~((1 << CS01) | (1 << CS00));
}

/* Base frequency (times 8) handler. */
ISR(TIMER1_COMPA_vect) {
    /* t counts the times 8 base beats. */
    static uint8_t t = 0;

    TRIGGER_PORT &= ~TRIGGER_OUT;

    ++t;

    /* Send a long pulse every 8 beats. */
    if(t % 8) {
        OCR0 = 1;
    } else {
        OCR0 = 32;
    }

    /* Restart Timer0 with CK/8 prescaler (2 MHz). */
    TCNT0 = 0;
    TCCR0 |= (1 << CS01) | (1 << CS00);
}

ISR(TIMER2_COMP_vect) {
    if(TRIGGER_PIN & TRIGGER_IN)
        beats = 0;

    /* Stop Timer2. */
    TCCR2 &= ~(1 << CS21);
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
trigger_output_init() {
    TRIGGER_DDR |= TRIGGER_OUT;

    /*
     * Timer1 generates an interrupt at 8x the base framerate.
     * The interrupt handler sets the output trigger pin low.
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
     * The interrupt handler sets the output trigger pin high.
     */

    TCCR0  = (1 << WGM01); // Clear timer on compare match.
    TIMSK |= (1 << OCIE0); // Interrupt on compare match.
}

static void
twi_send_byte(uint8_t byte) {
start:
    // Send start condition.
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while(!(TWCR & (1 << TWINT)));
    switch(TW_STATUS) {
    case TW_REP_START:
    case TW_START:
        break;

    default:
        return;
    }

    // Send SLA+W.
    TWDR = TWI_SLA_DISP | TW_WRITE;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while(!(TWCR & (1 << TWINT)));
    switch(TW_STATUS) {
    case TW_MT_SLA_ACK:
        break;

    case TW_MT_SLA_NACK:
    case TW_MT_ARB_LOST:
        goto start;

    default:
        goto end;
    }

    // Send character.
    TWDR = byte;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while(!(TWCR & (1 << TWINT)));

end:
    // Send stop condition.
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

static void
lcd_command(uint8_t command) {
    twi_send_byte(0xfe);
    twi_send_byte(command);
}

static void
lcd_putchar(uint8_t c) {
    twi_send_byte(c);
}

static void
lcd_puts(char *s) {
    uint8_t i = 0;
    while(s[i]) {
        if(s[i] == '\n') {
            lcd_command(LCD_CURSOR_SET);
            lcd_putchar(0x40);
        } else {
            lcd_putchar(s[i]);
        }

        ++i;
    }
}

static void
lcd_putdigit(uint8_t n) {
    lcd_putchar('0' + n);
}

static void
lcd_putuint(uint16_t n) {
    if(n >= 1000)
        lcd_putdigit(n / 1000);

    if(n >= 100)
        lcd_putdigit((n % 1000) / 100);

    if(n >= 10)
        lcd_putdigit((n % 100) / 10);

    lcd_putdigit(n % 10);
}

static void
lcd_message(char *msg) {
    lcd_command(LCD_CLEAR_SCREEN);
    lcd_command(LCD_CURSOR_HOME);
    lcd_puts(msg);
}

static void
lcd_init(void) {
    // Set bitrate.
    TWBR = 77; // XXX: Make more dynamic.

    lcd_command(LCD_DISPLAY_ON);
    lcd_command(LCD_BLINK_ON);

    cursor = 0x40;

    lcd_message("Mini trigger box\n   Simula  RL   ");
}

static void
lcd_update(void) {
    lcd_command(LCD_CLEAR_SCREEN);

    CAMERA_PORT |= ~CAMERA_MASK;
    for(uint8_t i = 0; i < 4; ++i) {
        lcd_command(LCD_CURSOR_SET);
        lcd_putchar(4 * i);
        lcd_putdigit(1 << multipliers[i]);
        lcd_putchar('x');

        if(!(CAMERA_PIN & (1 << (2 * i))))
            lcd_putchar('C');
        else
            lcd_putchar(' ');
    }

    lcd_command(LCD_CURSOR_SET);
    lcd_putchar(0x40);
    lcd_putuint(framerate);
    lcd_puts("hz");

    lcd_command(LCD_CURSOR_SET);
    twi_send_byte(0x40 + 4 * 2);
    lcd_puts("sto");
    lcd_puts(" menu");

    lcd_command(LCD_CURSOR_SET);
    twi_send_byte(cursor);
}

static void
restore_settings(void) {
    // Framerate.
    eeprom_busy_wait();
    framerate = eeprom_read_word((const uint16_t *)FRAMERATEADDR);
    if(framerate == 0xffff)
        framerate = 1;

    // Multipliers.
    eeprom_busy_wait();
    eeprom_read_block(multipliers, (uint16_t *)MULTIPLIERSADDR, sizeof multipliers);
    for(uint8_t i = 0; i < LENGTH(multipliers); ++i)
        if(multipliers[i] > 3)
            multipliers[i] = 0;
}

static void
store_settings(void) {
    // Framerate.
    eeprom_busy_wait();
    eeprom_write_word((uint16_t *)FRAMERATEADDR, framerate);

    // Multipliers.
    eeprom_busy_wait();
    eeprom_write_block(multipliers, (uint16_t *)MULTIPLIERSADDR, sizeof multipliers);
}

static void
button_handle_release(uint8_t button) {
}

static void
button_handle_press(uint8_t button) {
    if(button == BUTTON_2) {
        switch(cursor) {
        case 12:
            cursor = 0x40;
            break;
        case 0x40:
            cursor = 0x40 + 8;
            break;
        case 0x40 + 12:
            cursor = 0;
            break;
        default:
            cursor += 4;
            break;
        }

        lcd_update();
        return;
    }

    uint8_t m;

    switch(cursor) {
    case 0x40:
        // Set framerate.
        if(button == BUTTON_1) {
            if(framerate <= 500)
                OCR1A = PERIOD(++framerate);
        } else {
            if(framerate > 1)
                OCR1A = PERIOD(--framerate);
        }
        break;
    case 0x40 + 8:
        // Store settings.
        store_settings();
        lcd_message("Stored!");
        _delay_ms(1000);
        break;
    case 0x40 + 12:
        // Enter menu.
        break;
    default:
        // Set multiplier.
        m = cursor / 4;

        if(button == BUTTON_1) {
            if(multipliers[m] < 3)
                ++multipliers[m];
        } else {
            if(multipliers[m] > 0)
                --multipliers[m];
        }

        break;
    }

    lcd_update();
}

static void
button_handle_hold(uint8_t button) {
    if(cursor == 0x40) {
        // Set framerate.
        if(button == BUTTON_1) {
            if(framerate <= 500)
                OCR1A = PERIOD(++framerate);
        } else {
            if(framerate > 1)
                OCR1A = PERIOD(--framerate);
        }
    }

    lcd_update();
}

static void
button_init(void) {
    BUTTON_DDR &= ~(BUTTON_1 | BUTTON_2 | BUTTON_3);
    BUTTON_PORT |= (BUTTON_1 | BUTTON_2 | BUTTON_3);
}

int
main(void) {
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

    restore_settings();

    lcd_init();
    button_init();
    camera_output_init();
    led_init();
    trigger_input_init();
    trigger_output_init();

    // Display welcome dialog for a while.
    _delay_ms(1000);

    sei();

    uint8_t button_jiffies[3] = {0, 0, 0};
    uint8_t button_held = 0b00000000;
    uint8_t button_state = 0b00000000;
    uint8_t lcd_jiffies = 0;

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

        if(!(lcd_jiffies++ % 64))
            lcd_update();
    }
}
