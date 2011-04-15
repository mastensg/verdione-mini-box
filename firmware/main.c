#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/twi.h>

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

static volatile uint8_t multipliers[4] = {0, 0, 0, 0};
static volatile uint8_t time = 0;
static volatile uint8_t cameras = 0b10101010;
static volatile uint16_t framerate = 16;
static volatile uint8_t cursor = 0x40;

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
    /*
    static uint8_t col = 0;

    if(c == '\n') {
        for(; col < 40; ++col)
            twi_send_byte(' ');

        col = 0;
        return;
    }

    if(col == 16) {
        for(; col < 40; ++col)
            twi_send_byte(' ');

        col = 0;
    }
    */

    twi_send_byte(c);
    //++col;
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

    lcd_message("Mini trigger box\n   Simula  RL   ");
}

static void
lcd_update(void) {
    lcd_command(LCD_CLEAR_SCREEN);

    for(uint8_t i = 0; i < 4; ++i) {
        lcd_command(LCD_CURSOR_SET);
        lcd_putchar(4 * i);
        lcd_putdigit(1 << multipliers[i]);
        lcd_putchar('x');
        lcd_putchar('D');
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
            if(framerate > 2)
                OCR1A = PERIOD(--framerate);
        }
        break;
    case 0x40 + 8:
        // Store settings.
        eeprom_busy_wait();
        eeprom_write_byte((uint8_t *)FRAMERATEADDR, framerate);
        lcd_message("Saved!");
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
            if(framerate > 2)
                OCR1A = PERIOD(--framerate);
        }
    }

    lcd_update();
}

int
main(void) {
    uint8_t button_jiffies[3] = {0, 0, 0};
    uint8_t button_held = 0b00000000;
    uint8_t button_state = 0b00000000;

    lcd_init();

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

    lcd_update();

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
