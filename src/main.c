/* Includes ------------------------------------------------------------------*/
#include "settings.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>         /* itoa() function */
#include "twi.h"
#include "uart.h"
#include "lcd.h"

/* Constants and macros ------------------------------------------------------*/
#define UART_BAUD_RATE 9600
#define DHT12_SLAVE 0x5c
#define HUMIDITY_ADDRESS 0x00
#define TEMPERATURE_ADDRESS 0x02

#define TEMPERATURE_MAX 28
#define TEMPERATURE_MIN 20
#define HUMIDITY_MAX 10
#define HUMIDITY_MIN 23

/* Function prototypes -------------------------------------------------------*/
/**
 *  @brief Initialize lcd, TWI, and Timer/Counter1.
 */
void setup(void);
/**
 *  @brief TWI Finite State Machine
 */
void fsm_twi(void);
/**
 *  @brief Measure humidity with TWI
 */
void humidity_measure (void);
/**
 *  @brief Measure temperature with TWI
 */
void temperature_measure (void);
/**
 *  @brief Show by screen values of humidity and temperature
 */
void LCD_display (void);

/* Global variables ----------------------------------------------------------*/
typedef enum {
    IDLE_STATE = 1,
    HUMIDITY_STATE,
    TEMPERATURE_STATE,
    LCD_STATE,
    ALARM_STATE,
    ALARM_OFF_STATE
} state_t;
/* FSM for scanning TWI bus */
state_t twi_state = IDLE_STATE;

uint8_t twi_status;
char humidity_string;
char temperature_string;

/* Red button */
uint8_t button_press_alarm = 0;
/* Green button */
uint8_t button_press_on_off = 0;

struct Values {
    uint8_t humidity_integer;
    uint8_t humidity_decimal;
    uint8_t temperature_integer;
    uint8_t temperature_decimal;
};
/* Data structure for humidity and temperature values */
struct Values Meteo_values;

/* Functions -----------------------------------------------------------------*/
int main(void)
{
    /* Initializations */
    setup();

    /* Enables interrupts by setting the global interrupt mask */
    sei();

    /* Forever loop */
    while (1) {
        /* Cycle here, do nothing, and wait for an interrupt */
    }

    return 0;
}

/*******************************************************************************
 * Function: setup()
 * Purpose:  Initialize lcd, TWI, and Timer/Counter1.
 * Input:    None
 * Returns:  None
 ******************************************************************************/
void setup(void)
{
    /* Initialize lcd: asynchronous, 8-bit data, no parity, 1-bit stop */
    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));

    /* Initialize TWI */
    twi_init();

    /* Timer/Counter1: update FSM state */
    /* Clock prescaler 64 => overflows every 262 ms */
    TCCR1B |= _BV(CS11) | _BV(CS10);
    /* Overflow interrupt enable */
    TIMSK1 |= _BV(TOIE1);

    /*** GREEN BUTTON ***/
    /* Set input pin 2 (PD2) */
    DDRD &= ~_BV(PD2);
    /* Activate internal pull-up at pin 2 (PD2) */
    PORTD |= _BV(PD2);

    /* External Interrupt: enable External Interrupt by INT0 pin 2 (PD2) */
    /* Falling edge of INT0 generates an interrupt request */
    EICRA |= _BV(ISC01);
    /* Enable External Interrupt Request 0 */
    EIMSK |= _BV(INT0);

    /*** RED BUTTON ***/
    /* Set input pin 3 (PD3) */
    DDRD &= ~_BV(PD3);
    /* Activate internal pull-up at pin 2 (PD3) */
    PORTD |= _BV(PD3);

    /* External Interrupt: enable External Interrupt by INT0 pin 2 (PD2) */
    /* Falling edge of INT0 generates an interrupt request */
    EICRA |= _BV(ISC11);
    /* Enable External Interrupt Request 0 */
    EIMSK |= _BV(INT1);

    /*** LED ***/
    /* Set output pin 13 (PB5) */
    DDRB |= _BV(PB5);
    /* Turn LED off */
    PORTB &= ~_BV(PB5);

    /* Initialize display and select type of cursor */
    lcd_init(LCD_DISP_ON_CURSOR_BLINK);

    /* Clear display and set cursor to home position */
    lcd_clrscr();
}

/*******************************************************************************
 * Function:
 * Purpose:  Update state of TWI Finite State Machine.
 ******************************************************************************/
ISR(TIMER1_OVF_vect)
{
    fsm_twi();
}
/*******************************************************************************
 * Function:
 * Purpose:  Update values of button_press_on_off.
 ******************************************************************************/
ISR(INT0_vect)
{
    if (button_press_on_off == 0) {
        button_press_on_off = 1;
    } else {
        button_press_on_off = 0;
    }

}
/*******************************************************************************
 * Function:
 * Purpose:  Update values of button_press_alarm.
 ******************************************************************************/
ISR(INT1_vect)
{
    button_press_alarm = 1;

}
/*******************************************************************************
 * Function: fsm_twi()
 * Purpose:  TWI Finite State Machine.
 * Input:    None
 * Returns:  None
 ******************************************************************************/
void fsm_twi(void)
{
    switch (twi_state) {
        case IDLE_STATE:

            if (button_press_on_off == 1){
                twi_state = HUMIDITY_STATE;
            }
            else {
                lcd_clrscr();
                PORTB &= ~_BV(PB5);
                twi_state = IDLE_STATE;
            }

        break;

        case HUMIDITY_STATE:

            humidity_measure();
        break;

        case TEMPERATURE_STATE:

            temperature_measure();
        break;

        case LCD_STATE:

            LCD_display();
        break;

        case ALARM_STATE:

            if (button_press_on_off == 1){
                if ((Meteo_values.temperature_integer > TEMPERATURE_MIN && Meteo_values.temperature_integer < TEMPERATURE_MAX )|| (Meteo_values.humidity_integer > HUMIDITY_MIN && Meteo_values.humidity_integer < HUMIDITY_MAX )){

                  twi_state = HUMIDITY_STATE;
                }else {
                  /* LED ON */
                  PORTB |= _BV(PB5);
                  twi_state = ALARM_OFF_STATE;
                }
            } else {
                twi_state = IDLE_STATE;
            }
        break;

        case ALARM_OFF_STATE:

            if ( button_press_alarm == 1 )
            {
                /* LED OFF */
                PORTB &= ~_BV(PB5);
                button_press_alarm = 0;
                button_press_on_off = 0;
                twi_state = IDLE_STATE;
            } else {
                twi_state = ALARM_STATE;
            }
            break;

    default:
        twi_state = IDLE_STATE;
    } /* End of switch (twi_state) */

}
/*******************************************************************************
 * Function: humidity_measure()
 * Purpose:  Measure humidity with TWI
 * Input:    None
 * Returns:  None
 ******************************************************************************/
void humidity_measure (void){
    twi_status = twi_start((DHT12_SLAVE<<1) + TWI_WRITE);

    if (twi_status==0) {
        twi_write(HUMIDITY_ADDRESS);
        twi_stop();
        twi_start((DHT12_SLAVE<<1) + TWI_READ);
        Meteo_values.humidity_integer = twi_read_ack();
        Meteo_values.humidity_decimal = twi_read_nack();
        twi_stop();

        twi_state = TEMPERATURE_STATE;
    }
    else {
        twi_state = HUMIDITY_STATE;
    }

}
/*******************************************************************************
 * Function: temperature_measure()
 * Purpose:  Measure temperature with TWI
 * Input:    None
 * Returns:  None
 ******************************************************************************/
void temperature_measure (void){
    twi_status = twi_start((DHT12_SLAVE<<1) + TWI_WRITE);

    if (twi_status==0) {
        twi_write(TEMPERATURE_ADDRESS);
        twi_stop();
        twi_start((DHT12_SLAVE<<1) + TWI_READ);
        Meteo_values.temperature_integer = twi_read_ack();
        Meteo_values.temperature_decimal = twi_read_nack();
        twi_stop();

        twi_state = LCD_STATE;
    }
    else {
        twi_state = HUMIDITY_STATE;
    }

}
/*******************************************************************************
 * Function: LCD_display()
 * Purpose:  Show by screen values of humidity and temperature
 * Input:    None
 * Returns:  None
 ******************************************************************************/
void LCD_display (void){
    // Humidity
    lcd_gotoxy(0, 0);
    lcd_puts("humidity: ");
    itoa(Meteo_values.humidity_integer, humidity_string, 10);
    lcd_puts(humidity_string);
    lcd_puts(",");
    itoa(Meteo_values.humidity_decimal, humidity_string, 10);
    lcd_puts(humidity_string);
    lcd_puts("%");

    // Temperature
    lcd_gotoxy(0, 1);
    lcd_puts("temp.:    ");
    itoa(Meteo_values.temperature_integer, temperature_string, 10);
    lcd_puts(temperature_string);
    lcd_puts(",");
    itoa(Meteo_values.temperature_decimal, temperature_string, 10);
    lcd_puts(temperature_string);
    lcd_puts("C");

    twi_state = ALARM_STATE;
}

/* END OF FILE ****************************************************************/
