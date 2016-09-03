#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <avr/pgmspace.h>

#ifdef DEBUG
/*! If enabled, the warning LED doubles as UART Tx pin */
#include "uart.h"
#endif

#include "board.h"

/*! ADMUX setting for selecting 1.1V reference */
#define ADC_REF_1V1	(2 << REFS0)
/*! ADMUX setting for mains input voltage reading */
#define ADC_MUX_MAINS	(ADC_REF_1V1 | 0x00)
/*! ADMUX setting for solar input voltage reading */
#define ADC_MUX_SOLAR	(ADC_REF_1V1 | 0x01)
/*! ADMUX setting for battery input voltage reading */
#define ADC_MUX_BATT	(ADC_REF_1V1 | 0x02)
/*! ADMUX setting for temperature reading */
#define ADC_MUX_TEMP	(ADC_REF_1V1 | 0x22)

/*!
 * For state machine, the last state of the ADC MUX so we know whether
 * to ignore the sample or not.  Datasheet recommends discarding samples
 * to let things stabalise when switching sources/references.
 */
static volatile uint8_t last_admux = 0;

/*!
 * For state machine, determines what the battery was one sample ago so
 * we know if it's charging, discharging, or remaining static.  ADC units.
 */
static volatile uint16_t last_adc_batt = 0;

/*!
 * Current reading of the battery voltage in ADC units.
 */
static volatile uint16_t adc_batt = 0;

/*!
 * Current reading of the solar charger voltage in ADC units.
 */
static volatile uint16_t adc_solar = 0;

/*!
 * Current reading of the mains charger voltage in ADC units.
 */
static volatile uint16_t adc_mains = 0;

/*!
 * Current reading of the internal temperature sensor in ADC units.
 */
static volatile uint16_t adc_temp = 0;

/*!
 * How long before we next take a reading?
 */
static volatile uint16_t adc_timeout = 0;
/*!
 * How many Timer1 ticks between ADC readings?
 */
#define ADC_TIMEOUT (1200)

/*!
 * State of the battery.
 * -1:	discharging
 *  0:	remaining steady
 *  1:	charging
 */
static volatile int8_t batt_state = 0;
/*!
 * The state of the battery at last check.
 */
static volatile int8_t last_batt_state = 0;
/*!
 * The number of readings that the battery has maintained this state.
 */
static volatile uint8_t batt_state_counter = 0;
/*!
 * How long before we can consider switching sources.
 */
static volatile uint8_t src_timeout = 0;
#define SRC_TIMEOUT	(15)	/*!< How long to wait before switching */

/*!
 * How long before we change LED states?
 */
static volatile uint8_t led_timeout = 0;
#define LED_TIMEOUT	(150)

/*
 * Temperature ranges and fan PWM settings
 */
#define TEMP_MIN	(270 << 6)	/*!< ~20°C, approx ADC reading */
#define TEMP_MAX	(300 << 6)	/*!< ~30°C, approx ADC reading */
#define FAN_PWM_MIN	(80)		/*!< Minimum PWM value */
#define FAN_PWM_MAX	(255)		/*!< Maximum PWM value */
/*! Fan kick-start timeout */
static volatile uint8_t fan_timeout = 0;
#define FAN_TIMEOUT	(5)

/*
 * ADC Voltage divider settings
 */
#define VDIV_R1		(1500ULL)	/*!< R1 = 1.5kOhm */
#define VDIV_R2		(100ULL)	/*!< R2 = 100 Ohm */
/*
 * ADC settings
 */
#define ADC_REF		(1100ULL)	/*!< AREF = 1.1mV */
#define ADC_MAX		(65535ULL)	/*!< ADLAR = 1 */

/*!
 * Macro for computing ADC measurements.  This assumes the input to the
 * ADC pin is via a voltage divider made up of resistors R1 and R2, with
 * the input voltage applied across both resistors and the ADC measuring
 * across R2.
 *
 * @param	mv	Voltage in millivolts
 * @returns		Approximate ADC reading
 */
# define ADC_READ(mv) (						\
	(ADC_MAX * ((uint64_t)(mv)) * VDIV_R2)			\
	 /							\
	(ADC_REF * (VDIV_R1 + VDIV_R2))				\
)

/*!
 * "Critical" battery voltage.  This is considered a serious condition.
 */
#define VBATT_CRIT	ADC_READ(11800)
/*!
 * "Low" battery voltage.  Indication that we should turn a charger on.
 */
#define VBATT_LOW	ADC_READ(12000)
/*!
 * "High" battery voltage.  Indication we should turn the charger off.
 */
#define VBATT_HIGH	ADC_READ(13500)

/* Debug messages */
#ifdef DEBUG
const char STR_INIT[] PROGMEM = {"INIT "};
const char STR_ADC[] PROGMEM = {"ADC "};
const char STR_START[] PROGMEM = {"START "};
const char STR_READ[] PROGMEM = {"READ "};
const char STR_NL[] PROGMEM = {"\r\n"};
#endif

#define SRC_NONE	(0)	/*!< Turn off all chargers */
#define SRC_SOLAR	(1)	/*!< Turn on solar charger */
#define SRC_MAINS	(2)	/*!< Turn on mains charger */
/*!
 * Switch between chargers.  This is does a "break-before-make" switchover
 * of charging sources to switch from mains to solar, solar to mains, or to
 * switch from charging to discharging mode.  It expressly forbids turning
 * both chargers on simultaneously.
 */
void select_src(uint8_t src) {
	switch(src) {
		case SRC_SOLAR:
			FET_PORT &= ~FET_MAINS;
			FET_PORT |= FET_SOLAR;
			break;
		case SRC_MAINS:
			FET_PORT &= ~FET_SOLAR;
			FET_PORT |= FET_MAINS;
			break;
		case SRC_NONE:
		default:
			FET_PORT &= ~FET_SRC_MASK;
			break;
	}
	src_timeout = SRC_TIMEOUT;
}

/*!
 * Main entrypoint */
int main(void) {
	/* Configure LEDs */
	LED_PORT_DDR_REG = LED_PORT_DDR_VAL;
	LED_PORT = 0;

	/* Configure MOSFETs */
	FET_PORT_DDR_REG = FET_PORT_DDR_VAL;
	FET_PORT = 0;

	/* Turn on ADC and timers */
	PRR &= ~((1 << PRTIM0) | (1 << PRTIM1) | (1 << PRADC));

	/* Configure Timer0: Fan PWM */
	TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << CS00);
	OCR0A = 0;

	/*
	 * Configure Timer1: 1.2kHz System tick timer
	 * / baud rate generator for debug output
	 */
	TCCR1A = 0;
	TCCR1B = (1 << WGM12) | (1 << CS10);
	TCCR1C = 0;
	OCR1A = F_CPU/1200;
	TIMSK1 = (1 << OCIE1A);

	/* ADC configuration */
	DIDR0 = ADC_CH_EN;
	ADMUX = ADC_MUX_TEMP;
	ADCSRB = (1 << ADLAR);
	ADCSRA = (1 << ADIE)
		| (1 << ADPS2)
		| (1 << ADPS1)
		| (1 << ADPS0);

	/* Configure UART */
	sei();
#ifdef DEBUG
	uart_init();
	uart_tx_str(STR_INIT);
	uart_tx_hex_byte(MCUSR);
	uart_tx_str(STR_NL);
#endif
	MCUSR = 0;
	while(1) {
		if (!led_timeout) {
#ifndef DEBUG
			if ((adc_batt < VBATT_CRIT)
					|| (adc_temp > TEMP_MAX)) {
				/* Warning conditions */
				LED_PORT ^= LED_WARNING;
			} else {
				LED_PORT &= ~LED_WARNING;
			}
#endif

			if (adc_batt < VBATT_LOW) {
				/* Battery is low */
				LED_PORT &= ~LED_BATT_HIGH;
				LED_PORT ^= LED_BATT_GOOD;
			} else if (adc_batt >= VBATT_HIGH) {
				/* Battery is above "high" threshold */
				LED_PORT ^= LED_BATT_HIGH;
				LED_PORT &= ~LED_BATT_GOOD;
			} else {
				/* Battery is above "low" threshold */
				LED_PORT |= LED_BATT_GOOD;
				LED_PORT &= ~LED_BATT_HIGH;
			}

			if (adc_temp < TEMP_MIN) {
				LED_PORT |= LED_TEMP_LOW;
				LED_PORT &= ~LED_TEMP_HIGH;
			} else if (adc_temp < TEMP_MAX) {
				LED_PORT ^= LED_TEMP_LOW;
				LED_PORT &= ~LED_TEMP_HIGH;
			} else {
				LED_PORT &= ~LED_TEMP_LOW;
				LED_PORT ^= LED_TEMP_HIGH;
			}
			led_timeout = LED_TIMEOUT;
		}
		if (!adc_timeout) {
			adc_timeout = ADC_TIMEOUT;
			ADCSRA |= (1 << ADEN) | (1 << ADSC);
#ifdef DEBUG
			uart_tx_str(STR_ADC);
			uart_tx_str(STR_READ);
#endif
			while(ADCSRA & (1 << ADEN));
#ifdef DEBUG
			uart_tx_str(STR_NL);
			uart_tx_str(STR_ADC);
			uart_tx('T'); uart_tx_hex_word(adc_temp);
			uart_tx(' ');
			uart_tx('B'); uart_tx_hex_word(adc_batt);
			uart_tx(' ');
			uart_tx('L'); uart_tx_hex_word(last_adc_batt);
			uart_tx(' ');
			uart_tx('d');
			if (last_adc_batt > adc_batt) {
				uart_tx('-');
				uart_tx_hex_word(last_adc_batt - adc_batt);
			} else {
				uart_tx('+');
				uart_tx_hex_word(adc_batt - last_adc_batt);
			}
			uart_tx(' ');
			uart_tx('S'); uart_tx_hex_word(adc_solar);
			uart_tx(' ');
			uart_tx('M'); uart_tx_hex_word(adc_mains);
			uart_tx(' ');
			uart_tx('F'); uart_tx_hex_byte(FET_PORT);
#endif

			/* Battery direction */
#ifdef DEBUG
			uart_tx(' ');
			uart_tx('b');
#endif
			if ((!last_adc_batt) || (adc_batt == last_adc_batt)) {
				batt_state = 0;	/* Steady? */
#ifdef DEBUG
				uart_tx('=');
#endif
			} else if (adc_batt > last_adc_batt) {
				batt_state = 1;
#ifdef DEBUG
				uart_tx('+');
#endif
			} else if (adc_batt < last_adc_batt) {
				batt_state = -1;
#ifdef DEBUG
				uart_tx('-');
#endif
			}

			if (last_batt_state == batt_state) {
				batt_state_counter++;
			} else {
				batt_state_counter = 0;
				last_batt_state = batt_state;
			}

			/* Battery control */
			uint8_t state = FET_PORT & FET_SRC_MASK;
			switch (state) {
				case 0:
					/* Idle state */
#ifdef DEBUG
					uart_tx('I');
#endif
					if ((adc_batt < VBATT_CRIT)
							&& (adc_mains > adc_batt)) {
						/* Charger urgently needed. */
#ifdef DEBUG
						uart_tx('C');
						uart_tx('M');
#endif
						select_src(SRC_MAINS);
					} else if (adc_batt < VBATT_LOW) {
						/* Charger needed. */
#ifdef DEBUG
						uart_tx('L');
#endif
						if ((adc_solar >= adc_mains)
								&& (adc_solar > adc_batt)) {
#ifdef DEBUG
							uart_tx('S');
#endif
							select_src(SRC_SOLAR);
						} else {
#ifdef DEBUG
							uart_tx('M');
#endif
							select_src(SRC_MAINS);
						}
					}
					break;
				case FET_SOLAR:
#ifdef DEBUG
					uart_tx('S');
#endif
					/* Are we over voltage? */
					if (adc_batt >= VBATT_HIGH) {
#ifdef DEBUG
						uart_tx('H');
#endif
						select_src(SRC_NONE);
					} else if ((adc_batt < VBATT_CRIT)
							&& (adc_mains > adc_solar)
							&& (adc_mains > adc_batt)) {
#ifdef DEBUG
						uart_tx('C');
#endif
						select_src(SRC_MAINS);
						/* Are we still discharging? */
					} else if ((!src_timeout)
							&& (adc_mains > adc_batt)
							&& (batt_state <= 0)
							&& (batt_state_counter > 10)) {
#ifdef DEBUG
						uart_tx('M');
#endif
						select_src(SRC_MAINS);
					} else if (src_timeout) {
#ifdef DEBUG
						uart_tx('s');
						uart_tx_hex_byte(batt_state_counter);
						uart_tx('t');
						uart_tx_hex_byte(src_timeout);
#endif
						src_timeout--;
					}
					break;
				case FET_MAINS:
#ifdef DEBUG
					uart_tx('M');
#endif
					/* Are we over voltage? */
					if (adc_batt >= VBATT_HIGH) {
#ifdef DEBUG
						uart_tx('H');
#endif
						select_src(SRC_NONE);
						/* Are we still critical? */
					} else if (adc_batt < VBATT_CRIT) {
#ifdef DEBUG
						uart_tx('C');
#endif
						if (adc_mains < adc_solar) {
							/* Mains no good, try solar */
#ifdef DEBUG
							uart_tx('S');
#endif
							select_src(SRC_SOLAR);
						}
						/* Is solar better now? */
					} else if ((!src_timeout)
							&& (adc_solar > adc_mains)) {
#ifdef DEBUG
						uart_tx('S');
#endif
						select_src(SRC_SOLAR);
					} else if (src_timeout) {
#ifdef DEBUG
						uart_tx('t');
						uart_tx_hex_byte(src_timeout);
#endif
						src_timeout--;
					}
					break;
				default:
					/* Should not get here */
#ifdef DEBUG
					uart_tx('!');
#endif
					select_src(SRC_NONE);
			}

			/* Fan control */
			if (fan_timeout) {
				/* Kick-start mode */
				OCR0A = FAN_PWM_MAX;
				fan_timeout--;
			} else if (adc_temp > TEMP_MAX) {
				/* We're at the maximum temperature, FULL SPEED! */
				OCR0A = FAN_PWM_MAX;
			} else if (adc_temp > TEMP_MIN) {
				/* Scale fan speed linearly with temperature */
				uint8_t pwm = (((adc_temp - TEMP_MIN)
							* FAN_PWM_MAX)
						/ (TEMP_MAX - TEMP_MIN));
				if (OCR0A < FAN_PWM_MIN)
					/* Enter kick-start mode */
					fan_timeout = FAN_TIMEOUT;
				else if (pwm > FAN_PWM_MIN)
					OCR0A = pwm;
				else
					OCR0A = FAN_PWM_MIN;
			} else {
				/* Turn fans off completely. */
				OCR0A = 0;
			}
#ifdef DEBUG
			uart_tx(' ');
			uart_tx('f');
			uart_tx_hex_byte(OCR0A);

			uart_tx_str(STR_NL);
#endif
		}
	}
	return 0;
}

ISR(TIM1_COMPA_vect) {
#ifdef DEBUG
	uart_tick();
#endif
	if (adc_timeout)
		adc_timeout--;
	if (led_timeout)
		led_timeout--;
}

ISR(ADC_vect) {
	uint16_t adc = ADCW;
	if (last_admux == ADMUX) {
		switch(last_admux) {
			case ADC_MUX_TEMP:
				adc_temp = adc;
				ADMUX = ADC_MUX_BATT;
				ADCSRA |= (1 << ADSC);
				break;
			case ADC_MUX_BATT:
				last_adc_batt = adc_batt;
				adc_batt = adc;
				ADMUX = ADC_MUX_SOLAR;
				ADCSRA |= (1 << ADSC);
				break;
			case ADC_MUX_SOLAR:
				adc_solar = adc;
				ADMUX = ADC_MUX_MAINS;
				ADCSRA |= (1 << ADSC);
				break;
			case ADC_MUX_MAINS:
				adc_mains = adc;
			default:
				ADMUX = ADC_MUX_TEMP;
				ADCSRA &= ~(1 << ADEN);
		}
	} else {
		ADCSRA |= (1 << ADSC);
		last_admux = ADMUX;
	}
}
