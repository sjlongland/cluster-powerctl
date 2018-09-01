#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <avr/pgmspace.h>

#ifdef DEBUG
/*! If enabled, the warning LED doubles as UART Tx pin */
#include "uart.h"
#endif

#include "board.h"
#include "setpoints.h"

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

/* --- Thresholds --- */
#define V_H_ADC		ADC_READ(V_H_MV)
#define V_L_ADC		ADC_READ(V_L_MV)
#define V_SOL_MIN_ADC	ADC_READ(V_SOL_MIN_MV)

/* --- Timeouts --- */
#define T_ADC_TICKS	TIMER_TICKS(T_ADC_MS)

#define STATE_INIT	(0)	/*!< Initial start-up state */
#define STATE_SOLAR	(1)	/*!< Running from solar */
#define STATE_MAINS_CHG	(2)	/*!< Charging from mains */
#define STATE_MAINS_FLT	(3)	/*!< Floating on mains */

/*!
 * Charger state machine state.  We have four states we can be in.
 */
static volatile uint8_t charger_state = STATE_INIT;

/*!
 * For state machine, the last state of the ADC MUX so we know whether
 * to ignore the sample or not.  Datasheet recommends discarding samples
 * to let things stabalise when switching sources/references.
 */
static volatile uint8_t last_admux = 0;

/*!
 * Current reading of the battery voltage in ADC units.
 */
static volatile uint16_t v_bat_adc = 0;

/*!
 * Current reading of the solar voltage in ADC units.
 */
static volatile uint16_t v_sol_adc = 0;

/*!
 * Current reading of the internal temperature sensor in ADC units.
 */
static volatile uint16_t temp_adc = 0;

/*!
 * One-second event timer
 */
static volatile uint16_t t_second = 0;

/*!
 * How long before we next take a reading?
 */
static volatile uint16_t t_adc = 0;

/*!
 * Float timeout
 */
static volatile uint16_t t_float = 0;

/*!
 * Fan kick-start timeout
 */
static volatile uint8_t t_fan = 0;

/*!
 * ADC readings taken?
 */
static volatile uint8_t adc_checked = 0;

/* Debug messages */
#ifdef DEBUG
const char STR_INIT[] PROGMEM = {"INIT "};
const char STR_SELECT_SRC[] PROGMEM = {"SOURCE="};
const char STR_SRC_NONE[] PROGMEM = {"NONE"};
const char STR_SRC_SOLAR[] PROGMEM = {"SOLAR"};
const char STR_SRC_MAINS[] PROGMEM = {"MAINS"};
const char STR_DIS[] PROGMEM = {"\r\nDISCHARGE "};
const char STR_CHG[] PROGMEM = {"\r\nCHARGE "};
const char STR_CHK[] PROGMEM = {"CHECK\r\n"};
const char STR_WAIT[] PROGMEM = {"WAIT\r\n"};
const char STR_V_BN_GE_V_H[] PROGMEM = {"V_BN >= V_H? "};
const char STR_V_BN_GT_V_L[] PROGMEM = {"V_BN > V_L? "};
const char STR_V_BN_LE_V_CL[] PROGMEM = {"V_BN <= V_CL? "};
const char STR_V_BN_GE_V_CH[] PROGMEM = {"V_BN <= V_CH? "};
const char STR_V_BN_LE_V_BL[] PROGMEM = {"V_BN <= V_BL? "};
const char STR_HAVE_SOURCE[] PROGMEM = {"HAVE SOURCE? "};
const char STR_T_CHARGER[] PROGMEM = {"T_CHARGER EXPIRED? "};
const char STR_YES[] PROGMEM = {"YES\r\n"};
const char STR_NO[] PROGMEM = {"NO\r\n"};
const char STR_ADC[] PROGMEM = {"ADC "};
const char STR_START[] PROGMEM = {"START "};
const char STR_READ[] PROGMEM = {"READ "};
const char STR_NL[] PROGMEM = {"\r\n"};

static inline void uart_tx_bool(const char* msg, uint8_t val) {
	uart_tx_str(msg);
	if (val)
		uart_tx_str(STR_YES);
	else
		uart_tx_str(STR_NO);
}
#endif

/*!
 * Switch to charging from mains power.
 */
static void enter_mains_chg(void) {
	/* Enable mains power */
	FET_PORT &= ~FET_MAINS;

	/* Indicate via LEDs */
	LED_PORT |= LED_BATT_CHG;
	LED_PORT &= ~LED_BATT_FLT;

	/* Enter state */
	charger_state = STATE_MAINS_CHG;
}

/*!
 * Switch to floating on mains power.
 */
static void enter_mains_float(void) {
	/* Reset timer */
	t_float = T_FLOAT_S;

	/* Indicate via LEDs */
	LED_PORT &= ~LED_BATT_CHG;
	LED_PORT |= LED_BATT_FLT;

	/* Enter state */
	charger_state = STATE_MAINS_FLT;
}

/*!
 * Switch to running on solar.
 */
static void enter_solar(void) {
	/* Inhibit mains */
	FET_PORT |= FET_MAINS;

	/* Indicate via LEDs */
	LED_PORT &= ~(LED_BATT_FLT | LED_BATT_CHG);

	/* Enter state */
	charger_state = STATE_SOLAR;
}

/*!
 * Checks at start-up
 */
static void init_check(void) {
	/* Wait until we have our first readings from the ADC */
	if (!adc_checked)
		return;

	if ((v_bat_adc < V_L_ADC) || (v_sol_adc < V_SOL_MIN_ADC))
		/* Battery/solar is low, begin charging */
		enter_mains_chg();
	else
		/* Run from solar */
		enter_solar();
}

/*!
 * Checks whilst running on solar
 */
static void solar_check(void) {
	if (v_bat_adc < V_L_ADC) {
		/* Move to mains power */
		enter_mains_chg();
		return;
	}
}

/*!
 * Checks whilst charging from mains
 */
static void mains_chg_check(void) {
	if (v_bat_adc >= V_H_ADC) {
		/* We've reached the float voltage */
		enter_mains_float();
		return;
	}
}

/*!
 * Checks whilst floating on mains
 */
static void mains_float_check(void) {
	if (v_bat_adc < V_H_ADC) {
		/* We've regressed, go back to charging state! */
		enter_mains_chg();
		return;
	} else if ((!t_float) && (v_sol_adc >= V_SOL_MIN_ADC)) {
		/* Solar can take it from here */
		enter_solar();
	}
}

/*!
 * Main entrypoint
 */
int main(void) {
	/* Configure LEDs */
	LED_PORT_DDR_REG = LED_PORT_DDR_VAL;
	LED_PORT = 0;

	/* Configure MOSFETs */
	FET_PORT_DDR_REG = FET_PORT_DDR_VAL;
	FET_PORT = FET_MAINS | FET_SOLAR;

	/* Turn on ADC and timers */
	PRR &= ~((1 << PRTIM0) | (1 << PRTIM1) | (1 << PRADC));

	/* Configure Timer0: Fan PWM */
	TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << CS00);
	OCR0A = 0;

	/*
	 * Configure Timer1: TIMER_FREQ System tick timer
	 * / baud rate generator for debug output
	 */
	TCCR1A = 0;
	TCCR1B = (1 << WGM12) | (1 << CS10);
	TCCR1C = 0;
	OCR1A = F_CPU/TIMER_FREQ;
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
		/* One second passed, tick down the 1-second timers. */
		if (!t_second) {
			t_second = TIMER_FREQ;
			if (t_float)
				t_float--;
		}

		if (!t_adc) {
			t_adc = T_ADC_TICKS;
			ADCSRA |= (1 << ADEN) | (1 << ADSC);

			while(ADCSRA & (1 << ADEN));

			/* Temperature LED control */
			if (temp_adc < TEMP_MIN) {
				LED_PORT |= LED_TEMP_LOW;
				LED_PORT &= ~LED_TEMP_HIGH;
			} else if (temp_adc < TEMP_MAX) {
				LED_PORT |= (LED_TEMP_LOW | LED_TEMP_HIGH);
			} else {
				LED_PORT &= ~LED_TEMP_LOW;
				LED_PORT |= LED_TEMP_HIGH;
			}

			/*
			 * The "SOLAR" FET is no longer fitted, so this is more
			 * an indication of whether we consider solar to be
			 * "good enough".  In short, it's just controlling the
			 * LED where the MOSFET was now.
			 */
			if (v_sol_adc < V_SOL_MIN_ADC)
				FET_PORT |= FET_SOLAR;
			else
				FET_PORT &= ~FET_SOLAR;

			/* Fan control */
			if (t_fan) {
				/* Kick-start mode */
				OCR0A = FAN_PWM_MAX;
			} else if (temp_adc > TEMP_MAX) {
				/* We're at the maximum temperature, FULL SPEED! */
				OCR0A = FAN_PWM_MAX;
			} else if (temp_adc > TEMP_MIN) {
				/* Scale fan speed linearly with temperature */
				uint8_t pwm = (((temp_adc - TEMP_MIN)
							* FAN_PWM_MAX)
						/ (TEMP_MAX - TEMP_MIN));
				if (OCR0A < FAN_PWM_MIN)
					/* Enter kick-start mode */
					t_fan = T_FAN_S;
				else if (pwm > FAN_PWM_MIN)
					OCR0A = pwm;
				else
					OCR0A = FAN_PWM_MIN;
			} else {
				/* Turn fans off completely. */
				OCR0A = 0;
			}

			/* Battery state LED control */
			if (v_bat_adc <= V_L_ADC) {
				LED_PORT &= ~LED_BATT_GOOD;
			} else {
				LED_PORT |= LED_BATT_GOOD;
			}

			/* Charger control */
			switch (charger_state) {
				case STATE_INIT:
					init_check();
					break;
				case STATE_SOLAR:
					solar_check();
					break;
				case STATE_MAINS_CHG:
					mains_chg_check();
					break;
				case STATE_MAINS_FLT:
					mains_float_check();
					break;
				default:
					charger_state = STATE_INIT;
			}
		}
	}
	return 0;
}

ISR(TIM1_COMPA_vect) {
#ifdef DEBUG
	uart_tick();
#endif
	/* One-second timer for longer events */
	if (t_second)
		t_second--;

	if (t_adc)
		t_adc--;
}

ISR(ADC_vect) {
	uint16_t adc = ADCW;
	if (last_admux == ADMUX) {
		switch(last_admux) {
			case ADC_MUX_TEMP:
				temp_adc = adc;
				ADMUX = ADC_MUX_BATT;
				ADCSRA |= (1 << ADSC);
				break;
			case ADC_MUX_BATT:
				v_bat_adc = adc;
				ADMUX = ADC_MUX_SOLAR;
				ADCSRA |= (1 << ADSC);
				break;
			case ADC_MUX_SOLAR:
				v_sol_adc = adc;
				/* Once we get here, we've done a full cycle */
				adc_checked = 1;
			default:
				ADMUX = ADC_MUX_TEMP;
				ADCSRA &= ~(1 << ADEN);
		}
	} else {
		ADCSRA |= (1 << ADSC);
		last_admux = ADMUX;
	}
}
