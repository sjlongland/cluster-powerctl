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
#define V_CH_ADC	ADC_READ(V_CH_MV)
#define V_H_ADC		ADC_READ(V_H_MV)
#define V_L_ADC		ADC_READ(V_L_MV)
#define V_CL_ADC	ADC_READ(V_CL_MV)

/* --- Timeouts --- */
#define T_HF_TICKS	TIMER_TICKS(T_HF_MS)
#define T_LF_TICKS	TIMER_TICKS(T_LF_MS)
#define T_FAN_TICKS	TIMER_TICKS(T_FAN_MS)
#define T_LED_TICKS	TIMER_TICKS(T_LED_MS)
#define T_ADC_TICKS	TIMER_TICKS(T_ADC_MS)

#define STATE_DIS_CHECK	(0)	/*!< Check voltage in discharge state */
#define STATE_DIS_WAIT	(1)	/*!< Wait in discharge state */
#define STATE_CHG_CHECK	(2)	/*!< Check voltage in charging state */
#define STATE_CHG_WAIT	(3)	/*!< Wait in charging state */
/*!
 * Charger state machine state.  We have four states we can be in.
 */
static volatile uint8_t charger_state = STATE_DIS_CHECK;

#define SRC_NONE	(0)	/*!< Turn off all chargers */
#define SRC_SOLAR	(1)	/*!< Turn on solar charger */
#define SRC_MAINS	(2)	/*!< Turn on mains charger */
#define SRC_ALT		(3)	/*!< Alternate to *other* source,
				 * valid for select_src only.
				 */

/*!
 * Charging source.
 */
static volatile uint8_t charge_source = SRC_NONE;

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
static volatile uint16_t v_bl_adc = 0;

/*!
 * Current reading of the battery voltage in ADC units.
 */
static volatile uint16_t v_bn_adc = 0;

/*!
 * Current reading of the internal temperature sensor in ADC units.
 */
static volatile uint16_t temp_adc = 0;

/*!
 * How long before we next take a reading?
 */
static volatile uint16_t t_adc = 0;

/*!
 * How long before we change LED states?
 */
static volatile uint16_t t_led = 0;

/*! Fan kick-start timeout */
static volatile uint32_t t_fan = 0;

/*!
 * Charger timeout
 */
static volatile uint32_t t_charger = T_LF_TICKS;

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
 * Switch between chargers.  This is does a "break-before-make" switchover
 * of charging sources to switch from mains to solar, solar to mains, or to
 * switch from charging to discharging mode.  It expressly forbids turning
 * both chargers on simultaneously.
 *
 * Added is the ability to just alternate between sources.
 */
void select_src(uint8_t src) {
	if (src == SRC_ALT) {
		if (charge_source == SRC_SOLAR)
			src = SRC_MAINS;
		else
			src = SRC_SOLAR;
	}
#ifdef DEBUG
	uart_tx_str(STR_SELECT_SRC);
#endif
	switch(src) {
		case SRC_SOLAR:
			FET_PORT &= ~FET_MAINS;
			FET_PORT |= FET_SOLAR;
			charge_source = SRC_SOLAR;
#ifdef DEBUG
			uart_tx_str(STR_SRC_SOLAR);
#endif
			break;
		case SRC_MAINS:
			FET_PORT &= ~FET_SOLAR;
			FET_PORT |= FET_MAINS;
			charge_source = SRC_MAINS;
#ifdef DEBUG
			uart_tx_str(STR_SRC_MAINS);
#endif
			break;
		case SRC_NONE:
		default:
			FET_PORT &= ~FET_SRC_MASK;
			charge_source = SRC_NONE;
#ifdef DEBUG
			uart_tx_str(STR_SRC_NONE);
#endif
			break;
	}
#ifdef DEBUG
	uart_tx_str(STR_NL);
#endif
}

static void discharge_check() {
	/* Decide when we should do our next check */
#ifdef DEBUG
	uart_tx_str(STR_DIS); uart_tx_str(STR_CHK);
	uart_tx_bool(STR_V_BN_GE_V_H, v_bn_adc >= V_H_ADC);
#endif
	if (v_bn_adc >= V_H_ADC)
		t_charger = T_LF_TICKS;
	else
		t_charger = T_HF_TICKS;

	/* Snapshot the current battery voltage */
	v_bl_adc = v_bn_adc;

	/* Exit state */
#ifdef DEBUG
	uart_tx_bool(STR_V_BN_GT_V_L, v_bn_adc > V_L_ADC);
#endif
	if (v_bn_adc > V_L_ADC)
		charger_state = STATE_DIS_WAIT;
	else
		charger_state = STATE_CHG_CHECK;
}

static void discharge_wait() {
#ifdef DEBUG
	uart_tx_str(STR_DIS); uart_tx_str(STR_WAIT);
	uart_tx_bool(STR_V_BN_LE_V_CL, v_bn_adc <= V_CL_ADC);
#endif
	if (v_bn_adc <= V_CL_ADC)
		/* Expire timer */
		t_charger = 0;

	/* Exit state if timer is expired */
#ifdef DEBUG
	uart_tx_bool(STR_T_CHARGER, !t_charger);
#endif
	if (!t_charger)
		charger_state = STATE_DIS_CHECK;
}

static void charge_check() {
#ifdef DEBUG
	uart_tx_str(STR_CHG); uart_tx_str(STR_CHK);
	uart_tx_bool(STR_V_BN_LE_V_CL, v_bn_adc <= V_CL_ADC);
#endif
	/* Still need to charge, when should we next check? */
	if (v_bn_adc <= V_CL_ADC)
		t_charger = T_HF_TICKS;
	else
		t_charger = T_LF_TICKS;

#ifdef DEBUG
	uart_tx_bool(STR_HAVE_SOURCE, charge_source != SRC_NONE);
	uart_tx_bool(STR_V_BN_LE_V_BL, v_bn_adc <= v_bl_adc);
#endif
	if (charge_source == SRC_NONE) {
		/* Not yet charging, switch to primary source */
		select_src(SRC_SOLAR);
	} else if (v_bn_adc <= v_bl_adc) {
		/* Check for high voltage threshold, are we there yet? */
#ifdef DEBUG
		uart_tx_bool(STR_V_BN_GE_V_H, v_bn_adc >= V_H_ADC);
#endif
		if (v_bn_adc >= V_H_ADC) {
			/* We are done now */
			select_src(SRC_NONE);
			charger_state = STATE_DIS_CHECK;
			return;
		} else {
			/* Situation not improving, switch sources */
			select_src(SRC_ALT);
		}
	}

	v_bl_adc = v_bn_adc;
	charger_state = STATE_CHG_WAIT;
}

static void charge_wait() {
#ifdef DEBUG
	uart_tx_str(STR_CHG); uart_tx_str(STR_WAIT);
	uart_tx_bool(STR_V_BN_GE_V_CH, v_bn_adc >= V_CH_ADC);
#endif
	if (v_bn_adc >= V_CH_ADC)
		/* Expire timer */
		t_charger = 0;

#ifdef DEBUG
	uart_tx_bool(STR_T_CHARGER, !t_charger);
#endif
	if (!t_charger)
		charger_state = STATE_CHG_CHECK;
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
	FET_PORT = 0;

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
		if (!t_led) {
			if (v_bn_adc <= V_CL_ADC) {
				/* Battery is critically low */
				LED_PORT &= ~LED_BATT_HIGH;
				LED_PORT ^= LED_BATT_GOOD;
			} else if (v_bn_adc <= V_L_ADC) {
				/* Battery is low */
				LED_PORT &= ~(LED_BATT_HIGH|LED_BATT_GOOD);
			} else if (v_bn_adc <= V_H_ADC) {
				/* Battery is in "good" range */
				LED_PORT &= ~LED_BATT_HIGH;
				LED_PORT |= LED_BATT_GOOD;
			} else if (v_bn_adc <= V_CH_ADC) {
				/* Battery is above "high" threshold */
				LED_PORT |= LED_BATT_HIGH;
				LED_PORT &= ~LED_BATT_GOOD;
			} else {
				/* Battery is critically high */
				LED_PORT ^= LED_BATT_HIGH;
				LED_PORT &= ~LED_BATT_GOOD;
			}

			if (temp_adc < TEMP_MIN) {
				LED_PORT |= LED_TEMP_LOW;
				LED_PORT &= ~LED_TEMP_HIGH;
			} else if (temp_adc < TEMP_MAX) {
				LED_PORT ^= LED_TEMP_LOW;
				LED_PORT &= ~LED_TEMP_HIGH;
			} else {
				LED_PORT &= ~LED_TEMP_LOW;
				LED_PORT ^= LED_TEMP_HIGH;
			}
			t_led = T_LED_TICKS;
		}

		if (!t_adc) {
			t_adc = T_ADC_TICKS;
			ADCSRA |= (1 << ADEN) | (1 << ADSC);

			while(ADCSRA & (1 << ADEN));

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
					t_fan = T_FAN_TICKS;
				else if (pwm > FAN_PWM_MIN)
					OCR0A = pwm;
				else
					OCR0A = FAN_PWM_MIN;
			} else {
				/* Turn fans off completely. */
				OCR0A = 0;
			}

			/* Charger control */
			switch (charger_state) {
				case STATE_DIS_CHECK:
					discharge_check();
					break;
				case STATE_DIS_WAIT:
					discharge_wait();
					break;
				case STATE_CHG_CHECK:
					charge_check();
					break;
				case STATE_CHG_WAIT:
					charge_wait();
					break;
				default:
					charger_state = STATE_DIS_CHECK;
			}
		}
	}
	return 0;
}

ISR(TIM1_COMPA_vect) {
#ifdef DEBUG
	uart_tick();
#endif
	if (t_adc)
		t_adc--;
	if (t_led)
		t_led--;
	if (t_charger)
		t_charger--;
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
				v_bn_adc = adc;
#if 0
				/* Not being used for now */
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
#endif
			default:
				ADMUX = ADC_MUX_TEMP;
				ADCSRA &= ~(1 << ADEN);
		}
	} else {
		ADCSRA |= (1 << ADSC);
		last_admux = ADMUX;
	}
}
