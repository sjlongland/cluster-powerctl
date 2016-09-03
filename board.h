#ifndef _BOARD_H
#define _BOARD_H

/* LEDs */
#define LED_TEMP_LOW		(1 << 7)
#define LED_TEMP_HIGH		(1 << 6)
#define LED_BATT_HIGH		(1 << 5)
#define LED_WARNING		(1 << 4)
#define LED_BATT_GOOD		(1 << 3)
#define LED_PORT		PORTA
#define LED_PORT_DDR_REG	DDRA
#define LED_PORT_DDR_VAL	( LED_TEMP_LOW \
				| LED_TEMP_HIGH \
				| LED_BATT_HIGH \
				| LED_WARNING \
				| LED_BATT_GOOD )

/* MOSFETs */
#define FET_MAINS		(1 << 0)
#define FET_SOLAR		(1 << 1)
#define FET_FAN			(1 << 2)
#define FET_SRC_MASK		(FET_MAINS|FET_SOLAR)
#define FET_PORT		PORTB
#define FET_PORT_DDR_REG	DDRB
#define FET_PORT_DDR_VAL	( FET_MAINS \
				| FET_SOLAR \
				| FET_FAN )

/* ADC channels */
#define ADC_CH_MAINS		(1 << 0)
#define ADC_CH_SOLAR		(1 << 1)
#define ADC_CH_BATT		(1 << 2)
#define ADC_CH_EN		( ADC_CH_MAINS \
				| ADC_CH_SOLAR \
				| ADC_CH_BATT )

#endif
