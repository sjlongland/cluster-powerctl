Power controller for solar powered personal cloud
=================================================

This firmware is intended to control the charging of a battery bank that
powers a small computer cluster running a private cloud system.  The
idea is to try to keep the battery within a small range of voltages,
charging from solar or mains based chargers as necessary to top the
battery back up.

Circuit description
-------------------

An ATTiny24A microcontroller runs from a 5V rail derived from the
battery.  It uses its internal RC oscillator running at 1MHz.

Connected to PB0 and PB1 are the two MOSFETs that turn on the input
sources, one for solar (PB1), the other for a mains charger (PB0).

The source pins of these connect to the battery positive input, so when
the FET is on, power may flow from that source to the battery.

A third MOSFET connects to PB2, this is PWM-controlled to manage some
cooling fans.  The internal temperature sensor is used to decide whether
the fans should be on or off, and at what speed.

Connecting to each input, and to the battery, are separate voltage
dividers, comprising of a 1.5kOhm and 100Ohm resistors.  These divide
the input voltage by 16, and the divided voltage is fed into the ADC
pins PA0 (mains), PA1 (solar) and PA2 (battery).

LEDs connect to PA7, PA6, PA5, PA4 and PA3 to 0v.  ICSP is shared with the
LEDs.

GPIOs
-----

	PB0:	Mains MOSFET	(active HIGH)
	PB1:	Solar MOSFET	(active HIGH)
	PB2:	Fan MOSFET	(active HIGH)
	PB3:	nRESET
	PA7:	Temperature Low LED (active HIGH)
	PA6:	Temperature High LED (active HIGH) + ICSP MOSI
	PA5:	Battery Voltage High LED (active HIGH) + ICSP MISO
	PA4:	Warning LED (active high) + ICSP SCK + Debug Tx
	PA3:	Battery Voltage Good LED (active HIGH)
	PA2:	Analogue Input: Battery voltage
	PA1:	Analogue Input: Solar voltage
	PA0:	Analogue Input: Mains voltage

Firmware description
--------------------

The firmware may be compiled in DEBUG mode by adding -DDEBUG to the
CPPFLAGS.  In this mode, PA4 is used instead for a UART Tx pin at 1200
baud, 8 bits, no parity, one stop bit.  This is done in software, using
Timer 1 as a baud rate generator.

Timer 1 also does double-duty managing timings for events.

Timer 0 is used in PWM mode to control the fans.

The firmware does some quick initialisation before entering the main
loop.  If DEBUG is enabled, "INIT xx" will be displayed, where xx is the
value of the MCUSR register.

Two counters are decremented by the Timer 1 overflow interrupt.
`led_timeout` causes the main loop to update the state of the LEDs when
it hits zero, and `adc_timeout` triggers a new ADC capture run when it
reaches zero.

The three analogue inputs and temperature sensor are scanned when
`adc_timeout` reaches zero, then the state analysed.  The battery
voltage is compared to the previous reading to dissern if the battery is
charging, discharging or holding steady.

If this state has not changed, a battery state counter increments,
otherwise it is reset.

The current state of the FETs is checked.  Three states are valid:
- Idle state: all FETs off
- Mains charge: MAINS FET turned on
- Solar charge: SOLAR FET turned on

IF statements at this point compare the battery voltage to the
thresholds, and decide whether to switch voltage or not.

LED indications
---------------

The LEDs have the following meanings:

Warning LED (PA4; DEBUG undefined):
- Off:		No warning condition
- Flashing:	Battery below critical threshold or temperature above
  		maximum threshold
- On:		Not used

When in DEBUG mode, this LED may flicker with serial activity, and will
remain ON when idle.

Temperature LEDs (PA6, PA7):
- PA7 On, PA6 Off: Temperature below minimum threshold
- PA7 Flashing, PA6 Off: Temperature above minimum threshold
- PA7 Off, PA6 Flashing: Temperature above maximum threshold
- Other states: not used

Battery LEDs (PA5, PA3):
- PA3 Flashing, PA5 Off: Battery below low threshold
- PA3 Off, PA5 Flashing: Battery above high threshold
- PA3 On, PA5 Off: Battery is in "good" range (between low and high)
- Other states: not used
