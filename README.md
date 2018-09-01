Power controller for solar powered personal cloud
=================================================

This firmware is intended to control the charging of a battery bank that powers
a small computer cluster running a private cloud system.  The system has two
charging sources: mains power and solar power.

The chargers are assumed to have a logic-level signal which, when pulled low,
shuts down the relevant charger.

Circuit description
-------------------

An ATTiny24A microcontroller runs from a 5V rail derived from the
battery.  It uses its internal RC oscillator running at 1MHz.

Connected to PB0 and PB1 are the two transistors that turn on the input
sources, one for solar (PB1), the other for a mains charger (PB0).  These are
NPN bi-polar junction transistors.  When the base is turned on via PB0 or PB1,
this allows current to flow from the collector to ground, effectively pulling
the relevant logic level output low.

Connected to PB2 is a third NPN transistor which is wired to a P-channel
MOSFET, such that turning the transistor on also turns on that MOSFET.  This
is PWM-controlled to manage some cooling fans.  The internal temperature
sensor is used to decide whether the fans should be on or off, and at what
speed.

Connecting to each input, and to the battery, are separate voltage
dividers, comprising of a 1.5kOhm and 100Ohm resistors.  These divide
the input voltage by 16, and the divided voltage is fed into the ADC
pins PA1 (solar) and PA2 (battery).

LEDs connect to PA7, PA6, PA5, PA4 and PA3 to 0v.  ICSP is shared with the
LEDs.

GPIOs
-----

	PB0:	Mains enable	(active LOW)
	PB1:	Solar enable	(active LOW)
	PB2:	Fan MOSFET	(active HIGH)
	PB3:	nRESET
	PA7:	Temperature Low LED (active HIGH)
	PA6:	Temperature High LED (active HIGH) + ICSP MOSI
	PA5:	Mains Floating LED (active HIGH) + ICSP MISO
	PA4:	Mains Charging LED (active high) + ICSP SCK + Debug Tx
	PA3:	Battery Voltage Good LED (active HIGH)
	PA2:	Analogue Input: Battery voltage
	PA1:	Analogue Input: Solar voltage

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
`t_second` causes the main loop to decrement each one-second timer, `t_adc`
causes the ADC state machine to advance one tick and checkt the state of the
channels.

The two analogue inputs and temperature sensor are scanned when
`t_adc` reaches zero, then the state analysed.

The charger is in one of four states:
* initialisation
* solar
* charging from mains
* floating on mains

On power up, we enter the initialisation state.  In this state, we wait for
the first ADC readings to arrive before deciding on whether we run from mains
power or solar.  During this time, all power inputs are inhibited.

If either the battery or solar input are below minimum thresholds, the mains
charger is turned on and we enter the "charging from mains" state.
Otherwise the solar input is used and we enter the "solar" state.

In the "solar" state, we monitor the battery voltage.  If it drops below the
minimum voltage, we switch to the "charging from mains" state.

In the "charging from mains" state, we monitor the battery charging progress.
Upon reaching the high voltage threshold, we switch to the "floating on mains"
state.

In the "floating on mains" state, we wait a minimum of one hour for the
mains charger to finish its cycle.  If the battery drops below the
high-voltage threshold, we move back to the "charging from mains" state.

Once an hour has elapsed in the floating state, if the solar input is above
the minimum threshold, we turn off the mains charger and switch to the "solar"
state.

LED indications
---------------

The LEDs have the following meanings:

Temperature LEDs (PA6, PA7):
- PA7 Off, PA6 Off: Temperature below minimum threshold
- PA7 On, PA6 Off: Temperature between thresholds
- PA7 Off, PA6 On: Temperature above maximum threshold

Battery State LED (PA3):
- PA3 Off: Battery is below minimum voltage
- PA3 On: Battery is above minimum voltage

Mains Charger State LEDs (PA4, PA5):
- PA4 Off, PA5 Off: Mains supply is off
- PA4 On, PA5 Off: Mains supply is charging the battery
- PA4 Off, PA5 On: Mains supply is floating the battery

Note that in debug mode, PA4 instead becomes the serial TX line, and so will
remain on in the idle state, and will flicker with serial activity.
