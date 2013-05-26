Semar Mesem
===========

Line Tracker Robot with PID control system.

## spec

* ATMega16 with 4MHz clock
* L298 for motor driver (connected to PORTD.1 - PORTD.6)
* Ten Phototransistors connected to LM339 comparator for sensing the line,
  connected to PINA and PINB ( 5 and 6 ).
* LCD 2x16 connected to PORTC.
* Four tactile switches connected to PINB.0 - PINB.3 (used for menu navigation).
* Schematic can be found [here](https://github.com/gedex/semar-mesem-robot/raw/master/schematic.gif)

## Screenshot

<img src="https://github.com/gedex/semar-mesem-robot/raw/master/screenshot.jpg">

More photos [here](https://plus.google.com/photos/112369140878480353945/albums/5220254395056825857?banner=pwa).

## How it works

Semar Mesem uses 8 sensors infront to track the line. Ideal line's width is 1.5 - 2 cm
with possibility 2 - 3 sensors reading the line.
