Simple egg timer / break timer for the M24SR-DISCOVERY board:

http://www.st.com/st-web-ui/static/active/en/resource/technical/document/data_brief/DM00102423.pdf

Needs lots of cleanup.

This is a timer to remind me to take breaks. It cycles through 3 modes:

The "idle" mode, which is meant to be inobtrusive. It displays a slowly moving countdown until breaktime:

![](https://github.com/rogerdahl/m24sr-discovery-egg-timer/blob/master/images/idle.jpg)

The "get ready" mode, which is meant to tell me to get my code to a good place
before taking a break. In this mode, the time moves a bit faster and the LEDs
light up.

![](https://github.com/rogerdahl/m24sr-discovery-egg-timer/blob/master/images/move_soon.jpg)

The "annoying" mode, which is meant to tell me that I shouldn't be at the desk. In this mode, the
time bounces around, draing multicolored lines, and the LEDs are flashing.

![](https://github.com/rogerdahl/m24sr-discovery-egg-timer/blob/master/images/annoying.jpg)

The project compiles on Linux using the GCC open source toolchain and is flashed
to the board using Texane's stlink software: https://github.com/texane/stlink.

RTC interrupts are used for keeping track of the time. The joystick buttons can
be used for adjusting the time up and down.

The M24SR NFC chip is not in use.

This project is loosely based on the following project on GitHub:

https://github.com/tomvdb/stm32l1-discovery-basic-template

See the instructions there on how to set up an open source development
environment on Linux. When things are set up properly, build and flash with:

make burn
