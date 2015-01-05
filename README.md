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
