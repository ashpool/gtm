# General Temperature Machine

When incubating eggs and brewing beer, keeping a greenhouse cool, and countless other applications, it's desirable to maintain a constant temperature. A naive (and in many cases fully workable solution) is to use a thermostat, a device that turns on a heatsource or a cooler at a set temperature and turns it off when the temperature is correct. Using this method will cause the temperature to vary, or oscillate, to some degree, due to inertia in the system. I wanted to achive a near constant temperature, so I applied the PID algorithm.

> A proportional–integral–derivative controller (PID controller) is a control loop feedback mechanism (controller) commonly used in industrial control systems. A PID controller continuously calculates an error value as the difference between a desired setpoint and a measured process variable. The controller attempts to minimize the error over time by adjustment of a control variable, such as the position of a control valve, a damper, or the power supplied to a heating element, to a new value determined by a weighted sum..."
 ~[Wikipedia](https://en.wikipedia.org/wiki/PID_controller)

The PID in this case is not designed to output an analog value, since the relay can only be on or off, instead it outputs how long the heater or cooler should be on within a fixed interval. It's essentially a really slow version of pulse width modulation.

The hardware needed is a temperature sensor, a heat source or a cooler, and a controller. In my case I'm using:
* [DS18S20](https://arduino-info.wikispaces.com/Brick-Temperature-DS18B20) temperature sensor.
* [2 way relay module](https://arduino-info.wikispaces.com/RelayIsolation) (although 1 should work just fine).
* An Arduino as controller
