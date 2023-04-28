# EnderStrain
A cheap strain gauge based bed probe for the Creality Ender 3 3D printer

## Goals
Support stock Ender 3 (or similar machines)
Use the nozzle touching the bed as the probing mechanism
Cost less than $10 per probe

## Hardware
A PCB is used in place the X carriage. In addition to acting as the X carriage, it houses the probing mechanim and electronics.
The PCB houses a strain gauge, which will flex when a force acts on the nozzle.
There is an ADC present to monitor the strain guage and a microcontroller to read the ADC and generate a digital signal.

## Software
A simple C script reads the ADC as fast as possible, performs some signal processing and generates a digital output that indicates the probe is "triggered" when there is suficient force acting on the nozzle.
