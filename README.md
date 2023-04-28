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

#### ADC
Current: HX711, a 24 bit ADC with a sample rate of 80 Hz  
Rev 2: HX717, similar to HX711, but can sample at 320 Hz.

#### Microcontroller
STM32F030C8T6: a 48 MHz, 32-bit ARM Cortex M0 microcontroller.

#### Strain Guage
BF350, more specifically 4 of them in a wheatstone bridge. Currently only using two in a half bridge seems to work, but using 4 may provide better sensitivity (and there a better signal to noise ratio).

## Software
A C program reads the ADC as fast as possible, performs some signal processing and generates a digital output that indicates the probe is "triggered" when there is suficient force acting on the nozzle.
