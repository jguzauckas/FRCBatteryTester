# FRC Battery Tester

This repository is the software side of an FRC Battery Tester I am building. The goal is to test batteries in a more realistic FRC application to determine the quality of a battery's capabilities.

The tester uses parallelized linear MOSFETs to sink a variable electric load. The tester itself is controlled by an Arduino Uno 4, which connects to a DAC -> OpAmps to control the MOSFETs, and reads the following information back in from ADSes:
- Each MOSFET's current load, via Kelvin voltage sensing off of a shunt resistor
- Each MOSFET's temperature via a thermistor mounted to the top of the MOSFET
- The total current load read off of the main battery line
- The current battery voltage via Kelvin voltage sensing off the positive input of the battery

The design of this project allows each MOSFET to consume 10 amps of power at 12 volts, and with a total array of 32 mosfets, allows loads up to 320 amps (at least momentarily). The MOSFETs are mounted in a grid to a large aluminum heatsink, which is actively cooled with fans.

The intention is to use a RoboRio 1 and create a simulated robot that communicates over serial with the Arduino. This would allow the roborio to ultimately control the loads that the Arduino facilitates. This setup allows us to monitor all the information in AdvantageScope, and log it using the AdvantageKit framework.

I am considering setting it up so that this tester could run through previous real match loads by reading information from a WPILog file.
