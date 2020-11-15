# STC3117GenericDriver
STC3117 fuel gauge Open source generic driver
(STC3117 Generic Driver)

Driver type: LLD (Low level driver), platform independant

Driver version:
----------------
* This repository contains the generic driver (i.e. OS independant)   <br />
* __Linux driver__ available here:  <br />
https://github.com/st-sw/STC3117_LinuxDriver  <br />

Device under test:
----------------
Device:       STC3117 Battery Gas gauge  
Manufacturer: STMicroelectronics  

Typical Application:  Monitor a single cell battery <br />
Features: Measure the Voltage, Current, and Temperature. And then report the State of charge (%) <br />
Package:      CSP (9 pins) <br />
Operating supply: 2.7V to 4.5V  <br />

Hardware:
----------------
Can be used on any platform with an I2C Master (SCL & SDA pins) connected to the I2C Slave interface of STC3117 device.  <br />
For instance: STM32 Nucleo board, STM32 discovery board, Arduino, Raspberry Pi, Android dev kit, ...  <br />

The STC3117 is designed to be power supplied directly from the battery. In this case, the STC3117 remains active even if the whole platform is in standby or powered off.


SW Requirements:
----------------
Implement the I2C driver depending on your platform.

SW Configuration:
----------------
Update the configuration file depending on the battery characteristics (Capacity, Internal Impedance, battery default OCV curve, ...), and the schematic (Resistor value used for current sensing)

SW Use:
----------------
The host driver accesses the STC3117 registers via I2C every 5s typically (or longer, up to 30s).  <br />
So a 5s timer is required to be implemented.  <br />

The STC3117 monitors the battery continuously. <br />
But It is not needed for the host to access the STC3117 more often because the Battery charging/discharging variation is very slow.  <br />
However, even if the driver is accessed more frequently (every 1s), the STC3117 algorithm still works properly.  <br />

Battery State of Charge:
----------------
The STC3117 driver uses the ST OptimGauge(tm) algorithm to provide the Optimum accuracy regarding the estimation of the battery state of charge (in %).

Compatible battery:
----------------
Any single cell Lithium-Ion battery. <br />
For multi-cells battery, the hardware implementation needs to be slightly changed. <br />
Battery voltage max: Can monitor any battery from 2.7V to 4.5V.

Links:
----------------
Product page: &nbsp;&nbsp;&nbsp;  http://www.st.com/en/power-management/stc3117.html  
Blog : &nbsp;&nbsp;&nbsp; &nbsp;&nbsp;&nbsp;  http://blog.st.com/stc3117-battery-fuel-gauge-optimgauge/  

Notes:
----------------
Source code Examples are based on STC3115 drivers (the previous generation device).  

Issues:
----------------
For common issues, please refer to the FAQ first.

A short FAQ is available here: 
https://github.com/st-sw/STC3117GenericDriver/wiki/STC3117-FAQ
