# STC3117GenericDriver
STC3117 fuel gauge Open source generic driver
(STC3117 Generic Driver)

Device under test:
----------------
Device:       STC3117 Battery Gas gauge  
Manufacturer: STMicroelectronics  
Silicon release: July 2014

Hardware:
----------------
Can be used on any platform with I2C Master (pin SCL & SDA) connected to I2C Slave of STC3117 device.  <br />
For instance: STM32 Nucleo board, STM32 discovery board, Arduino, ...  <br />

The STC3117 is designed to be power supplied directly from the battery. In this case, the STC3117 remains active even if the whole platform is in standby or powered off.


SW Requirements:
----------------
Implement the I2C driver depending on your platform.

SW Configuration:
----------------
Update the configuration file depending on the battery characteristics (Capacity, Internal Impedance, battery default OCV curve, ...), and the schematic (Resistor value used for current sensing)

SW Use:
----------------
The host driver access the STC3117 registers via I2C every 5s typically (or longer, up to 30s).  <br />
So a 5s timer is required to be implemented.  <br />

The STC3117 monitors the battery continuously. <br />
But It is not needed for the host to access the STC3117 more often because the Battery charging/discharging variation is very slow.  <br />
However, even if the driver is access more frequently (every 1s), the STC3117 algorithm still works properly.  <br />

Battery State of Charge:
----------------
The STC3117 driver use the ST OptimGauge(tm) algorithm to give the Optimum accuracy regarding the estimation of the battery state of charge (in %).

Compatible battery:
----------------
Any single cell Lithium-Ion battery .
For multi-cells battery, the hardware implementation needs to be slightly changed.
Battery voltage max: Can monitor any battery from 2.7V to 5V.

Notes:
----------------
Source code Examples are based on STC3115 drivers (the previous generation device).  

Issues:
----------------
For any issues, please refer to the FAQ first.

A short FAQ is available here: 
https://github.com/st-sw/STC3117GenericDriver/wiki/STC3117-FAQ
