# efoil-datalogger
VESC based efoil data logger design by Mike Schulster
_____________________________________________________________________________________

DATA LOGGER DESIGN SPECS :
_________________________________________
Microcontroller : ESP32 Dev Module 38-pin

Storage : MicroSD module and 8GB card

GNSS : SAM-M10Q GNSS antenna module receiver from GNSS.store

Power : 5V from Vesc



COMMS ARCHITECTURE :
_______________________________
Throttle : PPM from VESC  

VESC telemetry : UART 1 (COMM port)

GNSS : ESP32 UART 2

SD card : SPI


WIRING:
_______________________________
VESC UART1 TX  →  ESP32 GPIO16

VESC UART1 RX  ←  ESP32 GPIO17

GPS TX         →  ESP32 GPIO16  [use UART2]

GPS RX         ←  ESP32 GPIO17

SD CS/MOSI/MISO/CLK → GPIO 15/23/19/18


OUTPUT FORMAT :
_______________________________________
Native VESC Tool CSV format

Exact header row from real VESC logs

One file per ride, named by GPS date/time

~5 Hz sample rate


KEY LOGGED CHANNELS :
___________________________________________
VESC: voltage, motor current, input current, ERPM, duty cycle, watt-hours, FET temp, motor temp (via NTC) GPS: lat, lon, ground speed, altitude, time



RIDE DETECTION:
___________________________________________________
Auto-start: GPS speed >1 m/s or ERPM above threshold

Auto-stop: 30s idle timeout

Clean file close on stop



ANTENNA INSTALLATION :
____________________________________________________
Mounted flat, ceramic face up,

Small copper/aluminium ground plane underneath

15–20 cm minimum from ESC and battery cables if possible

Cable routed away from power wiring


____________________________________________________________
EFOIL DATA LOGGER WIFI (can be changed in code lines 40 and 41) :
____________________________________________________________
SSID : eFoilLogger

Password : efoillogger


FUNCTIONALITY :
__________________________________________________________
The logger has auto start-stop functionality built in - 

AUTO START : GNSS speed > 1.0 m/s for 3 seconds OR VESC data is recent AND abs(ERPM) > 300 for 3 seconds (can be changed on lines 63-64)

AUTO STOP :  no recent VESC data OR abs(ERPM) < 200 AND abs(input current) < 1.0 A OR GNSS speed < 0.7 m/s for at least 54 of the last 60 one-second samples (can be changed on lines 65-71) 


Also the logging can be MANUALLY started and stopped from the web interface.

