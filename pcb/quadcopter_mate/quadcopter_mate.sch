EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:quadcopter_mate-cache
EELAYER 25 0
EELAYER END
$Descr User 7087 4724
encoding utf-8
Sheet 1 1
Title "UAV Payload Interface (Quad Side)"
Date "Thu 11 Jun 2015"
Rev "C"
Comp "Nolan Reker, Drew Troxell, David Troy"
Comment1 "Senior Project"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_02X05 HEADER1
U 1 1 55581036
P 5300 2150
F 0 "HEADER1" H 5300 2450 50  0000 C CNN
F 1 "CONN_02X05" H 5300 1850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_2x05" H 5300 950 60  0001 C CNN
F 3 "" H 5300 950 60  0000 C CNN
	1    5300 2150
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X05 POGO2
U 1 1 55581191
P 1400 2450
F 0 "POGO2" H 1400 2750 50  0000 C CNN
F 1 "CONN_02X05" H 1400 2150 50  0000 C CNN
F 2 "quadcopter_footprints:Pin_Header_Straight_2x05_SMD" H 1400 1250 60  0001 C CNN
F 3 "" H 1400 1250 60  0000 C CNN
	1    1400 2450
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X05 POGO1
U 1 1 555811EF
P 5300 1300
F 0 "POGO1" H 5300 1600 50  0000 C CNN
F 1 "CONN_02X05" H 5300 1000 50  0000 C CNN
F 2 "quadcopter_footprints:Pin_Header_Straight_2x05_SMD" H 5300 100 60  0001 C CNN
F 3 "" H 5300 100 60  0000 C CNN
	1    5300 1300
	1    0    0    -1  
$EndComp
$Comp
L RJ45 J1
U 1 1 55581526
P 1350 1200
F 0 "J1" H 1550 1700 60  0000 C CNN
F 1 "RJ45" H 1200 1700 60  0000 C CNN
F 2 "quadcopter_footprints:RJ45" H 1350 1200 60  0001 C CNN
F 3 "" H 1350 1200 60  0000 C CNN
	1    1350 1200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 5558122F
P 1000 2700
F 0 "#PWR01" H 1000 2450 50  0001 C CNN
F 1 "GND" H 1000 2550 50  0000 C CNN
F 2 "" H 1000 2700 60  0000 C CNN
F 3 "" H 1000 2700 60  0000 C CNN
	1    1000 2700
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR02
U 1 1 55581EA0
P 1800 2700
F 0 "#PWR02" H 1800 2550 50  0001 C CNN
F 1 "+5V" H 1800 2840 50  0000 C CNN
F 2 "" H 1800 2700 60  0000 C CNN
F 3 "" H 1800 2700 60  0000 C CNN
	1    1800 2700
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X01 MAG3
U 1 1 555833C6
P 3500 1350
F 0 "MAG3" H 3500 1450 50  0000 C CNN
F 1 "CONN_01X01" V 3600 1350 50  0001 C CNN
F 2 "quadcopter_footprints:8mmMagnet" H 3500 1350 60  0001 C CNN
F 3 "" H 3500 1350 60  0000 C CNN
	1    3500 1350
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 MAG4
U 1 1 55583589
P 3500 1900
F 0 "MAG4" H 3500 2000 50  0000 C CNN
F 1 "CONN_01X01" V 3600 1900 50  0001 C CNN
F 2 "quadcopter_footprints:8mmMagnet" H 3500 1900 60  0001 C CNN
F 3 "" H 3500 1900 60  0000 C CNN
	1    3500 1900
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 MAG2
U 1 1 555835E3
P 2600 1900
F 0 "MAG2" H 2600 2000 50  0000 C CNN
F 1 "CONN_01X01" V 2700 1900 50  0001 C CNN
F 2 "quadcopter_footprints:8mmMagnet" H 2600 1900 60  0001 C CNN
F 3 "" H 2600 1900 60  0000 C CNN
	1    2600 1900
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X01 MAG1
U 1 1 5558361D
P 2600 1350
F 0 "MAG1" H 2600 1450 50  0000 C CNN
F 1 "CONN_01X01" V 2700 1350 50  0001 C CNN
F 2 "quadcopter_footprints:8mmMagnet" H 2600 1350 60  0001 C CNN
F 3 "" H 2600 1350 60  0000 C CNN
	1    2600 1350
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR03
U 1 1 5558364B
P 3050 2000
F 0 "#PWR03" H 3050 1750 50  0001 C CNN
F 1 "GND" H 3050 1850 50  0000 C CNN
F 2 "" H 3050 2000 60  0000 C CNN
F 3 "" H 3050 2000 60  0000 C CNN
	1    3050 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 1100 6100 1100
Wire Wire Line
	6100 1100 6100 1950
Wire Wire Line
	6100 1950 5550 1950
Wire Wire Line
	5550 1200 6000 1200
Wire Wire Line
	6000 1200 6000 2050
Wire Wire Line
	6000 2050 5550 2050
Wire Wire Line
	5550 1300 5900 1300
Wire Wire Line
	5900 1300 5900 2150
Wire Wire Line
	5900 2150 5550 2150
Wire Wire Line
	5550 1400 5750 1400
Wire Wire Line
	5750 1400 5750 2250
Wire Wire Line
	5750 2250 5550 2250
Wire Wire Line
	5550 1500 5650 1500
Wire Wire Line
	5650 1500 5650 2350
Wire Wire Line
	5650 2350 5550 2350
Wire Wire Line
	5050 1100 4500 1100
Wire Wire Line
	4500 1100 4500 1950
Wire Wire Line
	4500 1950 5050 1950
Wire Wire Line
	5050 1200 4650 1200
Wire Wire Line
	4650 1200 4650 2050
Wire Wire Line
	4650 2050 5050 2050
Wire Wire Line
	5050 1300 4750 1300
Wire Wire Line
	4750 1300 4750 2150
Wire Wire Line
	4750 2150 5050 2150
Wire Wire Line
	5050 1400 4850 1400
Wire Wire Line
	4850 1400 4850 2250
Wire Wire Line
	4850 2250 5050 2250
Wire Wire Line
	5050 1500 4950 1500
Wire Wire Line
	4950 1500 4950 2350
Wire Wire Line
	4950 2350 5050 2350
Wire Wire Line
	3050 1350 3050 2000
Connection ~ 3050 1350
Connection ~ 3050 1900
$Comp
L GND #PWR04
U 1 1 555849FD
P 6300 1200
F 0 "#PWR04" H 6300 950 50  0001 C CNN
F 1 "GND" H 6300 1050 50  0000 C CNN
F 2 "" H 6300 1200 60  0000 C CNN
F 3 "" H 6300 1200 60  0000 C CNN
	1    6300 1200
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR05
U 1 1 55585945
P 4250 1250
F 0 "#PWR05" H 4250 1100 50  0001 C CNN
F 1 "+5V" H 4250 1390 50  0000 C CNN
F 2 "" H 4250 1250 60  0000 C CNN
F 3 "" H 4250 1250 60  0000 C CNN
	1    4250 1250
	-1   0    0    1   
$EndComp
Wire Wire Line
	1150 2650 1000 2650
Wire Wire Line
	1000 2650 1000 2700
Wire Wire Line
	1650 2650 1800 2650
Wire Wire Line
	1800 2650 1800 2700
Wire Wire Line
	1100 1650 1100 1900
Wire Wire Line
	1100 1900 1850 1900
Wire Wire Line
	1850 1900 1850 2550
Wire Wire Line
	1850 2550 1650 2550
Wire Wire Line
	1300 1650 1300 1850
Wire Wire Line
	1300 1850 1900 1850
Wire Wire Line
	1900 1850 1900 2450
Wire Wire Line
	1900 2450 1650 2450
Wire Wire Line
	1500 1650 1500 2000
Wire Wire Line
	1500 2000 1800 2000
Wire Wire Line
	1800 2000 1800 2350
Wire Wire Line
	1800 2350 1650 2350
Wire Wire Line
	1700 1650 1700 2250
Wire Wire Line
	1700 2250 1650 2250
Wire Wire Line
	1000 1650 1000 2550
Wire Wire Line
	1000 2550 1150 2550
Wire Wire Line
	1200 1650 1200 2000
Wire Wire Line
	1200 2000 900  2000
Wire Wire Line
	900  2000 900  2450
Wire Wire Line
	900  2450 1150 2450
Wire Wire Line
	1400 1650 1400 2050
Wire Wire Line
	1400 2050 1100 2050
Wire Wire Line
	1100 2050 1100 2350
Wire Wire Line
	1100 2350 1150 2350
Wire Wire Line
	1600 1650 1600 2100
Wire Wire Line
	1600 2100 1150 2100
Wire Wire Line
	1150 2100 1150 2250
Wire Wire Line
	6300 1200 6100 1200
Connection ~ 6100 1200
Wire Wire Line
	4250 1250 4250 1200
Wire Wire Line
	4250 1200 4500 1200
Connection ~ 4500 1200
Wire Wire Line
	2800 1900 3300 1900
Wire Wire Line
	2800 1350 3300 1350
$Comp
L GND #PWR?
U 1 1 55587CCE
P 2050 1000
F 0 "#PWR?" H 2050 750 50  0001 C CNN
F 1 "GND" H 2050 850 50  0000 C CNN
F 2 "" H 2050 1000 60  0000 C CNN
F 3 "" H 2050 1000 60  0000 C CNN
	1    2050 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 850  2050 850 
Wire Wire Line
	2050 850  2050 1000
$EndSCHEMATC
