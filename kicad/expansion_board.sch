EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
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
LIBS:expansion_board-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Ultrasonic receiver expansion board"
Date "2018-07-11"
Rev "0.1"
Comp "https://github.com/araobp"
Comment1 ""
Comment2 "Reset switch and LEDs(blue/red)"
Comment3 ""
Comment4 "Knowles MEMS mic and character LCD"
$EndDescr
$Comp
L Conn_01x08 J1
U 1 1 5B4461AB
P 2850 3250
F 0 "J1" H 2850 3650 50  0000 C CNN
F 1 "Conn_01x08" H 2850 2750 50  0000 C CNN
F 2 "expansion_board:Pin Header 8P" H 2850 3250 50  0001 C CNN
F 3 "" H 2850 3250 50  0001 C CNN
	1    2850 3250
	-1   0    0    -1  
$EndComp
$Comp
L Conn_01x06 J2
U 1 1 5B4461E6
P 2850 4900
F 0 "J2" H 2850 5200 50  0000 C CNN
F 1 "Conn_01x06" H 2850 4500 50  0000 C CNN
F 2 "expansion_board:Pin Header 6P" H 2850 4900 50  0001 C CNN
F 3 "" H 2850 4900 50  0001 C CNN
	1    2850 4900
	-1   0    0    -1  
$EndComp
$Comp
L Conn_01x10 J8
U 1 1 5B446201
P 6900 3350
F 0 "J8" H 6900 3850 50  0000 C CNN
F 1 "Conn_01x10" H 6900 2750 50  0000 C CNN
F 2 "expansion_board:Pin Header 10P" H 6900 3350 50  0001 C CNN
F 3 "" H 6900 3350 50  0001 C CNN
	1    6900 3350
	1    0    0    1   
$EndComp
$Comp
L GND #PWR01
U 1 1 5B446400
P 4150 4900
F 0 "#PWR01" H 4150 4650 50  0001 C CNN
F 1 "GND" H 4150 4750 50  0000 C CNN
F 2 "" H 4150 4900 50  0001 C CNN
F 3 "" H 4150 4900 50  0001 C CNN
	1    4150 4900
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR02
U 1 1 5B446498
P 4000 2000
F 0 "#PWR02" H 4000 1850 50  0001 C CNN
F 1 "+5V" H 4000 2140 50  0000 C CNN
F 2 "" H 4000 2000 50  0001 C CNN
F 3 "" H 4000 2000 50  0001 C CNN
	1    4000 2000
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR03
U 1 1 5B4464C6
P 3750 2000
F 0 "#PWR03" H 3750 1850 50  0001 C CNN
F 1 "+3.3V" H 3750 2140 50  0000 C CNN
F 2 "" H 3750 2000 50  0001 C CNN
F 3 "" H 3750 2000 50  0001 C CNN
	1    3750 2000
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x02 J3
U 1 1 5B4464F0
P 4350 5900
F 0 "J3" H 4350 6000 50  0000 C CNN
F 1 "Conn_01x02" H 4350 5700 50  0000 C CNN
F 2 "expansion_board:Pin Header 2P" H 4350 5900 50  0001 C CNN
F 3 "" H 4350 5900 50  0001 C CNN
	1    4350 5900
	0    1    1    0   
$EndComp
$Comp
L Conn_01x04_Female J4
U 1 1 5B4499CF
P 4650 2900
F 0 "J4" H 4650 3100 50  0000 C CNN
F 1 "Conn_01x04_Female" H 4650 2600 50  0000 C CNN
F 2 "expansion_board:Pin Header 4P" H 4650 2900 50  0001 C CNN
F 3 "" H 4650 2900 50  0001 C CNN
	1    4650 2900
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x04_Female J6
U 1 1 5B449A02
P 5400 2900
F 0 "J6" H 5400 3100 50  0000 C CNN
F 1 "Conn_01x04_Female" H 5400 2600 50  0000 C CNN
F 2 "expansion_board:Pin Header 4P" H 5400 2900 50  0001 C CNN
F 3 "" H 5400 2900 50  0001 C CNN
	1    5400 2900
	-1   0    0    -1  
$EndComp
Text Label 4700 2950 0    60   ~ 0
L/R
Text Label 4700 2850 0    60   ~ 0
GND
Text Label 4700 3050 0    60   ~ 0
GND
Text Label 4700 3150 0    60   ~ 0
CLK
Text Label 5150 2850 0    60   ~ 0
VDD
Text Label 5150 2950 0    60   ~ 0
GND
Text Label 5150 3050 0    60   ~ 0
GND
Text Label 5150 3150 0    60   ~ 0
DAT
Text Label 4350 5750 0    60   ~ 0
CLK
$Comp
L LED D1
U 1 1 5B44A37E
P 5000 4400
F 0 "D1" H 5000 4500 50  0000 C CNN
F 1 "LED red" H 5000 4300 50  0000 C CNN
F 2 "expansion_board:LED" H 5000 4400 50  0001 C CNN
F 3 "" H 5000 4400 50  0001 C CNN
	1    5000 4400
	-1   0    0    1   
$EndComp
$Comp
L LED D2
U 1 1 5B44A3FD
P 5000 4050
F 0 "D2" H 5000 4150 50  0000 C CNN
F 1 "LED blue" H 5000 3950 50  0000 C CNN
F 2 "expansion_board:LED" H 5000 4050 50  0001 C CNN
F 3 "" H 5000 4050 50  0001 C CNN
	1    5000 4050
	-1   0    0    1   
$EndComp
$Comp
L R R1
U 1 1 5B44A445
P 5500 4400
F 0 "R1" V 5580 4400 50  0000 C CNN
F 1 "470" V 5500 4400 50  0000 C CNN
F 2 "expansion_board:Register" V 5430 4400 50  0001 C CNN
F 3 "" H 5500 4400 50  0001 C CNN
	1    5500 4400
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 5B44A4A3
P 5500 4050
F 0 "R2" V 5580 4050 50  0000 C CNN
F 1 "470" V 5500 4050 50  0000 C CNN
F 2 "expansion_board:Register" V 5430 4050 50  0001 C CNN
F 3 "" H 5500 4050 50  0001 C CNN
	1    5500 4050
	0    1    1    0   
$EndComp
$Comp
L Conn_01x04_Female J5
U 1 1 5B44A653
P 4950 1700
F 0 "J5" H 4950 1900 50  0000 C CNN
F 1 "Conn_01x04_Female" H 4950 1400 50  0000 C CNN
F 2 "expansion_board:Pin Header 4P" H 4950 1700 50  0001 C CNN
F 3 "" H 4950 1700 50  0001 C CNN
	1    4950 1700
	0    -1   -1   0   
$EndComp
$Comp
L SW_Push SW1
U 1 1 5B44A80B
P 3650 3950
F 0 "SW1" H 3700 4050 50  0000 L CNN
F 1 "SW_Push" H 3650 3890 50  0000 C CNN
F 2 "expansion_board:Tactile switch 4P" H 3650 4150 50  0001 C CNN
F 3 "" H 3650 4150 50  0001 C CNN
	1    3650 3950
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5B44A83A
P 3650 4250
F 0 "C1" H 3675 4350 50  0000 L CNN
F 1 "0.1pF" H 3675 4150 50  0000 L CNN
F 2 "expansion_board:Capacitor" H 3688 4100 50  0001 C CNN
F 3 "" H 3650 4250 50  0001 C CNN
	1    3650 4250
	0    1    1    0   
$EndComp
Wire Wire Line
	4150 3550 3050 3550
Wire Wire Line
	4150 2650 4150 2800
Wire Wire Line
	4150 2800 4150 3550
Wire Wire Line
	4150 3550 4150 4250
Wire Wire Line
	4150 4250 4150 4750
Wire Wire Line
	4150 4750 4150 4900
Wire Wire Line
	4000 3350 3050 3350
Wire Wire Line
	4000 2000 4000 2350
Wire Wire Line
	4000 2350 4000 2450
Wire Wire Line
	4000 2450 4000 3350
Wire Wire Line
	3750 2000 3750 2350
Wire Wire Line
	3750 2350 3750 2550
Wire Wire Line
	3750 2550 3750 2900
Wire Wire Line
	3750 2900 3750 3250
Wire Wire Line
	3050 3250 3750 3250
Wire Wire Line
	3750 3250 4550 3250
Wire Wire Line
	5600 3100 6200 3100
Connection ~ 4150 3550
Wire Wire Line
	4450 3100 4350 3100
Wire Wire Line
	4350 3100 4350 5700
Wire Wire Line
	4450 2900 3750 2900
Connection ~ 3750 2900
Wire Wire Line
	5600 2800 5800 2800
Wire Wire Line
	5800 2800 5800 2550
Wire Wire Line
	5800 2550 3750 2550
Connection ~ 3750 2550
Wire Wire Line
	5150 4400 5350 4400
Wire Wire Line
	5150 4050 5350 4050
Wire Wire Line
	5650 4050 5800 4050
Wire Wire Line
	5800 4050 5800 5050
Wire Wire Line
	5800 5050 6700 5050
Wire Wire Line
	5650 4400 5950 4400
Wire Wire Line
	5950 4400 5950 4950
Wire Wire Line
	5950 4950 6700 4950
Wire Wire Line
	4550 3250 4550 4050
Wire Wire Line
	4550 4050 4550 4400
Wire Wire Line
	4550 4400 4850 4400
Connection ~ 3750 3250
Wire Wire Line
	4550 4050 4850 4050
Wire Wire Line
	4850 1900 4850 2450
Wire Wire Line
	4850 2450 4000 2450
Connection ~ 4000 2450
Wire Wire Line
	5150 1900 5150 2650
Wire Wire Line
	5150 2650 4150 2650
Wire Wire Line
	4950 1900 4950 2450
Wire Wire Line
	4950 2450 6350 2450
Wire Wire Line
	6350 2450 6350 2850
Wire Wire Line
	6350 2850 6700 2850
Wire Wire Line
	5050 1900 5050 2350
Wire Wire Line
	5050 2350 6200 2350
Wire Wire Line
	6200 2350 6200 2950
Wire Wire Line
	6200 2950 6700 2950
NoConn ~ 5600 2900
NoConn ~ 5600 3000
Wire Wire Line
	4450 2800 4150 2800
Connection ~ 4150 2800
NoConn ~ 4450 3000
NoConn ~ 6700 3050
NoConn ~ 6700 3150
NoConn ~ 6700 3250
NoConn ~ 6700 3350
NoConn ~ 6700 3450
NoConn ~ 3050 2950
NoConn ~ 3050 3050
NoConn ~ 3050 3450
NoConn ~ 3050 3650
NoConn ~ 6700 4750
NoConn ~ 6700 4850
NoConn ~ 6700 5250
NoConn ~ 6700 5350
NoConn ~ 6700 5450
NoConn ~ 3050 5200
NoConn ~ 3050 5100
NoConn ~ 3050 5000
NoConn ~ 3050 4900
NoConn ~ 3050 4800
NoConn ~ 3050 4700
Wire Wire Line
	3800 4250 3950 4250
Wire Wire Line
	3950 4250 4150 4250
Connection ~ 4150 4250
Wire Wire Line
	3850 3950 3950 3950
Wire Wire Line
	3950 3950 3950 4250
Connection ~ 3950 4250
Wire Wire Line
	3200 3950 3450 3950
Wire Wire Line
	3050 3150 3200 3150
Wire Wire Line
	3200 3150 3200 3950
Wire Wire Line
	3200 3950 3200 4250
Wire Wire Line
	3200 4250 3500 4250
Connection ~ 3200 3950
$Comp
L PWR_FLAG #FLG04
U 1 1 5B44DCA7
P 3400 2150
F 0 "#FLG04" H 3400 2225 50  0001 C CNN
F 1 "PWR_FLAG" H 3400 2300 50  0000 C CNN
F 2 "" H 3400 2150 50  0001 C CNN
F 3 "" H 3400 2150 50  0001 C CNN
	1    3400 2150
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG05
U 1 1 5B44DCD7
P 4350 2150
F 0 "#FLG05" H 4350 2225 50  0001 C CNN
F 1 "PWR_FLAG" H 4350 2300 50  0000 C CNN
F 2 "" H 4350 2150 50  0001 C CNN
F 3 "" H 4350 2150 50  0001 C CNN
	1    4350 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 2150 3400 2350
Wire Wire Line
	3400 2350 3750 2350
Connection ~ 3750 2350
Wire Wire Line
	4350 2150 4350 2350
Wire Wire Line
	4350 2350 4000 2350
Connection ~ 4000 2350
Text Label 7000 2900 0    60   ~ 0
PB8/SCL1/D15
Text Label 7000 3000 0    60   ~ 0
PB9/SDA1/D14
$Comp
L Conn_01x08 J7
U 1 1 5B44E3D6
P 6900 5150
F 0 "J7" H 6900 5550 50  0000 C CNN
F 1 "Conn_01x08" H 6900 4650 50  0000 C CNN
F 2 "expansion_board:Pin Header 8P" H 6900 5150 50  0001 C CNN
F 3 "" H 6900 5150 50  0001 C CNN
	1    6900 5150
	1    0    0    1   
$EndComp
NoConn ~ 6700 3650
NoConn ~ 6700 3750
Text Label 7000 3600 0    60   ~ 0
PB6/D10/DFSDM1_DATAIN5
Text Label 7000 5100 0    60   ~ 0
PB5/GPIO/D5
Text Label 7000 5000 0    60   ~ 0
PB4/GPIO/D6
Text Label 2600 3400 0    60   ~ 0
5V
Text Label 2550 3300 0    60   ~ 0
3.3V
Text Label 2550 3600 0    60   ~ 0
GND
Text Label 2550 3200 0    60   ~ 0
NRST
Text Label 4900 1600 0    60   ~ 0
LCD
Text Label 4950 2800 0    60   ~ 0
MIC
$Comp
L PWR_FLAG #FLG06
U 1 1 5B44E863
P 3900 4700
F 0 "#FLG06" H 3900 4775 50  0001 C CNN
F 1 "PWR_FLAG" H 3900 4850 50  0000 C CNN
F 2 "" H 3900 4700 50  0001 C CNN
F 3 "" H 3900 4700 50  0001 C CNN
	1    3900 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 4700 3900 4750
Wire Wire Line
	3900 4750 4150 4750
Connection ~ 4150 4750
NoConn ~ 4250 5700
NoConn ~ 6700 5150
Wire Wire Line
	6200 3100 6200 3550
Wire Wire Line
	6200 3550 6700 3550
Text Label 3300 6300 0    60   ~ 0
To_PC2/DFSDM1_CLKOUT(Morpho_connector_CN7_pin#35)
Connection ~ 4550 4050
$EndSCHEMATC
