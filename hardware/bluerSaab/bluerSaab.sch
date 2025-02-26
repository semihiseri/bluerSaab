EESchema Schematic File Version 4
LIBS:bluerSaab-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L bluerSaab-rescue:ESP32-DEVKITC-bluerSaab U3
U 1 1 5D068673
P 2750 3550
F 0 "U3" H 2750 4575 50  0000 C CNN
F 1 "ESP32-DEVKITC" H 2750 4484 50  0000 C CNN
F 2 "bluerSaab:ESP32-DEVKITC" H 2750 4450 50  0001 C CNN
F 3 "" H 2750 4450 50  0001 C CNN
	1    2750 3550
	1    0    0    -1  
$EndComp
$Comp
L bluerSaab-rescue:LM2596-bluerSaab U1
U 1 1 5D068B16
P 3150 1250
F 0 "U1" H 3150 1675 50  0000 C CNN
F 1 "LM2596" H 3150 1584 50  0000 C CNN
F 2 "bluerSaab:LM2596" H 3150 1550 50  0001 C CNN
F 3 "" H 3150 1550 50  0001 C CNN
	1    3150 1250
	1    0    0    -1  
$EndComp
$Comp
L bluerSaab-rescue:MCP2515-bluerSaab U4
U 1 1 5D068DEA
P 6250 4700
F 0 "U4" H 6250 5225 50  0000 C CNN
F 1 "MCP2515" H 6250 5134 50  0000 C CNN
F 2 "bluerSaab:MCP2515" H 6250 5150 50  0001 C CNN
F 3 "" H 6250 5150 50  0001 C CNN
	1    6250 4700
	1    0    0    -1  
$EndComp
$Comp
L bluerSaab-rescue:PCM5102-bluerSaab U2
U 1 1 5D069228
P 6250 3050
F 0 "U2" H 6250 3775 50  0000 C CNN
F 1 "PCM5102" H 6250 3684 50  0000 C CNN
F 2 "bluerSaab:PCM5102" H 6250 3650 50  0001 C CNN
F 3 "" H 6250 3650 50  0001 C CNN
	1    6250 3050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x12 J1
U 1 1 5D069D8A
P 9800 3700
F 0 "J1" H 9880 3692 50  0000 L CNN
F 1 "Conn_01x12" H 9880 3601 50  0000 L CNN
F 2 "bluerSaab:SAAB_CDC" H 9800 3700 50  0001 C CNN
F 3 "~" H 9800 3700 50  0001 C CNN
	1    9800 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 1150 2650 1150
Wire Wire Line
	2650 1150 2650 1100
Wire Wire Line
	2650 1050 2750 1050
Wire Wire Line
	2750 1350 2650 1350
Wire Wire Line
	2650 1350 2650 1400
Wire Wire Line
	2650 1450 2750 1450
Wire Wire Line
	3550 1450 3650 1450
Wire Wire Line
	3650 1450 3650 1400
Wire Wire Line
	3650 1350 3550 1350
Wire Wire Line
	3550 1150 3650 1150
Wire Wire Line
	3650 1150 3650 1100
Wire Wire Line
	3650 1050 3550 1050
Wire Wire Line
	2650 1100 2550 1100
Connection ~ 2650 1100
Wire Wire Line
	2650 1100 2650 1050
Wire Wire Line
	2650 1400 2550 1400
Connection ~ 2650 1400
Wire Wire Line
	2650 1400 2650 1450
Text Label 1350 1100 0    50   ~ 0
CAR_GND
Text Label 1350 1400 0    50   ~ 0
CAR_12V
Text Label 4050 1100 2    50   ~ 0
GND
Text Label 4050 1400 2    50   ~ 0
5V
Connection ~ 3650 1400
Wire Wire Line
	3650 1400 3650 1350
Connection ~ 3650 1100
Wire Wire Line
	3650 1100 3650 1050
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5D06E557
P 2550 1100
F 0 "#FLG01" H 2550 1175 50  0001 C CNN
F 1 "PWR_FLAG" H 2550 1273 50  0000 C CNN
F 2 "" H 2550 1100 50  0001 C CNN
F 3 "~" H 2550 1100 50  0001 C CNN
	1    2550 1100
	1    0    0    -1  
$EndComp
Connection ~ 2550 1100
Wire Wire Line
	2550 1100 1350 1100
$Comp
L power:PWR_FLAG #FLG03
U 1 1 5D06EF9A
P 2550 1400
F 0 "#FLG03" H 2550 1475 50  0001 C CNN
F 1 "PWR_FLAG" H 2550 1573 50  0000 C CNN
F 2 "" H 2550 1400 50  0001 C CNN
F 3 "~" H 2550 1400 50  0001 C CNN
	1    2550 1400
	1    0    0    -1  
$EndComp
Text Label 5350 2550 0    50   ~ 0
5V
Text Label 5350 2750 0    50   ~ 0
GND
Text Label 5350 3150 0    50   ~ 0
BCK
Text Label 5350 3250 0    50   ~ 0
DIN
Text Label 5350 3350 0    50   ~ 0
LCK
Text Label 4900 3550 0    50   ~ 0
XMT
Wire Wire Line
	5850 2750 5750 2750
Wire Wire Line
	5750 2750 5750 2850
Wire Wire Line
	5750 2950 5850 2950
Wire Wire Line
	5850 2850 5750 2850
Connection ~ 5750 2850
Wire Wire Line
	5750 2850 5750 2950
Wire Wire Line
	5850 3450 5750 3450
Wire Wire Line
	5750 3450 5750 3050
Connection ~ 5750 2950
NoConn ~ 3250 2850
NoConn ~ 3250 2950
NoConn ~ 3250 3050
NoConn ~ 3250 3150
NoConn ~ 3250 3250
NoConn ~ 3250 3350
NoConn ~ 3250 3450
NoConn ~ 3250 3550
NoConn ~ 3250 3650
NoConn ~ 3250 3750
NoConn ~ 3250 3850
NoConn ~ 3250 3950
NoConn ~ 3250 4050
NoConn ~ 3250 4150
NoConn ~ 3250 4350
NoConn ~ 3250 4450
NoConn ~ 3250 4550
NoConn ~ 2250 2750
NoConn ~ 2250 2850
NoConn ~ 2250 2950
NoConn ~ 2250 3050
NoConn ~ 2250 3150
NoConn ~ 2250 3950
NoConn ~ 2250 4250
NoConn ~ 2250 4350
NoConn ~ 2250 4450
Wire Wire Line
	5350 2550 5850 2550
Wire Wire Line
	5750 2750 5350 2750
Connection ~ 5750 2750
Wire Wire Line
	5850 3050 5750 3050
Connection ~ 5750 3050
Wire Wire Line
	5750 3050 5750 2950
Wire Wire Line
	5350 3150 5850 3150
Wire Wire Line
	5350 3250 5850 3250
Wire Wire Line
	5350 3350 5850 3350
Wire Wire Line
	4900 3550 5100 3550
Text Label 7400 2950 2    50   ~ 0
AUDIO_RIGHT
Text Label 7400 3050 2    50   ~ 0
AUDIO_COMM
Text Label 7400 3150 2    50   ~ 0
AUDIO_LEFT
Wire Wire Line
	6650 2950 7400 2950
Wire Wire Line
	6650 3050 7400 3050
Wire Wire Line
	6650 3150 7400 3150
Text Label 1750 3550 0    50   ~ 0
LCK
Text Label 1750 3750 0    50   ~ 0
BCK
Text Label 1750 3650 0    50   ~ 0
DIN
Text Label 5350 4400 0    50   ~ 0
CAN_INT
Text Label 5350 4500 0    50   ~ 0
CAN_SCK
Text Label 5350 4600 0    50   ~ 0
CAN_MOSI
Text Label 5350 4700 0    50   ~ 0
CAN_MISO
Text Label 5350 4800 0    50   ~ 0
CAN_CS
Text Label 5350 4900 0    50   ~ 0
GND
Text Label 5350 5000 0    50   ~ 0
5V
Wire Wire Line
	5350 5000 5850 5000
Wire Wire Line
	5850 4900 5350 4900
Wire Wire Line
	5350 4800 5850 4800
Wire Wire Line
	5850 4700 5350 4700
Wire Wire Line
	5350 4600 5850 4600
Wire Wire Line
	5850 4500 5350 4500
Wire Wire Line
	5350 4400 5850 4400
Text Label 7000 4600 2    50   ~ 0
CAN_H
Text Label 7000 4700 2    50   ~ 0
CAN_L
Wire Wire Line
	7000 4700 6650 4700
Wire Wire Line
	6650 4600 7000 4600
Text Label 1750 4150 0    50   ~ 0
CAN_INT
Text Label 1750 4050 0    50   ~ 0
GND
Text Label 1750 4550 0    50   ~ 0
5V
Text Label 3650 2750 2    50   ~ 0
GND
Text Label 1750 3450 0    50   ~ 0
CAN_SCK
Text Label 1750 3350 0    50   ~ 0
CAN_MOSI
Text Label 1750 3250 0    50   ~ 0
CAN_MISO
Text Label 1750 3850 0    50   ~ 0
CAN_CS
Wire Wire Line
	3250 2750 3650 2750
Wire Wire Line
	1750 3250 2250 3250
Wire Wire Line
	1750 3350 2250 3350
Wire Wire Line
	1750 3450 2250 3450
Wire Wire Line
	1750 3550 2250 3550
Wire Wire Line
	1750 3650 2250 3650
Wire Wire Line
	1750 3750 2250 3750
Wire Wire Line
	1750 3850 2250 3850
Wire Wire Line
	1750 4050 2250 4050
Wire Wire Line
	1750 4150 2250 4150
Wire Wire Line
	1750 4550 2250 4550
NoConn ~ 9600 3200
NoConn ~ 9600 3500
NoConn ~ 9600 4000
NoConn ~ 9600 4100
Text Label 8900 3700 0    50   ~ 0
CAR_12V
Text Label 8900 4300 0    50   ~ 0
CAR_GND
Text Label 8900 4200 0    50   ~ 0
CAN_H
Text Label 8900 3600 0    50   ~ 0
CAN_L
Text Label 8900 3300 0    50   ~ 0
AUDIO_COMM
Text Label 8900 3800 0    50   ~ 0
AUDIO_RIGHT
Text Label 8900 3900 0    50   ~ 0
AUDIO_LEFT
Wire Wire Line
	8900 3300 9500 3300
Wire Wire Line
	9600 3400 9500 3400
Wire Wire Line
	9500 3400 9500 3300
Connection ~ 9500 3300
Wire Wire Line
	9500 3300 9600 3300
Wire Wire Line
	8900 3600 9600 3600
Wire Wire Line
	9600 3700 8900 3700
Wire Wire Line
	8900 3800 9600 3800
Wire Wire Line
	9600 3900 8900 3900
Wire Wire Line
	8900 4200 9600 4200
Wire Wire Line
	9600 4300 8900 4300
Wire Wire Line
	3650 1100 4050 1100
Wire Wire Line
	3650 1400 4050 1400
$Comp
L Jumper:SolderJumper_2_Open JP1
U 1 1 5D0D4612
P 5100 3050
F 0 "JP1" V 5050 2850 50  0000 L CNN
F 1 "SolderJumper_2_Open" V 5150 2200 50  0000 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_TrianglePad1.0x1.5mm" H 5100 3050 50  0001 C CNN
F 3 "~" H 5100 3050 50  0001 C CNN
	1    5100 3050
	0    1    1    0   
$EndComp
Wire Wire Line
	5850 2650 5200 2650
Wire Wire Line
	5100 2650 5100 2900
Wire Wire Line
	5100 3200 5100 3550
Connection ~ 5100 3550
Wire Wire Line
	5100 3550 5850 3550
Text Label 3600 4250 2    50   ~ 0
XMT
Wire Wire Line
	3600 4250 3250 4250
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5D0E4303
P 5200 2650
F 0 "#FLG0101" H 5200 2725 50  0001 C CNN
F 1 "PWR_FLAG" H 4950 2750 50  0000 C CNN
F 2 "" H 5200 2650 50  0001 C CNN
F 3 "~" H 5200 2650 50  0001 C CNN
	1    5200 2650
	1    0    0    -1  
$EndComp
Connection ~ 5200 2650
Wire Wire Line
	5200 2650 5100 2650
$Comp
L Device:Fuse F1
U 1 1 5D0E6F79
P 2050 1400
F 0 "F1" V 2150 1450 50  0000 L CNN
F 1 "Fuse" V 1950 1350 50  0000 L CNN
F 2 "Fuse_Holders_and_Fuses:Fuseholder_Fuse_TR5_Littlefuse-No560_No460" V 1980 1400 50  0001 C CNN
F 3 "~" H 2050 1400 50  0001 C CNN
	1    2050 1400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2550 1400 2200 1400
Connection ~ 2550 1400
Wire Wire Line
	1900 1400 1350 1400
$EndSCHEMATC
