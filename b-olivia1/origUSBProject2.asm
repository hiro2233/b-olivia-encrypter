
; PICBASIC PRO(TM) Compiler 2.60, (c) 1998, 2009 microEngineering Labs, Inc. All Rights Reserved. 
_USED			EQU	1

	INCLUDE	"C:\PBP\18F2550.INC"


; Define statements.
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00010	DEFINE OSC 48
#define		OSC		 48
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00011	DEFINE RESET_ORG 1000h
#define		RESET_ORG		 1000h
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00029	DEFINE DEBUG_REG PORTA 
#define		DEBUG_REG		 PORTA 
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00032	DEFINE DEBUG_BIT 0 
#define		DEBUG_BIT		 0 
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00035	DEFINE DEBUG_BAUD 9600
#define		DEBUG_BAUD		 9600
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00038	DEFINE DEBUG_MODE 1
#define		DEBUG_MODE		 1

RAM_START       		EQU	00000h
RAM_END         		EQU	007FFh
RAM_BANKS       		EQU	00008h
BANK0_START     		EQU	00060h
BANK0_END       		EQU	000FFh
BANK1_START     		EQU	00100h
BANK1_END       		EQU	001FFh
BANK2_START     		EQU	00200h
BANK2_END       		EQU	002FFh
BANK3_START     		EQU	00300h
BANK3_END       		EQU	003FFh
BANK4_START     		EQU	00400h
BANK4_END       		EQU	004FFh
BANK5_START     		EQU	00500h
BANK5_END       		EQU	005FFh
BANK6_START     		EQU	00600h
BANK6_END       		EQU	006FFh
BANK7_START     		EQU	00700h
BANK7_END       		EQU	007FFh
BANKA_START     		EQU	00000h
BANKA_END       		EQU	0005Fh

; C:\PBP\PBPPIC18.RAM      	00028	A00000	FLAGS   VAR     BYTE BANKA SYSTEM       ' Static flags
FLAGS           		EQU	RAM_START + 000h
; C:\PBP\PBPPIC18.RAM      	00012	A00001	R0      VAR     WORD BANKA SYSTEM       ' System Register
R0              		EQU	RAM_START + 001h
; C:\PBP\PBPPIC18.RAM      	00013	A00003	R1      VAR     WORD BANKA SYSTEM       ' System Register
R1              		EQU	RAM_START + 003h
; C:\PBP\PBPPIC18.RAM      	00014	A00005	R2      VAR     WORD BANKA SYSTEM       ' System Register
R2              		EQU	RAM_START + 005h
; C:\PBP\PBPPIC18.RAM      	00015	A00007	R3      VAR     WORD BANKA SYSTEM       ' System Register
R3              		EQU	RAM_START + 007h
; C:\PBP\PBPPIC18.RAM      	00016	A00009	R4      VAR     WORD BANKA SYSTEM       ' System Register
R4              		EQU	RAM_START + 009h
; C:\PBP\PBPPIC18.RAM      	00017	A0000B	R5      VAR     WORD BANKA SYSTEM       ' System Register
R5              		EQU	RAM_START + 00Bh
; C:\PBP\PBPPIC18.RAM      	00018	A0000D	R6      VAR     WORD BANKA SYSTEM       ' System Register
R6              		EQU	RAM_START + 00Dh
; C:\PBP\PBPPIC18.RAM      	00019	A0000F	R7      VAR     WORD BANKA SYSTEM       ' System Register
R7              		EQU	RAM_START + 00Fh
; C:\PBP\PBPPIC18.RAM      	00020	A00011	R8      VAR     WORD BANKA SYSTEM       ' System Register
R8              		EQU	RAM_START + 011h
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00001	A00013	' ************************************************************
T1              		EQU	RAM_START + 013h
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00001	A00015	' ************************************************************
T2              		EQU	RAM_START + 015h
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00001	A00017	' ************************************************************
T3              		EQU	RAM_START + 017h
; C:\PBP\PBPPIC18.RAM      	00027	A00019	GOP     VAR     BYTE BANKA SYSTEM       ' Gen Op Parameter
GOP             		EQU	RAM_START + 019h
; C:\PBP\PBPPIC18.RAM      	00023	A0001A	RM1     VAR     BYTE BANKA SYSTEM       ' Pin 1 Mask
RM1             		EQU	RAM_START + 01Ah
; C:\PBP\PBPPIC18.RAM      	00026	A0001B	RM2     VAR     BYTE BANKA SYSTEM       ' Pin 2 Mask
RM2             		EQU	RAM_START + 01Bh
; C:\PBP\PBPPIC18.RAM      	00021	A0001C	RR1     VAR     BYTE BANKA SYSTEM       ' Pin 1 Register
RR1             		EQU	RAM_START + 01Ch
; C:\PBP\PBPPIC18.RAM      	00024	A0001D	RR2     VAR     BYTE BANKA SYSTEM       ' Pin 2 Register
RR2             		EQU	RAM_START + 01Dh
; C:\PBP\PBPPIC18.RAM      	00022	A0001E	RS1     VAR     BYTE BANKA SYSTEM       ' Pin 1 Bank
RS1             		EQU	RAM_START + 01Eh
; C:\PBP\PBPPIC18.RAM      	00025	A0001F	RS2     VAR     BYTE BANKA SYSTEM       ' Pin 2 Bank
RS2             		EQU	RAM_START + 01Fh
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00001	A00020	' ************************************************************
PB01            		EQU	RAM_START + 020h
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00048	A00021	aux var byte
_aux             		EQU	RAM_START + 021h
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00045	A00022	i var byte
_i               		EQU	RAM_START + 022h
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00051	A00023	pos var byte
_pos             		EQU	RAM_START + 023h
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00050	A00024	sw1 var byte
_sw1             		EQU	RAM_START + 024h
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00047	A00025	temp var byte
_temp            		EQU	RAM_START + 025h
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00026	A00026	USBBufferCount   Var Byte 
_USBBufferCount  		EQU	RAM_START + 026h
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00049	A00027	var1 var byte
_var1            		EQU	RAM_START + 027h
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00025	A00028	USBBuffer        Var Byte[USBBufferSizeMax] 
_USBBuffer       		EQU	RAM_START + 028h
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00024	A00030	USBBufferaux        Var Byte[USBBufferSizeMax + 1]
_USBBufferaux    		EQU	RAM_START + 030h
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\DESCUSBPROJECT.BAS	00005	A00400	USBReservedMemory Var Byte[USBMEMORYSIZE] USBMEMORYADDRESS	' Reserve memory used by USB assembler code
_USBReservedMemory		EQU	RAM_START + 00400h
; C:\PBP\18F2550.BAS       	00026	PORTL   VAR     PORTB
_PORTL           		EQU	 PORTB
; C:\PBP\18F2550.BAS       	00027	PORTH   VAR     PORTC
_PORTH           		EQU	 PORTC
; C:\PBP\18F2550.BAS       	00028	TRISL   VAR     TRISB
_TRISL           		EQU	 TRISB
; C:\PBP\18F2550.BAS       	00029	TRISH   VAR     TRISC
_TRISH           		EQU	 TRISC
#define _pin1            	 PB01, 000h
#define _PORTB??3        	 PORTB, 003h
#define _PORTB??2        	 PORTB, 002h
#define _PORTB??1        	 PORTB, 001h
#define _PORTB??0        	 PORTB, 000h
#define _PORTB??4        	 PORTB, 004h
#define _PORTB??5        	 PORTB, 005h
#define _PORTB??6        	 PORTB, 006h
#define _PORTB??7        	 PORTB, 007h

; Constants.
_USBMEMORYADDRESS		EQU	00400h
_USBMEMORYSIZE   		EQU	00100h
_USBBufferSizeMax		EQU	00008h
_USBBufferSizeTX 		EQU	00008h
_USBBufferSizeRX 		EQU	00008h

; EEPROM data.


	INCLUDE	"ORIGUS~1.MAC"
	INCLUDE	"C:\PBP\PBPPIC18.LIB"


; C:\PBP\18F2550.BAS       	00012	BANKA   $0000, $005F
; C:\PBP\18F2550.BAS       	00013	BANK0   $0060, $00FF
; C:\PBP\18F2550.BAS       	00014	BANK1   $0100, $01FF
; C:\PBP\18F2550.BAS       	00015	BANK2   $0200, $02FF
; C:\PBP\18F2550.BAS       	00016	BANK3   $0300, $03FF
; C:\PBP\18F2550.BAS       	00017	BANK4   $0400, $04FF
; C:\PBP\18F2550.BAS       	00018	BANK5   $0500, $05FF
; C:\PBP\18F2550.BAS       	00019	BANK6   $0600, $06FF
; C:\PBP\18F2550.BAS       	00020	BANK7   $0700, $07FF
; C:\PBP\18F2550.BAS       	00022	LIBRARY "PBPPIC18"

; C:\PBP\18F2550.BAS       	00024	        include "PIC18EXT.BAS"

; C:\PBP\18F2550.BAS       	00031	        include "PBPPIC18.RAM"
; C:\PBP\18F2550.BAS       	00032	USBMEMORYADDRESS Con	$400	' USB RAM starts here

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00006	include	"DESCUSBProject.bas"
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\DESCUSBPROJECT.BAS	00004	USBMEMORYSIZE	Con	256	' USB RAM size in bytes

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\DESCUSBPROJECT.BAS	00007	goto	hid_desc_end	' Skip over all of the USB assembler code
	GOTO?L	_hid_desc_end

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\DESCUSBPROJECT.BAS	00008	asm

	ASM?


#define	USB_EP0_BUFF_SIZE 	64	; 8, 16, 32, or 64
#define	USB_MAX_NUM_INT		1	; For tracking Alternate Setting
#define	USB_MAX_EP_NUMBER  	1	; UEP1
#define	NUM_CONFIGURATIONS	1
#define	NUM_INTERFACES		1

#define UCFG_VAL	USB_PULLUP_ENABLE|USB_INTERNAL_TRANSCEIVER|USB_FULL_SPEED|USB_PING_PONG__NO_PING_PONG
;#define UCFG_VAL	USB_PULLUP_ENABLE|USB_INTERNAL_TRANSCEIVER|USB_LOW_SPEED|USB_PING_PONG__NO_PING_PONG

;#define USE_SELF_POWER_SENSE_IO
;#define USE_USB_BUS_SENSE_IO

#define USB_POLLING

; HID
; Endpoints Allocation
#define	HID_INTF_ID		  0x00
#define	HID_EP			  1
#define	HID_INT_OUT_EP_SIZE	  64
#define	HID_INT_IN_EP_SIZE	  64
#define	HID_NUM_OF_DSC		  1

   include	"usb_hid.asm"	; Include rest of USB files, starting with HID class code

; ******************************************************************
; This table is polled by the host immediately after USB Reset has been released.
; This table defines the maximum packet size EP0 can take.
; See section 9.6.1 of the Rev 1.0 USB specification.
; These fields are application DEPENDENT. Modify these to meet
; your specifications.
; ******************************************************************
DeviceDescriptor
	retlw	(EndDeviceDescriptor-DeviceDescriptor)/2	; bLength Length of this descriptor
	retlw	USB_DESCRIPTOR_DEVICE ; bDescType This is a DEVICE descriptor
	retlw	0x10		; bcdUSBUSB Revision 1.10 (low byte)
	retlw	0x01		; high byte
	retlw	0x00		; bDeviceClass zero means each interface operates independently
	retlw	0x00		; bDeviceSubClass
	retlw	0x00		; bDeviceProtocol
	retlw	USB_EP0_BUFF_SIZE ; bMaxPacketSize for EP0

        ; idVendor (low byte, high byte)
	retlw	0xD8
	retlw	0x04

        ; idProduct (low byte, high byte)
	retlw	0x0C
	retlw	0x00

        retlw	0x00		; bcdDevice (low byte)
	retlw	0x00		; (high byte)
	retlw	0x01		; iManufacturer (string index)
	retlw	0x02		; iProduct      (string index)

        ; iSerialNumber (string index)
	retlw	0x03
	retlw	NUM_CONFIGURATIONS ; bNumConfigurations
EndDeviceDescriptor

; ******************************************************************
; This table is retrieved by the host after the address has been set.
; This table defines the configurations available for the device.
; See section 9.6.2 of the Rev 1.0 USB specification (page 184).
; These fields are application DEPENDENT. 
; Modify these to meet your specifications.
; ******************************************************************
; Configuration pointer table
USB_CD_Ptr
Configs
	db	low Config1, high Config1
	db	upper Config1, 0

; Configuration Descriptor
Config1
	retlw	(Interface1-Config1)/2	; bLength Length of this descriptor
	retlw	USB_DESCRIPTOR_CONFIGURATION ; bDescType 2=CONFIGURATION
Config1Len
	retlw	low ((EndConfig1 - Config1)/2)	; Length of this configuration
	retlw	high ((EndConfig1 - Config1)/2)
	retlw	0x01		; bNumInterfaces Number of interfaces
	retlw	0x01		; bConfigValue Configuration Value
	retlw	0x00		; iConfig (string index)
	retlw	_DEFAULT|_SELF	; bmAttributes attributes - bus powered

        ; Max power consumption (2X mA)
	retlw	0x32
Interface1
	retlw	(HIDDescriptor1-Interface1)/2	; length of descriptor
	retlw	USB_DESCRIPTOR_INTERFACE
	retlw	0x00		; number of interface, 0 based array
	retlw	0x00		; alternate setting
	retlw	0x02		; number of endpoints used in this interface
	retlw	0x03		; interface class - assigned by the USB
	retlw	0x00		; boot device
	retlw	0x00		; interface protocol
	retlw 	0x00		; index to string descriptor that describes this interface
HIDDescriptor1
	retlw	(Endpoint1In-HIDDescriptor1)/2	; descriptor size (9 bytes)
        retlw	DSC_HID		; descriptor type (HID)
	retlw	0x11		; HID class release number (1.11)
	retlw	0x01
        retlw	0x00		; Localized country code (none)
        retlw	0x01		; # of HID class descriptor to follow (1)
        retlw	0x22		; Report descriptor type (HID)
ReportDescriptor1Len
	retlw	low ((EndReportDescriptor1-ReportDescriptor1)/2)
	retlw	high ((EndReportDescriptor1-ReportDescriptor1)/2)
Endpoint1In
	retlw	(EndPoint1Out-Endpoint1In)/2	; length of descriptor
	retlw	USB_DESCRIPTOR_ENDPOINT
	retlw	HID_EP|_EP_IN		; EP1, In
	retlw	_INT		; Interrupt
	retlw	low (HID_INT_IN_EP_SIZE)		; This should be the size of the endpoint buffer
	retlw	high (HID_INT_IN_EP_SIZE)
	retlw	0x0A                        ; Polling interval
EndPoint1Out
	retlw	(EndConfig1-EndPoint1Out)/2	; Length of this Endpoint Descriptor
	retlw	USB_DESCRIPTOR_ENDPOINT		; bDescriptorType = 5 for Endpoint Descriptor
	retlw	HID_EP|_EP_OUT		; Endpoint number & direction
	retlw	_INT		; Transfer type supported by this Endpoint
	retlw	low (HID_INT_OUT_EP_SIZE)		; This should be the size of the endpoint buffer
	retlw	high (HID_INT_OUT_EP_SIZE)
	retlw	0x0A                        ; Polling interval
EndConfig1

ReportDescriptor1
    ; vendor defined usage page
    retlw	0x06		
    retlw	0x00
    retlw	0xFF

    ; vendor defined usage
    retlw	0x09
    retlw	0x00

    ; collection(application)
    retlw	0xA1
    retlw	0x01

    ; *** INPUT REPORT ***

    ; vendor defined usage
    retlw	0x09
    retlw	0x01

    retlw	0x15 	; logical minimum (-128)
    retlw	0x80    ;
    retlw	0x25 	; logical maximum (127)
    retlw	0x7F    ;
    retlw	0x35 	; Physical Minimum (0)
    retlw	0x00    ;
    retlw	0x45 	; Physical Maximum (255)
    retlw	0xFF    ;

    ; report size in bits
    retlw	0x75
    retlw	0x08

    ; report count (number of fields)
    retlw	0x95
    retlw	0x08

    ; Input (Data, Variable, Absolute)
    retlw	0x81
    retlw	0x02

    ; *** OUTPUT REPORT ***

    ; vendor defined usage
    retlw	0x09	    ; usage (Vendor Defined)
    retlw	0x02        ;

    retlw	0x15	    ; logical minimum (-128)
    retlw	0x80        ;
    retlw	0x25	    ; logical maximum (127)
    retlw	0x7F        ;
    retlw	0x35	    ; Physical Minimum (0)
    retlw	0x00        ;
    retlw	0x45	    ; Physical Maximum (255)
    retlw	0xFF        ;

    ; report size in bits
    retlw	0x75
    retlw	0x08

    ; report count (number of fields)
    retlw	0x95
    retlw	0x08

    ; Output (Data, Variable, Absolute)
    retlw	0x91
    retlw	0x02

    retlw   0xC0       	   ; end collection

EndReportDescriptor1

; String pointer table
USB_SD_Ptr
Strings
	db	low String0, high String0
        db	upper String0, 0
	db	low String1, high String1
        db	upper String1, 0
	db	low String2, high String2
       	db	upper String2, 0
	db	low String3, high String3
	db	upper String3, 0

String0
	retlw	(String1-String0)/2	; Length of string
	retlw	USB_DESCRIPTOR_STRING   ; Descriptor type 3
	retlw	0x09		        ; Language ID (as defined by MS 0x0409)
	retlw	0x04

; company name
String1
	retlw	(String2-String1)/2
	retlw	USB_DESCRIPTOR_STRING
	
        retlw   'h'
        retlw   0x00
        retlw   't'
        retlw   0x00
        retlw   'g'
        retlw   0x00
        retlw   'a'
        retlw   0x00
        retlw   'm'
        retlw   0x00
        retlw   'e'
        retlw   0x00
        retlw   's'
        retlw   0x00

	
; product name	
String2
	retlw	(String3-String2)/2
	retlw	USB_DESCRIPTOR_STRING
	
        retlw   'p'
        retlw   0x00
        retlw   'r'
        retlw   0x00
        retlw   'u'
        retlw   0x00
        retlw   'e'
        retlw   0x00
        retlw   'b'
        retlw   0x00
        retlw   'a'
        retlw   0x00


; serial number
String3
	retlw	(String4-String3)/2
	retlw	USB_DESCRIPTOR_STRING
	
        retlw   '1'
        retlw   0x00
        retlw   '2'
        retlw   0x00
        retlw   '3'
        retlw   0x00
        retlw   '4'
        retlw   0x00

String4




	ENDASM?


; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\DESCUSBPROJECT.BAS	00283	hid_desc_end

	LABEL?L	_hid_desc_end	
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00010	DEFINE OSC 48
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00011	DEFINE RESET_ORG 1000h

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00013	trisb = %00000000
	MOVE?CB	000h, TRISB

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00014	portb = %00000000
	MOVE?CB	000h, PORTB

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00016	trisa = %00000000
	MOVE?CB	000h, TRISA

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00017	porta = %00000000
	MOVE?CB	000h, PORTA
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00019	USBBufferSizeMax   con 8  ' maximum buffer size
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00020	USBBufferSizeTX    con 8  ' input
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00021	USBBufferSizeRX    con 8  ' output
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00029	DEFINE DEBUG_REG PORTA 
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00032	DEFINE DEBUG_BIT 0 
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00035	DEFINE DEBUG_BAUD 9600
; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00038	DEFINE DEBUG_MODE 1

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00053	pause 500
	PAUSE?C	001F4h

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00054	DEbug "B-Olivia ID: 01",13
	DEBUG?C	042h
	DEBUG?C	02Dh
	DEBUG?C	04Fh
	DEBUG?C	06Ch
	DEBUG?C	069h
	DEBUG?C	076h
	DEBUG?C	069h
	DEBUG?C	061h
	DEBUG?C	020h
	DEBUG?C	049h
	DEBUG?C	044h
	DEBUG?C	03Ah
	DEBUG?C	020h
	DEBUG?C	030h
	DEBUG?C	031h
	DEBUG?C	00Dh

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00055	debug "ON "
	DEBUG?C	04Fh
	DEBUG?C	04Eh
	DEBUG?C	020h

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00056	pause 500
	PAUSE?C	001F4h

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00057	debug "ON",13
	DEBUG?C	04Fh
	DEBUG?C	04Eh
	DEBUG?C	00Dh

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00059	i = 0
	MOVE?CB	000h, _i

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00060	sw1 = 0
	MOVE?CB	000h, _sw1

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00061	pos = 1
	MOVE?CB	001h, _pos

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00064	pause 400
	PAUSE?C	00190h

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00065	high portb.3
	HIGH?T	_PORTB??3

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00066	pause 200
	PAUSE?C	0C8h

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00067	low portb.3
	LOW?T	_PORTB??3

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00068	pause 200
	PAUSE?C	0C8h

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00069	high portb.3
	HIGH?T	_PORTB??3

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00070	pause 200
	PAUSE?C	0C8h

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00071	low portb.3
	LOW?T	_PORTB??3

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00072	pause 400
	PAUSE?C	00190h

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00073	high portb.3
	HIGH?T	_PORTB??3

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00074	pause 500
	PAUSE?C	001F4h

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00075	low portb.3
	LOW?T	_PORTB??3

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00076	pause 700
	PAUSE?C	002BCh

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00077	high portb.3
	HIGH?T	_PORTB??3

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00078	high portb.2
	HIGH?T	_PORTB??2

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00079	pause 700
	PAUSE?C	002BCh

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00086	usbinit ' initialise USB...
	USBINIT?	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00088	low portb.3
	LOW?T	_PORTB??3

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00089	low portb.2
	LOW?T	_PORTB??2

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00090	pause 500
	PAUSE?C	001F4h

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00091	high portb.3
	HIGH?T	_PORTB??3

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00092	high portb.2
	HIGH?T	_PORTB??2

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00093	pause 500
	PAUSE?C	001F4h

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00097	gosub DoUSBIn
	GOSUB?L	_DoUSBIn

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00105	ProgramStart:

	LABEL?L	_ProgramStart	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00106	   portb.1 = 1 
	MOVE?CT	001h, _PORTB??1

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00107	   pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00108	   gosub DoUSBIn 
	GOSUB?L	_DoUSBIn

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00109	   portb.1 = 0
	MOVE?CT	000h, _PORTB??1

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00110	   pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00111	   portb.1 = 1
	MOVE?CT	001h, _PORTB??1

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00112	   pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00113	   portb.1 = 0
	MOVE?CT	000h, _PORTB??1

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00114	   pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00115	   portb.1 = 1
	MOVE?CT	001h, _PORTB??1

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00116	   pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00117	   portb.1 = 0
	MOVE?CT	000h, _PORTB??1

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00118	   pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00120	   for i = 0 to  7
	MOVE?CB	000h, _i
	LABEL?L	L00009	
	CMPGT?BCL	_i, 007h, L00010

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00121	     usbbufferaux[i+1] =  usbbuffer[i]
	ADD?BCW	_i, 001h, T2
	AOUT?BBB	_USBBuffer, _i, T1
	AIN?BBW	T1, _USBBufferaux, T2

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00122	   next
	NEXT?BCL	_i, 001h, L00009
	LABEL?L	L00010	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00123	   pos = usbbufferaux[pos]
	AOUT?BBB	_USBBufferaux, _pos, _pos

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00124	   while (pos > 8)
	LABEL?L	L00011	
	CMPLE?BCL	_pos, 008h, L00012

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00125	     pos = pos - 8
	SUB?BCB	_pos, 008h, _pos

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00126	   wend  
	GOTO?L	L00011
	LABEL?L	L00012	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00130	   if ((usbbuffer[0] = 3)and (usbbuffer[1] = 255) and (usbbuffer[1] = 255)) then
	CMPEQ?BCB	_USBBuffer, 003h, T1
	CMPEQ?BCB	_USBBuffer + 00001h, 0FFh, T2
	LAND?BBW	T1, T2, T2
	CMPEQ?BCB	_USBBuffer + 00001h, 0FFh, T3
	LAND?WBW	T2, T3, T3
	CMPF?WL	T3, L00013

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00131	     portb.3 = 1
	MOVE?CT	001h, _PORTB??3

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00132	     pause 1000
	PAUSE?C	003E8h

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00133	   endif
	LABEL?L	L00013	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00134	   if ((usbbuffer[0] = 3)and (usbbuffer[1] = 254)and (usbbuffer[1] = 254)) then
	CMPEQ?BCB	_USBBuffer, 003h, T1
	CMPEQ?BCB	_USBBuffer + 00001h, 0FEh, T2
	LAND?BBW	T1, T2, T2
	CMPEQ?BCB	_USBBuffer + 00001h, 0FEh, T3
	LAND?WBW	T2, T3, T3
	CMPF?WL	T3, L00015

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00135	     portb.3 = 0
	MOVE?CT	000h, _PORTB??3

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00136	     pause 1000
	PAUSE?C	003E8h

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00137	   endif 
	LABEL?L	L00015	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00148	   temp = usbbuffer[0]
	MOVE?BB	_USBBuffer, _temp

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00150	   debug "Dato USB HOST recibido... ",13   
	DEBUG?C	044h
	DEBUG?C	061h
	DEBUG?C	074h
	DEBUG?C	06Fh
	DEBUG?C	020h
	DEBUG?C	055h
	DEBUG?C	053h
	DEBUG?C	042h
	DEBUG?C	020h
	DEBUG?C	048h
	DEBUG?C	04Fh
	DEBUG?C	053h
	DEBUG?C	054h
	DEBUG?C	020h
	DEBUG?C	072h
	DEBUG?C	065h
	DEBUG?C	063h
	DEBUG?C	069h
	DEBUG?C	062h
	DEBUG?C	069h
	DEBUG?C	064h
	DEBUG?C	06Fh
	DEBUG?C	02Eh
	DEBUG?C	02Eh
	DEBUG?C	02Eh
	DEBUG?C	020h
	DEBUG?C	00Dh

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00151	   for i = 0 to 7
	MOVE?CB	000h, _i
	LABEL?L	L00017	
	CMPGT?BCL	_i, 007h, L00018

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00155	     aux = i
	MOVE?BB	_i, _aux

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00160	     if pos = 1 then      
	CMPNE?BCL	_pos, 001h, L00019

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00161	       gosub buscar1
	GOSUB?L	_buscar1

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00162	     endif
	LABEL?L	L00019	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00163	     if pos = 2 then      
	CMPNE?BCL	_pos, 002h, L00021

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00164	       gosub buscar2
	GOSUB?L	_buscar2

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00165	     endif
	LABEL?L	L00021	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00166	     if pos = 3 then      
	CMPNE?BCL	_pos, 003h, L00023

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00167	       gosub buscar3
	GOSUB?L	_buscar3

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00168	     endif
	LABEL?L	L00023	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00169	     if pos = 4 then      
	CMPNE?BCL	_pos, 004h, L00025

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00170	       gosub buscar4
	GOSUB?L	_buscar4

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00171	     endif     
	LABEL?L	L00025	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00172	     if pos = 5 then      
	CMPNE?BCL	_pos, 005h, L00027

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00173	       gosub buscar5
	GOSUB?L	_buscar5

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00174	     endif
	LABEL?L	L00027	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00175	     if pos = 6 then      
	CMPNE?BCL	_pos, 006h, L00029

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00176	       gosub buscar6
	GOSUB?L	_buscar6

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00177	     endif
	LABEL?L	L00029	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00178	     if pos = 7 then      
	CMPNE?BCL	_pos, 007h, L00031

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00179	       gosub buscar7
	GOSUB?L	_buscar7

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00180	     endif
	LABEL?L	L00031	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00181	     if pos = 8 then      
	CMPNE?BCL	_pos, 008h, L00033

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00182	       gosub buscar8
	GOSUB?L	_buscar8

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00183	     endif
	LABEL?L	L00033	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00186	     debug hex usbbuffer[i], "-" 
	DEBUGCOUNT?C	000h
	AOUT?BBB	_USBBuffer, _i, T1
	DEBUGNUM?B	T1
	DEBUGHEX?	
	DEBUG?C	02Dh

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00188	     if ((i = 0)  and (usbbuffer[i] <= 240)) then
	CMPEQ?BCB	_i, 000h, T1
	AOUT?BBB	_USBBuffer, _i, T2
	CMPLE?BCB	T2, 0F0h, T2
	LAND?BBW	T1, T2, T2
	CMPF?WL	T2, L00035

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00189	       usbbuffer[i] = usbbuffer[i] + var1
	AOUT?BBB	_USBBuffer, _i, T1
	ADD?BBW	T1, _var1, T1
	AIN?WBB	T1, _USBBuffer, _i

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00190	     endif
	LABEL?L	L00035	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00191	     if ((i = 1)  and (usbbuffer[i] <= 240)) then
	CMPEQ?BCB	_i, 001h, T1
	AOUT?BBB	_USBBuffer, _i, T2
	CMPLE?BCB	T2, 0F0h, T2
	LAND?BBW	T1, T2, T2
	CMPF?WL	T2, L00037

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00192	       usbbuffer[i] = usbbuffer[i] + var1
	AOUT?BBB	_USBBuffer, _i, T1
	ADD?BBW	T1, _var1, T1
	AIN?WBB	T1, _USBBuffer, _i

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00193	     endif
	LABEL?L	L00037	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00194	     if ((i = 2) and (usbbuffer[i] >= 28)) then    '28
	CMPEQ?BCB	_i, 002h, T1
	AOUT?BBB	_USBBuffer, _i, T2
	CMPGE?BCB	T2, 01Ch, T2
	LAND?BBW	T1, T2, T2
	CMPF?WL	T2, L00039

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00195	       usbbuffer[i] = usbbuffer[i] // var1
	AOUT?BBB	_USBBuffer, _i, T1
	MOD?BBW	T1, _var1, T1
	AIN?WBB	T1, _USBBuffer, _i

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00196	     endif
	LABEL?L	L00039	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00197	     if ((i = 3) and (usbbuffer[i] <= 17)) then    '28
	CMPEQ?BCB	_i, 003h, T1
	AOUT?BBB	_USBBuffer, _i, T2
	CMPLE?BCB	T2, 011h, T2
	LAND?BBW	T1, T2, T2
	CMPF?WL	T2, L00041

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00198	       usbbuffer[i] = usbbuffer[i] * var1
	AOUT?BBB	_USBBuffer, _i, T1
	MUL?BBW	T1, _var1, T1
	AIN?WBB	T1, _USBBuffer, _i

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00199	     endif
	LABEL?L	L00041	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00200	     if ((i = 4)  and (usbbuffer[i] <= 240)) then
	CMPEQ?BCB	_i, 004h, T1
	AOUT?BBB	_USBBuffer, _i, T2
	CMPLE?BCB	T2, 0F0h, T2
	LAND?BBW	T1, T2, T2
	CMPF?WL	T2, L00043

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00201	       usbbuffer[i] = usbbuffer[i] + var1
	AOUT?BBB	_USBBuffer, _i, T1
	ADD?BBW	T1, _var1, T1
	AIN?WBB	T1, _USBBuffer, _i

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00202	     endif
	LABEL?L	L00043	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00203	     if ((i = 5) and (usbbuffer[i] >= 17)) then
	CMPEQ?BCB	_i, 005h, T1
	AOUT?BBB	_USBBuffer, _i, T2
	CMPGE?BCB	T2, 011h, T2
	LAND?BBW	T1, T2, T2
	CMPF?WL	T2, L00045

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00204	       usbbuffer[i] = usbbuffer[i] - var1
	AOUT?BBB	_USBBuffer, _i, T1
	SUB?BBW	T1, _var1, T1
	AIN?WBB	T1, _USBBuffer, _i

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00205	     endif
	LABEL?L	L00045	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00206	     if ((i = 6) and (usbbuffer[i] <= 17)) then
	CMPEQ?BCB	_i, 006h, T1
	AOUT?BBB	_USBBuffer, _i, T2
	CMPLE?BCB	T2, 011h, T2
	LAND?BBW	T1, T2, T2
	CMPF?WL	T2, L00047

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00207	       usbbuffer[i] = usbbuffer[i] * var1
	AOUT?BBB	_USBBuffer, _i, T1
	MUL?BBW	T1, _var1, T1
	AIN?WBB	T1, _USBBuffer, _i

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00208	     endif
	LABEL?L	L00047	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00209	     if ((i = 7)  and (usbbuffer[i] <= 240)) then
	CMPEQ?BCB	_i, 007h, T1
	AOUT?BBB	_USBBuffer, _i, T2
	CMPLE?BCB	T2, 0F0h, T2
	LAND?BBW	T1, T2, T2
	CMPF?WL	T2, L00049

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00210	       usbbuffer[i] = usbbuffer[i] + var1
	AOUT?BBB	_USBBuffer, _i, T1
	ADD?BBW	T1, _var1, T1
	AIN?WBB	T1, _USBBuffer, _i

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00211	     endif
	LABEL?L	L00049	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00212	     pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00213	   next
	NEXT?BCL	_i, 001h, L00017
	LABEL?L	L00018	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00215	   debug 13, "Dato USB LOCAL enviado... ",13
	DEBUG?C	00Dh
	DEBUG?C	044h
	DEBUG?C	061h
	DEBUG?C	074h
	DEBUG?C	06Fh
	DEBUG?C	020h
	DEBUG?C	055h
	DEBUG?C	053h
	DEBUG?C	042h
	DEBUG?C	020h
	DEBUG?C	04Ch
	DEBUG?C	04Fh
	DEBUG?C	043h
	DEBUG?C	041h
	DEBUG?C	04Ch
	DEBUG?C	020h
	DEBUG?C	065h
	DEBUG?C	06Eh
	DEBUG?C	076h
	DEBUG?C	069h
	DEBUG?C	061h
	DEBUG?C	064h
	DEBUG?C	06Fh
	DEBUG?C	02Eh
	DEBUG?C	02Eh
	DEBUG?C	02Eh
	DEBUG?C	020h
	DEBUG?C	00Dh

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00217	   for i = 0 to 7
	MOVE?CB	000h, _i
	LABEL?L	L00051	
	CMPGT?BCL	_i, 007h, L00052

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00218	     debug hex usbbuffer[i], "-"
	DEBUGCOUNT?C	000h
	AOUT?BBB	_USBBuffer, _i, T1
	DEBUGNUM?B	T1
	DEBUGHEX?	
	DEBUG?C	02Dh

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00219	     pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00220	   next
	NEXT?BCL	_i, 001h, L00051
	LABEL?L	L00052	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00221	   debug 13,13
	DEBUG?C	00Dh
	DEBUG?C	00Dh

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00233	   portb.0 = 1
	MOVE?CT	001h, _PORTB??0

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00234	   pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00235	   gosub DoUSBOut
	GOSUB?L	_DoUSBOut

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00236	   portb.0 = 0
	MOVE?CT	000h, _PORTB??0

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00237	   pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00238	   goto ProgramStart  
	GOTO?L	_ProgramStart

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00243	DoUSBIn:

	LABEL?L	_DoUSBIn	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00244	   USBBufferCount = USBBufferSizeRX              ' RX buffer size
	MOVE?CB	_USBBufferSizeRX, _USBBufferCount

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00245	   USBService                                    ' keep connection alive
	USBSERVICE?	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00246	   portb.4 = 1
	MOVE?CT	001h, _PORTB??4

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00247	   pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00248	   portb.4 = 0
	MOVE?CT	000h, _PORTB??4

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00249	   pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00250	   USBIn 1, USBBuffer, USBBufferCount, DoUSBIn   ' read data, if available
	USBIN?CBBL	001h, _USBBuffer, _USBBufferCount, _DoUSBIn

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00251	   portb.5 = 1
	MOVE?CT	001h, _PORTB??5

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00252	   pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00253	   portb.5 = 0
	MOVE?CT	000h, _PORTB??5

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00254	   pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00255	   return
	RETURN?	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00260	DoUSBOut:

	LABEL?L	_DoUSBOut	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00261	   USBBufferCount = USBBufferSizeTX              ' TX buffer size
	MOVE?CB	_USBBufferSizeTX, _USBBufferCount

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00262	   USBService                                    ' keep connection alive
	USBSERVICE?	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00263	   portb.6 = 1
	MOVE?CT	001h, _PORTB??6

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00264	   pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00265	   portb.6 = 0
	MOVE?CT	000h, _PORTB??6

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00266	   pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00267	   USBOut 1, USBBuffer, USBBufferCount, DoUSBOut ' if bus available, transmit data
	USBOUT?CBBL	001h, _USBBuffer, _USBBufferCount, _DoUSBOut

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00268	   portb.7 = 1
	MOVE?CT	001h, _PORTB??7

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00269	   pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00270	   portb.7 = 0
	MOVE?CT	000h, _PORTB??7

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00271	   pause 10
	PAUSE?C	00Ah

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00272	   return
	RETURN?	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00274	buscar1:

	LABEL?L	_buscar1	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00275	  lookup aux,[1,8,3,2,3,3,4,2],var1
	LOOKUP?BCLB	_aux, 008h, L00001, _var1
	LURET?C	001h
	LURET?C	008h
	LURET?C	003h
	LURET?C	002h
	LURET?C	003h
	LURET?C	003h
	LURET?C	004h
	LURET?C	002h

	LABEL?L	L00001	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00276	return 
	RETURN?	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00277	buscar2:

	LABEL?L	_buscar2	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00278	  lookup aux,[8,2,5,4,7,4,3,6],var1
	LOOKUP?BCLB	_aux, 008h, L00002, _var1
	LURET?C	008h
	LURET?C	002h
	LURET?C	005h
	LURET?C	004h
	LURET?C	007h
	LURET?C	004h
	LURET?C	003h
	LURET?C	006h

	LABEL?L	L00002	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00279	return 
	RETURN?	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00280	buscar3:

	LABEL?L	_buscar3	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00281	  lookup aux,[3,7,8,7,5,1,2,9],var1
	LOOKUP?BCLB	_aux, 008h, L00003, _var1
	LURET?C	003h
	LURET?C	007h
	LURET?C	008h
	LURET?C	007h
	LURET?C	005h
	LURET?C	001h
	LURET?C	002h
	LURET?C	009h

	LABEL?L	L00003	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00282	return 
	RETURN?	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00283	buscar4:

	LABEL?L	_buscar4	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00284	  lookup aux,[2,6,7,3,1,6,5,1],var1
	LOOKUP?BCLB	_aux, 008h, L00004, _var1
	LURET?C	002h
	LURET?C	006h
	LURET?C	007h
	LURET?C	003h
	LURET?C	001h
	LURET?C	006h
	LURET?C	005h
	LURET?C	001h

	LABEL?L	L00004	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00285	return 
	RETURN?	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00286	buscar5:

	LABEL?L	_buscar5	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00287	  lookup aux,[9,8,1,2,3,5,4,2],var1
	LOOKUP?BCLB	_aux, 008h, L00005, _var1
	LURET?C	009h
	LURET?C	008h
	LURET?C	001h
	LURET?C	002h
	LURET?C	003h
	LURET?C	005h
	LURET?C	004h
	LURET?C	002h

	LABEL?L	L00005	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00288	return 
	RETURN?	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00289	buscar6:

	LABEL?L	_buscar6	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00290	  lookup aux,[8,1,1,4,5,4,3,6],var1
	LOOKUP?BCLB	_aux, 008h, L00006, _var1
	LURET?C	008h
	LURET?C	001h
	LURET?C	001h
	LURET?C	004h
	LURET?C	005h
	LURET?C	004h
	LURET?C	003h
	LURET?C	006h

	LABEL?L	L00006	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00291	return 
	RETURN?	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00292	buscar7:

	LABEL?L	_buscar7	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00293	  lookup aux,[3,2,8,2,5,1,2,9],var1
	LOOKUP?BCLB	_aux, 008h, L00007, _var1
	LURET?C	003h
	LURET?C	002h
	LURET?C	008h
	LURET?C	002h
	LURET?C	005h
	LURET?C	001h
	LURET?C	002h
	LURET?C	009h

	LABEL?L	L00007	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00294	return 
	RETURN?	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00295	buscar8:

	LABEL?L	_buscar8	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00296	  lookup aux,[1,2,7,3,5,6,5,1],var1
	LOOKUP?BCLB	_aux, 008h, L00008, _var1
	LURET?C	001h
	LURET?C	002h
	LURET?C	007h
	LURET?C	003h
	LURET?C	005h
	LURET?C	006h
	LURET?C	005h
	LURET?C	001h

	LABEL?L	L00008	

; C:\HIRO\ELECTR~1\USBPRO~1\OLIVIA~1\ORIGUS~1.PBP	00297	return 
	RETURN?	

	END
