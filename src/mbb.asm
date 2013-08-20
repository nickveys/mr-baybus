;----------------------------------------------------------------------------------------
; Copyright (c) 2000, Nicholas Veys, veys.com  All rights reserved.
; 
; Redistribution and use in source and binary forms, with or without modification, 
;   are permitted provided that the following conditions are met:
;
; * Redistributions of source code must retain the above copyright notice, this 
;     list of conditions and the following disclaimer.
; * Redistributions in binary form must reproduce the above copyright notice, 
;     this list of conditions and the following disclaimer in the documentation 
;     and/or other materials provided with the distribution.
; * Neither the name of the VEYS.COM nor the names of its contributors may be used 
;     to endorse or promote products derived from this software without specific 
;     prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
; EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
; OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
; SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
; INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
; TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
; BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
; ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
; DAMAGE.
;----------------------------------------------------------------------------------------

 title  "Mr. BayBus Fan System!"

  LIST R=DEC
  INCLUDE "p16f84.inc"
 __CONFIG _CP_OFF & _WDT_OFF & _XT_OSC & _PWRTE_ON

;----------------------------------------------------------------------------------------
; Variable declarations
;----------------------------------------------------------------------------------------
 CBLOCK 0x0C
_w, _status					; interrupt push variables
spi_flag, spi_byte, spi_state			; spi transfer variables
delayvar, delaycounter				; delay routine variables
mode, buttonflags				; main program variables
backlight					; backlight setting
contrast					; contrast setting
fanstatus					; fan status flags
eeprom_address, eeprom_data			; EEPROM function variables
 ENDC

;----------------------------------------------------------------------------------------
; #define's
;----------------------------------------------------------------------------------------
#define SPI_ISR  spi_flag,1			; spi transfer signifier
#define SPI_DONE spi_flag,0			; spi transfer complete flag
#define SPI_CS   PORTA,0			; spi chip select line
#define SPI_CLK  PORTA,1			; spi clock line
#define SPI_DATA PORTA,2			; spi data line
#define DELAY_ISR delayvar,0			; delay routine signifier
#define _DEBOUNCE 2				; 131ms debounce delay time 
#define _STARTUP 7				; LCD startup delay
#define _1 15					; 1 second
#define _2 31					; 2 seconds
#define _3 46					; 3 seconds
#define _4 61					; 4 seconds
#define _5 76					; 5 seconds
#define BUTTON_1 PORTB,0			; 1 button
#define BUTTON_2 PORTB,1			; 2 button
#define BUTTON_3 PORTB,2			; 3 button
#define BUTTON_4 PORTB,3			; 4 button
#define FAN_1	 PORTB,4			; fan 1 line
#define FAN_2	 PORTB,5			; fan 2 line
#define FAN_3	 PORTB,6			; fan 3 line
#define FAN_4	 PORTB,7			; fan 4 line
#define BUTTON_OPTIONS PORTA,3			; options toggle button
#define CHANGE_OPTIONS buttonflags,0		; options toggle indicator
#define F1_STATUS fanstatus,0			; fan 1 status
#define F2_STATUS fanstatus,1			; fan 2 status
#define F3_STATUS fanstatus,2			; fan 3 status
#define F4_STATUS fanstatus,3			; fan 4 status
#define BACKLIGHT_ADDR 	0x00			; EEPROM address for backlight
#define CONTRAST_ADDR 	0x01			; EEPROM address for contrast
#define FAN_ADDR 	0x02			; EEPROM address for fan config

;----------------------------------------------------------------------------------------
; Macro declarations
;----------------------------------------------------------------------------------------
BANK0 macro
  bcf STATUS, RP0           			; Goto Bank 0
  endm

BANK1 macro
  bsf STATUS, RP0           			; Goto Bank 1
  endm

READ_DATA macro ADDRESS
  movlw ADDRESS
  movwf eeprom_address
  call EEPROM_read
  endm

WRITE_DATA macro ADDRESS
  movlw ADDRESS
  movwf eeprom_address
  call EEPROM_write
  endm

SET_OPTION_REG macro DATA
  BANK1
  movlw DATA
  movwf OPTION_REG ^ 0x80
  BANK0
  endm

DELAY macro TIME
  movlw TIME					; load the time entered
  movwf delaycounter				; place it in the counter
  SET_OPTION_REG 0xD7				; highest prescalar ((f/4)/256)
  bsf DELAY_ISR					; say we want the delay ISR
  clrf TMR0					; clear TMR0
  bsf INTCON, T0IE				; enable TMR0 interrupt
  clrw						; clear W
  subwf delaycounter,f				; test for 0
  btfss STATUS, Z				; are we @ 0 yet?
    goto $-4					; if not, keep testing
  bcf INTCON, T0IE				; disable TMR0 interrupt
  bcf DELAY_ISR					; we don't need the delay ISR anymore
  endm

SPI_SEND macro DATA
  movlw DATA
  movwf spi_byte				; load spi byte
  call SPI_Xfer					; send...
  endm

SPI_BIT_OUT macro BITNUM
  bsf SPI_DATA					; assume HIGH
  btfss spi_byte,BITNUM				; if not..
    bcf SPI_DATA				; fix it...
  endm

LCD_OUT_SPLASH_SCREEN macro
  SPI_SEND 12					; clear display, reset cursor
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND 'M'
  SPI_SEND 'r'
  SPI_SEND '.'
  SPI_SEND ' '
  SPI_SEND 'B'
  SPI_SEND 'a'
  SPI_SEND 'y'
  SPI_SEND 'B'
  SPI_SEND 'u'
  SPI_SEND 's'
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND 'F'
  SPI_SEND 'a'
  SPI_SEND 'n'
  SPI_SEND ' '
  SPI_SEND 'S'
  SPI_SEND 'y'
  SPI_SEND 's'
  SPI_SEND 't'
  SPI_SEND 'e'
  SPI_SEND 'm'
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND 'v'
  SPI_SEND '1'
  SPI_SEND '.'
  SPI_SEND '0'
  endm

;----------------------------------------------------------------------------------------
; Program code
;----------------------------------------------------------------------------------------
  PAGE

 org     0
  bsf SPI_CS					; set SPI Chip Select HIGH initially
  bsf SPI_CLK					; set SPI Clock HIGH initially
  call Initialize				; initialize stuff
  goto   Main					; jump to main program code

 org     4
ISR						; Interrupt Service Routine
  btfss DELAY_ISR				; is it the Delay ISR?
    goto ISR_Timers_2				; if not, skip to next test
ISR_Timers_Delay				; delay timer ISR
  decf delaycounter, f				; delay cycle completed, counter--
  bcf INTCON, T0IF				; handle timer interrupt flag
  goto ISR_end					; goto end of ISR
ISR_Timers_2
  btfss SPI_ISR					; is it the SPI ISR?
    goto ISR_Timers_3				; if not, skip to next test
  movf spi_state, w				; load the current state
  incf spi_state, f				; increment state
  BANK1						; switch to BANK 1
  addwf PCL,f					; advance the PC to the correct state
  goto case_cs_low				; state 0
  goto case_bit_7				; state 1
  goto case_clock_low				; state 2
  goto case_clock_high				; state 3
  goto case_bit_6				; state 4
  goto case_clock_low				; state 5
  goto case_clock_high				; state 6
  goto case_bit_5				; state 7
  goto case_clock_low				; state 8
  goto case_clock_high				; state 9
  goto case_bit_4				; state 10
  goto case_clock_low				; state 11
  goto case_clock_high				; state 12
  goto case_bit_3				; state 13
  goto case_clock_low				; state 14
  goto case_clock_high				; state 15
  goto case_bit_2				; state 16
  goto case_clock_low				; state 17
  goto case_clock_high				; state 18
  goto case_bit_1				; state 19
  goto case_clock_low				; state 20
  goto case_clock_high				; state 21
  goto case_bit_0				; state 22
  goto case_clock_low				; state 23
  goto case_clock_high				; state 24
  BANK0
  bsf SPI_DONE					; state 25, we're done
  bsf SPI_CS					; raise the chip select
  goto ISR_SPI_end
case_cs_low					; initial state, drop the chip select
  BANK0
  bcf SPI_CS
  goto ISR_SPI_end
case_clock_low					
  BANK0
  bcf SPI_CLK
  goto ISR_SPI_end
case_clock_high
  BANK0
  bsf SPI_CLK
  goto ISR_SPI_end
case_bit_7					; bit sending states 7-0 (optimize?)
  BANK0
  SPI_BIT_OUT 7
  goto ISR_SPI_end
case_bit_6
  BANK0
  SPI_BIT_OUT 6
  goto ISR_SPI_end
case_bit_5
  BANK0
  SPI_BIT_OUT 5
  goto ISR_SPI_end
case_bit_4
  BANK0
  SPI_BIT_OUT 4
  goto ISR_SPI_end
case_bit_3
  BANK0
  SPI_BIT_OUT 3
  goto ISR_SPI_end
case_bit_2
  BANK0
  SPI_BIT_OUT 2
  goto ISR_SPI_end
case_bit_1
  BANK0
  SPI_BIT_OUT 1
  goto ISR_SPI_end
case_bit_0
  BANK0
  SPI_BIT_OUT 0
  goto ISR_SPI_end
ISR_SPI_end
  bcf INTCON, T0IF				; handle timer interrupt flag
  goto ISR_end					; get outta here
ISR_Timers_3
  goto ISR_end
ISR_end
  retfie					; return from interrupt

;----------------------------------------------------------------------------------------
; User function definitions
;----------------------------------------------------------------------------------------
Initialize					; Initialization function
  BANK1						; switch to BANK 1
  movlw 0xF8					; W <- 1111 1000
  movwf TRISA ^ 0x80				; PORTA[4:3] INPUT, PORTA[2:0] OUTPUT
  movlw 0x0F					; W <- 0000 1111
  movwf TRISB ^ 0x80				; PORTB[7:4] OUTPUT, PORTB[3:0] INPUT
  BANK0						; switch back to BANK 0
  clrf mode					; reset mode to 0
  clrf buttonflags				; clear out change flags
  READ_DATA FAN_ADDR				; load previous fan settings
  movwf fanstatus				; set it.
  btfss F1_STATUS				; set fan 1?
    goto F1_DISABLE				; if not, disable it
  bsf FAN_1					; turn it on
  goto F2_Inittest				; done
F1_DISABLE
  bcf FAN_1					; turn it off
F2_Inittest
  btfss F2_STATUS				; set fan 2?
    goto F2_DISABLE				; if not, disable it
  bsf FAN_2					; turn it on
  goto F3_Inittest				; done
F2_DISABLE
  bcf FAN_2					; turn it off
F3_Inittest
  btfss F3_STATUS				; set fan 3?
    goto F3_DISABLE				; if not, disable it
  bsf FAN_3					; turn it on
  goto F4_Inittest				; done
F3_DISABLE
  bcf FAN_3					; turn it off
F4_Inittest
  btfss F4_STATUS				; set fan 4?
    goto F4_DISABLE				; if not, disable it
  bsf FAN_4					; turn it on
  goto Done_Inittest				; done
F4_DISABLE
  bcf FAN_4					; turn it off
Done_Inittest
  bsf INTCON, GIE				; enable global interrupts
  DELAY _1					; delay to let LCD startup
  SPI_SEND 14					; backlight opcode
  READ_DATA BACKLIGHT_ADDR			; load previous backlight setting
  movwf backlight				; put in backlight variable
  movwf spi_byte				; load function variable
  call SPI_Xfer					; send spi_byte (backlight) to LCD
  SPI_SEND 15					; contrast opcode
  READ_DATA CONTRAST_ADDR			; load previous contrast setting
  movwf contrast				; put in contrast variable
  movwf spi_byte				; load function variable
  call SPI_Xfer					; send spi_byte (contrast) to LCD
  SPI_SEND 4					; NO CURSOR
  SPI_SEND 20					; SCROLL OFF
  return					; return to calling code

SPI_Xfer					; SPI Transfer function
  SET_OPTION_REG 0xD0				; no prescalar
  bcf SPI_DONE					; clear completion flag
  bsf SPI_ISR					; signify that we need the SPI ISR
  clrf spi_state				; reset state machine
  movlw 172					; TMR0 offset
  movwf TMR0					; load it up...
  bsf INTCON, T0IE				; enable timer interrupt
  btfss SPI_DONE				; are we done yet?
    goto $-1					; if not, loop til we are!
  bcf INTCON, T0IE				; disable timer interrupt
  bcf SPI_ISR					; done with the ISR for us...
  return					; return to calling code

BUTTONPRESS_1					; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_1				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_1				; wait for release
    goto $-1
  movlw 0x01					; load 0000 0001
  xorwf fanstatus,f				; toggle fanstatus bit
  movf fanstatus,w
  movwf eeprom_data				; load data
  WRITE_DATA FAN_ADDR				; save fan status
  call FAN_Update				; update the screen
  btfss F1_STATUS				; set fan 1?
    goto F1_DISABLEb				; if not, disable it
  bsf FAN_1					; turn it on
  return					; done
F1_DISABLEb
  bcf FAN_1					; turn it off
  return					; done

BUTTONPRESS_2					; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_2				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_2				; wait for release
    goto $-1
  movlw 0x02					; load 0000 0010
  xorwf fanstatus,f				; toggle fanstatus bit
  movf fanstatus,w
  movwf eeprom_data				; load data
  WRITE_DATA FAN_ADDR				; save fan status
  call FAN_Update				; update the screen
  btfss F2_STATUS				; set fan 2?
    goto F2_DISABLEb				; if not, disable it
  bsf FAN_2					; turn it on
  return					; done
F2_DISABLEb
  bcf FAN_2					; turn it off
  return					; done

BUTTONPRESS_3					; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_3				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_3				; wait for release
    goto $-1
  movlw 0x04					; load 0000 0100
  xorwf fanstatus,f				; toggle fanstatus bit
  movf fanstatus,w
  movwf eeprom_data				; load data
  WRITE_DATA FAN_ADDR				; save fan status
  call FAN_Update				; update the screen
  btfss F3_STATUS				; set fan 3?
    goto F3_DISABLEb				; if not, disable it
  bsf FAN_3					; turn it on
  return					; done
F3_DISABLEb
  bcf FAN_3					; turn it off
  return					; done

BUTTONPRESS_4					; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_4				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_4				; wait for release
    goto $-1  
  movlw 0x08					; load 0000 1000
  xorwf fanstatus,f				; toggle fanstatus bit
  movf fanstatus,w
  movwf eeprom_data				; load data
  WRITE_DATA FAN_ADDR				; save fan status
  call FAN_Update				; update the screen
  btfss F4_STATUS				; set fan 4?
    goto F4_DISABLEb				; if not, disable it
  bsf FAN_4					; turn it on
  return					; done
F4_DISABLEb
  bcf FAN_4					; turn it off
  return					; done

OPT_BL_DN					; decrease backlight level
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_1				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_1				; wait for release
    goto $-1
  clrw						; load 0
  subwf backlight,w				; test 0
  btfsc STATUS,Z				; are we @ 0?
    return					; if so, do nothin...
  movlw 10					; load 10
  subwf backlight,f				; backlight = backlight - 10
  SPI_SEND 14					; BACKLIGHT
  movf backlight,w				; load backlight for execution
  movwf spi_byte				; assign to function variable
  call SPI_Xfer					; spit out the value
  call OPTIONS_Update				; update the screen
  return

OPT_BL_UP					; increase backlight level
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_2				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_2				; wait for release
    goto $-1
  movlw 100					; load 100
  subwf backlight,w				; test 100
  btfsc STATUS,Z				; are we @ 100?
    return					; if so, do nothin...
  movlw 10					; load 10
  addwf backlight,f				; backlight = backlight + 10
  SPI_SEND 14					; BACKLIGHT
  movf backlight,w				; load backlight for execution
  movwf spi_byte				; assign to function variable
  call SPI_Xfer					; spit out the value
  call OPTIONS_Update				; update the screen
  return

OPT_CT_DN
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_3				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_3				; wait for release
    goto $-1
  clrw						; load 0
  subwf contrast,w				; test 0
  btfsc STATUS,Z				; are we @ 0?
    return					; if so, do nothin...
  movlw 10					; load 10
  subwf contrast,f				; contrast = contrast - 10
  SPI_SEND 15					; CONTRAST
  movf contrast,w				; load contrast for execution
  movwf spi_byte				; assign to function variable
  call SPI_Xfer					; spit out the value
  call OPTIONS_Update				; update the screen
  return

OPT_CT_UP
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_4				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_4				; wait for release
    goto $-1
  movlw 100					; load 100
  subwf contrast,w				; test 100
  btfsc STATUS,Z				; are we @ 0?
    return					; if so, do nothin...
  movlw 10					; load 10
  addwf contrast,f				; contrast = contrast + 10
  SPI_SEND 15					; CONTRAST
  movf contrast,w				; load contrast for execution
  movwf spi_byte				; assign to function variable
  call SPI_Xfer					; spit out the value
  call OPTIONS_Update				; update the screen
  return

BUTTONPRESS_OPTIONS				; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_OPTIONS				; are we still pushing?
    return					; if not, get out...
  movlw 0x01					; load 0000 0001
  xorwf buttonflags,f				; toggle CHANGE_OPTIONS
  btfss BUTTON_OPTIONS				; wait for release
    goto $-1
  call OPTIONS_Update				; update the display
OPTION_LOOP					; now we're in the option mode
  btfss BUTTON_1
    call OPT_BL_DN				; backlight--
  btfss BUTTON_2
    call OPT_BL_UP				; backlight++
  btfss BUTTON_3
    call OPT_CT_DN				; contrast--
  btfss BUTTON_4
    call OPT_CT_UP				; contrast++
  btfss BUTTON_OPTIONS
    goto END_OPTIONS				; done
  goto OPTION_LOOP				; keep watching for keypress
END_OPTIONS
  movf backlight,w
  movwf eeprom_data				; load variable
  WRITE_DATA BACKLIGHT_ADDR			; write backlight to EEPROM
  movf contrast,w
  movwf eeprom_data				; load variable
  WRITE_DATA CONTRAST_ADDR			; write contrast to EEPROM
  call FAN_Update
  return

FAN_OUT_ON
  SPI_SEND 'O'
  SPI_SEND 'n'
  SPI_SEND ' '
  SPI_SEND ' '
  return

FAN_OUT_OFF
  SPI_SEND 'O'
  SPI_SEND 'f'
  SPI_SEND 'f'
  SPI_SEND ' '
  return

FAN_Update
  SPI_SEND 12					; clear display
  SPI_SEND ' '
  SPI_SEND '1'
  SPI_SEND ':'
  btfss F1_STATUS
    goto F1_OFF
F1_ON
  call FAN_OUT_ON
  goto F2_Test
F1_OFF
  call FAN_OUT_OFF
F2_Test
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND '2'
  SPI_SEND ':'
  btfss F2_STATUS
    goto F2_OFF
F2_ON
  call FAN_OUT_ON
  goto F3_Test
F2_OFF
  call FAN_OUT_OFF
F3_Test
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND '3'
  SPI_SEND ':'
  btfss F3_STATUS
    goto F3_OFF
F3_ON
  call FAN_OUT_ON
  goto F4_Test
F3_OFF
  call FAN_OUT_OFF
F4_Test
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND '4'
  SPI_SEND ':'
  btfss F4_STATUS
    goto F4_OFF
F4_ON
  call FAN_OUT_ON
  return
F4_OFF
  call FAN_OUT_OFF
  return

OPTIONS_Update
  SPI_SEND 12					; clear screen
B_Start
  SPI_SEND 'B'
  SPI_SEND 'a'
  SPI_SEND 'c'
  SPI_SEND 'k'
  SPI_SEND 'l'
  SPI_SEND 'i'
  SPI_SEND 'g'
  SPI_SEND 'h'
  SPI_SEND 't'
  SPI_SEND ':'
  SPI_SEND ' '
  movlw 100
  subwf backlight,w
  btfss STATUS,C
    goto B_80
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  goto C_Start
B_80
  movlw 80
  subwf backlight,w
  btfss STATUS,C
    goto B_60
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  goto C_Start
B_60
  movlw 60
  subwf backlight,w
  btfss STATUS,C
    goto B_40
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  goto C_Start
B_40
  movlw 40
  subwf backlight,w
  btfss STATUS,C
    goto B_20
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  goto C_Start
B_20
  movlw 20
  subwf backlight,w
  btfss STATUS,C
    goto B_0
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  goto C_Start
B_0
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
C_Start
  SPI_SEND ' '
  SPI_SEND 'C'
  SPI_SEND 'o'
  SPI_SEND 'n'
  SPI_SEND 't'
  SPI_SEND 'r'
  SPI_SEND 'a'
  SPI_SEND 's'
  SPI_SEND 't'
  SPI_SEND ':'
  SPI_SEND ' '
  movlw 100
  subwf contrast,w
  btfss STATUS,C
    goto C_80
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  goto END_OPTIONS_Update
C_80
  movlw 80
  subwf contrast,w
  btfss STATUS,C
    goto C_60
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  goto END_OPTIONS_Update
C_60
  movlw 60
  subwf contrast,w
  btfss STATUS,C
    goto C_40
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  goto END_OPTIONS_Update
C_40
  movlw 40
  subwf contrast,w
  btfss STATUS,C
    goto C_20
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  goto END_OPTIONS_Update
C_20
  movlw 20
  subwf contrast,w
  btfss STATUS,C
    goto C_0
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  goto END_OPTIONS_Update
C_0
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
END_OPTIONS_Update
  return					; done

BAR_BLANK
  SPI_SEND ' '
  return

BAR_SOLID
  SPI_SEND 255
  return

EEPROM_read
  movf eeprom_address,w	
  movwf EEADR					; load address to read
  BANK1
  bsf EECON1 ^ 0x80,RD				; initiate read
  BANK0
  movf EEDATA,w					; retrieve data
  return

EEPROM_write
  movf eeprom_address,w
  movwf EEADR					; load address to write to
  movf eeprom_data,w
  movwf EEDATA					; load data to write to EEPROM
  BANK1
  bsf EECON1 ^ 0x80,WREN			; enable writing
  movlw 0x55					; send
  movwf EECON2 ^ 0x80				; required
  movlw 0xAA					; EEPROM
  movwf EECON2 ^ 0x80				; crap
  bsf EECON1 ^ 0x80,WR				; initiate the write
  bcf EECON1 ^ 0x80,WREN			; disable writing
  btfss EECON1 ^ 0x80,EEIF			; wait for completion
    goto $-1
  bcf EECON1 ^ 0x80,EEIF			; clear flag
  BANK0
  return

;----------------------------------------------------------------------------------------
; Main program code
;----------------------------------------------------------------------------------------
Main
  LCD_OUT_SPLASH_SCREEN				; initial splash screen
  DELAY _3
  call FAN_Update				; start with fan display

Mainloop
  btfss BUTTON_1
    call BUTTONPRESS_1
  btfss BUTTON_2
    call BUTTONPRESS_2
  btfss BUTTON_3
    call BUTTONPRESS_3
  btfss BUTTON_4
    call BUTTONPRESS_4
  btfss BUTTON_OPTIONS
    call BUTTONPRESS_OPTIONS
  goto Mainloop

 end

  LIST R=DEC
  INCLUDE "p16f84.inc"
 __CONFIG _CP_OFF & _WDT_OFF & _XT_OSC & _PWRTE_ON

;----------------------------------------------------------------------------------------
; Variable declarations
;----------------------------------------------------------------------------------------
 CBLOCK 0x0C
_w, _status					; interrupt push variables
spi_flag, spi_byte, spi_state			; spi transfer variables
delayvar, delaycounter				; delay routine variables
mode, buttonflags				; main program variables
backlight					; backlight setting
contrast					; contrast setting
fanstatus					; fan status flags
eeprom_address, eeprom_data			; EEPROM function variables
 ENDC

;----------------------------------------------------------------------------------------
; #define's
;----------------------------------------------------------------------------------------
#define SPI_ISR  spi_flag,1			; spi transfer signifier
#define SPI_DONE spi_flag,0			; spi transfer complete flag
#define SPI_CS   PORTA,0			; spi chip select line
#define SPI_CLK  PORTA,1			; spi clock line
#define SPI_DATA PORTA,2			; spi data line
#define DELAY_ISR delayvar,0			; delay routine signifier
#define _DEBOUNCE 2				; 131ms debounce delay time 
#define _STARTUP 7				; LCD startup delay
#define _1 15					; 1 second
#define _2 31					; 2 seconds
#define _3 46					; 3 seconds
#define _4 61					; 4 seconds
#define _5 76					; 5 seconds
#define BUTTON_1 PORTB,0			; 1 button
#define BUTTON_2 PORTB,1			; 2 button
#define BUTTON_3 PORTB,2			; 3 button
#define BUTTON_4 PORTB,3			; 4 button
#define FAN_1	 PORTB,4			; fan 1 line
#define FAN_2	 PORTB,5			; fan 2 line
#define FAN_3	 PORTB,6			; fan 3 line
#define FAN_4	 PORTB,7			; fan 4 line
#define BUTTON_OPTIONS PORTA,3			; options toggle button
#define CHANGE_OPTIONS buttonflags,0		; options toggle indicator
#define F1_STATUS fanstatus,0			; fan 1 status
#define F2_STATUS fanstatus,1			; fan 2 status
#define F3_STATUS fanstatus,2			; fan 3 status
#define F4_STATUS fanstatus,3			; fan 4 status
#define BACKLIGHT_ADDR 	0x00			; EEPROM address for backlight
#define CONTRAST_ADDR 	0x01			; EEPROM address for contrast
#define FAN_ADDR 	0x02			; EEPROM address for fan config

;----------------------------------------------------------------------------------------
; Macro declarations
;----------------------------------------------------------------------------------------
BANK0 macro
  bcf STATUS, RP0           			; Goto Bank 0
  endm

BANK1 macro
  bsf STATUS, RP0           			; Goto Bank 1
  endm

READ_DATA macro ADDRESS
  movlw ADDRESS
  movwf eeprom_address
  call EEPROM_read
  endm

WRITE_DATA macro ADDRESS
  movlw ADDRESS
  movwf eeprom_address
  call EEPROM_write
  endm

SET_OPTION_REG macro DATA
  BANK1
  movlw DATA
  movwf OPTION_REG ^ 0x80
  BANK0
  endm

DELAY macro TIME
  movlw TIME					; load the time entered
  movwf delaycounter				; place it in the counter
  SET_OPTION_REG 0xD7				; highest prescalar ((f/4)/256)
  bsf DELAY_ISR					; say we want the delay ISR
  clrf TMR0					; clear TMR0
  bsf INTCON, T0IE				; enable TMR0 interrupt
  clrw						; clear W
  subwf delaycounter,f				; test for 0
  btfss STATUS, Z				; are we @ 0 yet?
    goto $-4					; if not, keep testing
  bcf INTCON, T0IE				; disable TMR0 interrupt
  bcf DELAY_ISR					; we don't need the delay ISR anymore
  endm

SPI_SEND macro DATA
  movlw DATA
  movwf spi_byte				; load spi byte
  call SPI_Xfer					; send...
  endm

SPI_BIT_OUT macro BITNUM
  bsf SPI_DATA					; assume HIGH
  btfss spi_byte,BITNUM				; if not..
    bcf SPI_DATA				; fix it...
  endm

LCD_OUT_SPLASH_SCREEN macro
  SPI_SEND 12					; clear display, reset cursor
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND 'M'
  SPI_SEND 'r'
  SPI_SEND '.'
  SPI_SEND ' '
  SPI_SEND 'B'
  SPI_SEND 'a'
  SPI_SEND 'y'
  SPI_SEND 'B'
  SPI_SEND 'u'
  SPI_SEND 's'
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND 'F'
  SPI_SEND 'a'
  SPI_SEND 'n'
  SPI_SEND ' '
  SPI_SEND 'S'
  SPI_SEND 'y'
  SPI_SEND 's'
  SPI_SEND 't'
  SPI_SEND 'e'
  SPI_SEND 'm'
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND 'v'
  SPI_SEND '1'
  SPI_SEND '.'
  SPI_SEND '0'
  endm

;----------------------------------------------------------------------------------------
; Program code
;----------------------------------------------------------------------------------------
  PAGE

 org     0
  bsf SPI_CS					; set SPI Chip Select HIGH initially
  bsf SPI_CLK					; set SPI Clock HIGH initially
  call Initialize				; initialize stuff
  goto   Main					; jump to main program code

 org     4
ISR						; Interrupt Service Routine
  btfss DELAY_ISR				; is it the Delay ISR?
    goto ISR_Timers_2				; if not, skip to next test
ISR_Timers_Delay				; delay timer ISR
  decf delaycounter, f				; delay cycle completed, counter--
  bcf INTCON, T0IF				; handle timer interrupt flag
  goto ISR_end					; goto end of ISR
ISR_Timers_2
  btfss SPI_ISR					; is it the SPI ISR?
    goto ISR_Timers_3				; if not, skip to next test
  movf spi_state, w				; load the current state
  incf spi_state, f				; increment state
  BANK1						; switch to BANK 1
  addwf PCL,f					; advance the PC to the correct state
  goto case_cs_low				; state 0
  goto case_bit_7				; state 1
  goto case_clock_low				; state 2
  goto case_clock_high				; state 3
  goto case_bit_6				; state 4
  goto case_clock_low				; state 5
  goto case_clock_high				; state 6
  goto case_bit_5				; state 7
  goto case_clock_low				; state 8
  goto case_clock_high				; state 9
  goto case_bit_4				; state 10
  goto case_clock_low				; state 11
  goto case_clock_high				; state 12
  goto case_bit_3				; state 13
  goto case_clock_low				; state 14
  goto case_clock_high				; state 15
  goto case_bit_2				; state 16
  goto case_clock_low				; state 17
  goto case_clock_high				; state 18
  goto case_bit_1				; state 19
  goto case_clock_low				; state 20
  goto case_clock_high				; state 21
  goto case_bit_0				; state 22
  goto case_clock_low				; state 23
  goto case_clock_high				; state 24
  BANK0
  bsf SPI_DONE					; state 25, we're done
  bsf SPI_CS					; raise the chip select
  goto ISR_SPI_end
case_cs_low					; initial state, drop the chip select
  BANK0
  bcf SPI_CS
  goto ISR_SPI_end
case_clock_low					
  BANK0
  bcf SPI_CLK
  goto ISR_SPI_end
case_clock_high
  BANK0
  bsf SPI_CLK
  goto ISR_SPI_end
case_bit_7					; bit sending states 7-0 (optimize?)
  BANK0
  SPI_BIT_OUT 7
  goto ISR_SPI_end
case_bit_6
  BANK0
  SPI_BIT_OUT 6
  goto ISR_SPI_end
case_bit_5
  BANK0
  SPI_BIT_OUT 5
  goto ISR_SPI_end
case_bit_4
  BANK0
  SPI_BIT_OUT 4
  goto ISR_SPI_end
case_bit_3
  BANK0
  SPI_BIT_OUT 3
  goto ISR_SPI_end
case_bit_2
  BANK0
  SPI_BIT_OUT 2
  goto ISR_SPI_end
case_bit_1
  BANK0
  SPI_BIT_OUT 1
  goto ISR_SPI_end
case_bit_0
  BANK0
  SPI_BIT_OUT 0
  goto ISR_SPI_end
ISR_SPI_end
  bcf INTCON, T0IF				; handle timer interrupt flag
  goto ISR_end					; get outta here
ISR_Timers_3
  goto ISR_end
ISR_end
  retfie					; return from interrupt

;----------------------------------------------------------------------------------------
; User function definitions
;----------------------------------------------------------------------------------------
Initialize					; Initialization function
  BANK1						; switch to BANK 1
  movlw 0xF8					; W <- 1111 1000
  movwf TRISA ^ 0x80				; PORTA[4:3] INPUT, PORTA[2:0] OUTPUT
  movlw 0x0F					; W <- 0000 1111
  movwf TRISB ^ 0x80				; PORTB[7:4] OUTPUT, PORTB[3:0] INPUT
  BANK0						; switch back to BANK 0
  clrf mode					; reset mode to 0
  clrf buttonflags				; clear out change flags
  READ_DATA FAN_ADDR				; load previous fan settings
  movwf fanstatus				; set it.
  btfss F1_STATUS				; set fan 1?
    goto F1_DISABLE				; if not, disable it
  bsf FAN_1					; turn it on
  goto F2_Inittest				; done
F1_DISABLE
  bcf FAN_1					; turn it off
F2_Inittest
  btfss F2_STATUS				; set fan 2?
    goto F2_DISABLE				; if not, disable it
  bsf FAN_2					; turn it on
  goto F3_Inittest				; done
F2_DISABLE
  bcf FAN_2					; turn it off
F3_Inittest
  btfss F3_STATUS				; set fan 3?
    goto F3_DISABLE				; if not, disable it
  bsf FAN_3					; turn it on
  goto F4_Inittest				; done
F3_DISABLE
  bcf FAN_3					; turn it off
F4_Inittest
  btfss F4_STATUS				; set fan 4?
    goto F4_DISABLE				; if not, disable it
  bsf FAN_4					; turn it on
  goto Done_Inittest				; done
F4_DISABLE
  bcf FAN_4					; turn it off
Done_Inittest
  bsf INTCON, GIE				; enable global interrupts
  DELAY _1					; delay to let LCD startup
  SPI_SEND 14					; backlight opcode
  READ_DATA BACKLIGHT_ADDR			; load previous backlight setting
  movwf backlight				; put in backlight variable
  movwf spi_byte				; load function variable
  call SPI_Xfer					; send spi_byte (backlight) to LCD
  SPI_SEND 15					; contrast opcode
  READ_DATA CONTRAST_ADDR			; load previous contrast setting
  movwf contrast				; put in contrast variable
  movwf spi_byte				; load function variable
  call SPI_Xfer					; send spi_byte (contrast) to LCD
  SPI_SEND 4					; NO CURSOR
  SPI_SEND 20					; SCROLL OFF
  return					; return to calling code

SPI_Xfer					; SPI Transfer function
  SET_OPTION_REG 0xD0				; no prescalar
  bcf SPI_DONE					; clear completion flag
  bsf SPI_ISR					; signify that we need the SPI ISR
  clrf spi_state				; reset state machine
  movlw 172					; TMR0 offset
  movwf TMR0					; load it up...
  bsf INTCON, T0IE				; enable timer interrupt
  btfss SPI_DONE				; are we done yet?
    goto $-1					; if not, loop til we are!
  bcf INTCON, T0IE				; disable timer interrupt
  bcf SPI_ISR					; done with the ISR for us...
  return					; return to calling code

BUTTONPRESS_1					; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_1				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_1				; wait for release
    goto $-1
  movlw 0x01					; load 0000 0001
  xorwf fanstatus,f				; toggle fanstatus bit
  movf fanstatus,w
  movwf eeprom_data				; load data
  WRITE_DATA FAN_ADDR				; save fan status
  call FAN_Update				; update the screen
  btfss F1_STATUS				; set fan 1?
    goto F1_DISABLEb				; if not, disable it
  bsf FAN_1					; turn it on
  return					; done
F1_DISABLEb
  bcf FAN_1					; turn it off
  return					; done

BUTTONPRESS_2					; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_2				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_2				; wait for release
    goto $-1
  movlw 0x02					; load 0000 0010
  xorwf fanstatus,f				; toggle fanstatus bit
  movf fanstatus,w
  movwf eeprom_data				; load data
  WRITE_DATA FAN_ADDR				; save fan status
  call FAN_Update				; update the screen
  btfss F2_STATUS				; set fan 2?
    goto F2_DISABLEb				; if not, disable it
  bsf FAN_2					; turn it on
  return					; done
F2_DISABLEb
  bcf FAN_2					; turn it off
  return					; done

BUTTONPRESS_3					; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_3				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_3				; wait for release
    goto $-1
  movlw 0x04					; load 0000 0100
  xorwf fanstatus,f				; toggle fanstatus bit
  movf fanstatus,w
  movwf eeprom_data				; load data
  WRITE_DATA FAN_ADDR				; save fan status
  call FAN_Update				; update the screen
  btfss F3_STATUS				; set fan 3?
    goto F3_DISABLEb				; if not, disable it
  bsf FAN_3					; turn it on
  return					; done
F3_DISABLEb
  bcf FAN_3					; turn it off
  return					; done

BUTTONPRESS_4					; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_4				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_4				; wait for release
    goto $-1  
  movlw 0x08					; load 0000 1000
  xorwf fanstatus,f				; toggle fanstatus bit
  movf fanstatus,w
  movwf eeprom_data				; load data
  WRITE_DATA FAN_ADDR				; save fan status
  call FAN_Update				; update the screen
  btfss F4_STATUS				; set fan 4?
    goto F4_DISABLEb				; if not, disable it
  bsf FAN_4					; turn it on
  return					; done
F4_DISABLEb
  bcf FAN_4					; turn it off
  return					; done

OPT_BL_DN					; decrease backlight level
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_1				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_1				; wait for release
    goto $-1
  clrw						; load 0
  subwf backlight,w				; test 0
  btfsc STATUS,Z				; are we @ 0?
    return					; if so, do nothin...
  movlw 10					; load 10
  subwf backlight,f				; backlight = backlight - 10
  SPI_SEND 14					; BACKLIGHT
  movf backlight,w				; load backlight for execution
  movwf spi_byte				; assign to function variable
  call SPI_Xfer					; spit out the value
  call OPTIONS_Update				; update the screen
  return

OPT_BL_UP					; increase backlight level
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_2				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_2				; wait for release
    goto $-1
  movlw 100					; load 100
  subwf backlight,w				; test 100
  btfsc STATUS,Z				; are we @ 100?
    return					; if so, do nothin...
  movlw 10					; load 10
  addwf backlight,f				; backlight = backlight + 10
  SPI_SEND 14					; BACKLIGHT
  movf backlight,w				; load backlight for execution
  movwf spi_byte				; assign to function variable
  call SPI_Xfer					; spit out the value
  call OPTIONS_Update				; update the screen
  return

OPT_CT_DN
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_3				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_3				; wait for release
    goto $-1
  clrw						; load 0
  subwf contrast,w				; test 0
  btfsc STATUS,Z				; are we @ 0?
    return					; if so, do nothin...
  movlw 10					; load 10
  subwf contrast,f				; contrast = contrast - 10
  SPI_SEND 15					; CONTRAST
  movf contrast,w				; load contrast for execution
  movwf spi_byte				; assign to function variable
  call SPI_Xfer					; spit out the value
  call OPTIONS_Update				; update the screen
  return

OPT_CT_UP
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_4				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_4				; wait for release
    goto $-1
  movlw 100					; load 100
  subwf contrast,w				; test 100
  btfsc STATUS,Z				; are we @ 0?
    return					; if so, do nothin...
  movlw 10					; load 10
  addwf contrast,f				; contrast = contrast + 10
  SPI_SEND 15					; CONTRAST
  movf contrast,w				; load contrast for execution
  movwf spi_byte				; assign to function variable
  call SPI_Xfer					; spit out the value
  call OPTIONS_Update				; update the screen
  return

BUTTONPRESS_OPTIONS				; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_OPTIONS				; are we still pushing?
    return					; if not, get out...
  movlw 0x01					; load 0000 0001
  xorwf buttonflags,f				; toggle CHANGE_OPTIONS
  btfss BUTTON_OPTIONS				; wait for release
    goto $-1
  call OPTIONS_Update				; update the display
OPTION_LOOP					; now we're in the option mode
  btfss BUTTON_1
    call OPT_BL_DN				; backlight--
  btfss BUTTON_2
    call OPT_BL_UP				; backlight++
  btfss BUTTON_3
    call OPT_CT_DN				; contrast--
  btfss BUTTON_4
    call OPT_CT_UP				; contrast++
  btfss BUTTON_OPTIONS
    goto END_OPTIONS				; done
  goto OPTION_LOOP				; keep watching for keypress
END_OPTIONS
  movf backlight,w
  movwf eeprom_data				; load variable
  WRITE_DATA BACKLIGHT_ADDR			; write backlight to EEPROM
  movf contrast,w
  movwf eeprom_data				; load variable
  WRITE_DATA CONTRAST_ADDR			; write contrast to EEPROM
  call FAN_Update
  return

FAN_OUT_ON
  SPI_SEND 'O'
  SPI_SEND 'n'
  SPI_SEND ' '
  SPI_SEND ' '
  return

FAN_OUT_OFF
  SPI_SEND 'O'
  SPI_SEND 'f'
  SPI_SEND 'f'
  SPI_SEND ' '
  return

FAN_Update
  SPI_SEND 12					; clear display
  SPI_SEND ' '
  SPI_SEND '1'
  SPI_SEND ':'
  btfss F1_STATUS
    goto F1_OFF
F1_ON
  call FAN_OUT_ON
  goto F2_Test
F1_OFF
  call FAN_OUT_OFF
F2_Test
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND '2'
  SPI_SEND ':'
  btfss F2_STATUS
    goto F2_OFF
F2_ON
  call FAN_OUT_ON
  goto F3_Test
F2_OFF
  call FAN_OUT_OFF
F3_Test
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND '3'
  SPI_SEND ':'
  btfss F3_STATUS
    goto F3_OFF
F3_ON
  call FAN_OUT_ON
  goto F4_Test
F3_OFF
  call FAN_OUT_OFF
F4_Test
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND '4'
  SPI_SEND ':'
  btfss F4_STATUS
    goto F4_OFF
F4_ON
  call FAN_OUT_ON
  return
F4_OFF
  call FAN_OUT_OFF
  return

OPTIONS_Update
  SPI_SEND 12					; clear screen
B_Start
  SPI_SEND 'B'
  SPI_SEND 'a'
  SPI_SEND 'c'
  SPI_SEND 'k'
  SPI_SEND 'l'
  SPI_SEND 'i'
  SPI_SEND 'g'
  SPI_SEND 'h'
  SPI_SEND 't'
  SPI_SEND ':'
  SPI_SEND ' '
  movlw 100
  subwf backlight,w
  btfss STATUS,C
    goto B_80
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  goto C_Start
B_80
  movlw 80
  subwf backlight,w
  btfss STATUS,C
    goto B_60
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  goto C_Start
B_60
  movlw 60
  subwf backlight,w
  btfss STATUS,C
    goto B_40
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  goto C_Start
B_40
  movlw 40
  subwf backlight,w
  btfss STATUS,C
    goto B_20
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  goto C_Start
B_20
  movlw 20
  subwf backlight,w
  btfss STATUS,C
    goto B_0
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  goto C_Start
B_0
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
C_Start
  SPI_SEND ' '
  SPI_SEND 'C'
  SPI_SEND 'o'
  SPI_SEND 'n'
  SPI_SEND 't'
  SPI_SEND 'r'
  SPI_SEND 'a'
  SPI_SEND 's'
  SPI_SEND 't'
  SPI_SEND ':'
  SPI_SEND ' '
  movlw 100
  subwf contrast,w
  btfss STATUS,C
    goto C_80
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  goto END_OPTIONS_Update
C_80
  movlw 80
  subwf contrast,w
  btfss STATUS,C
    goto C_60
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  goto END_OPTIONS_Update
C_60
  movlw 60
  subwf contrast,w
  btfss STATUS,C
    goto C_40
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  goto END_OPTIONS_Update
C_40
  movlw 40
  subwf contrast,w
  btfss STATUS,C
    goto C_20
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  goto END_OPTIONS_Update
C_20
  movlw 20
  subwf contrast,w
  btfss STATUS,C
    goto C_0
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  goto END_OPTIONS_Update
C_0
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
END_OPTIONS_Update
  return					; done

BAR_BLANK
  SPI_SEND ' '
  return

BAR_SOLID
  SPI_SEND 255
  return

EEPROM_read
  movf eeprom_address,w	
  movwf EEADR					; load address to read
  BANK1
  bsf EECON1 ^ 0x80,RD				; initiate read
  BANK0
  movf EEDATA,w					; retrieve data
  return

EEPROM_write
  movf eeprom_address,w
  movwf EEADR					; load address to write to
  movf eeprom_data,w
  movwf EEDATA					; load data to write to EEPROM
  BANK1
  bsf EECON1 ^ 0x80,WREN			; enable writing
  movlw 0x55					; send
  movwf EECON2 ^ 0x80				; required
  movlw 0xAA					; EEPROM
  movwf EECON2 ^ 0x80				; crap
  bsf EECON1 ^ 0x80,WR				; initiate the write
  bcf EECON1 ^ 0x80,WREN			; disable writing
  btfss EECON1 ^ 0x80,EEIF			; wait for completion
    goto $-1
  bcf EECON1 ^ 0x80,EEIF			; clear flag
  BANK0
  return

;----------------------------------------------------------------------------------------
; Main program code
;----------------------------------------------------------------------------------------
Main
  LCD_OUT_SPLASH_SCREEN				; initial splash screen
  DELAY _3
  call FAN_Update				; start with fan display

Mainloop
  btfss BUTTON_1
    call BUTTONPRESS_1
  btfss BUTTON_2
    call BUTTONPRESS_2
  btfss BUTTON_3
    call BUTTONPRESS_3
  btfss BUTTON_4
    call BUTTONPRESS_4
  btfss BUTTON_OPTIONS
    call BUTTONPRESS_OPTIONS
  goto Mainloop

 end

  LIST R=DEC
  INCLUDE "p16f84.inc"
 __CONFIG _CP_OFF & _WDT_OFF & _XT_OSC & _PWRTE_ON

;----------------------------------------------------------------------------------------
; Variable declarations
;----------------------------------------------------------------------------------------
 CBLOCK 0x0C
_w, _status					; interrupt push variables
spi_flag, spi_byte, spi_state			; spi transfer variables
delayvar, delaycounter				; delay routine variables
mode, buttonflags				; main program variables
backlight					; backlight setting
contrast					; contrast setting
fanstatus					; fan status flags
eeprom_address, eeprom_data			; EEPROM function variables
 ENDC

;----------------------------------------------------------------------------------------
; #define's
;----------------------------------------------------------------------------------------
#define SPI_ISR  spi_flag,1			; spi transfer signifier
#define SPI_DONE spi_flag,0			; spi transfer complete flag
#define SPI_CS   PORTA,0			; spi chip select line
#define SPI_CLK  PORTA,1			; spi clock line
#define SPI_DATA PORTA,2			; spi data line
#define DELAY_ISR delayvar,0			; delay routine signifier
#define _DEBOUNCE 2				; 131ms debounce delay time 
#define _STARTUP 7				; LCD startup delay
#define _1 15					; 1 second
#define _2 31					; 2 seconds
#define _3 46					; 3 seconds
#define _4 61					; 4 seconds
#define _5 76					; 5 seconds
#define BUTTON_1 PORTB,0			; 1 button
#define BUTTON_2 PORTB,1			; 2 button
#define BUTTON_3 PORTB,2			; 3 button
#define BUTTON_4 PORTB,3			; 4 button
#define FAN_1	 PORTB,4			; fan 1 line
#define FAN_2	 PORTB,5			; fan 2 line
#define FAN_3	 PORTB,6			; fan 3 line
#define FAN_4	 PORTB,7			; fan 4 line
#define BUTTON_OPTIONS PORTA,3			; options toggle button
#define CHANGE_OPTIONS buttonflags,0		; options toggle indicator
#define F1_STATUS fanstatus,0			; fan 1 status
#define F2_STATUS fanstatus,1			; fan 2 status
#define F3_STATUS fanstatus,2			; fan 3 status
#define F4_STATUS fanstatus,3			; fan 4 status
#define BACKLIGHT_ADDR 	0x00			; EEPROM address for backlight
#define CONTRAST_ADDR 	0x01			; EEPROM address for contrast
#define FAN_ADDR 	0x02			; EEPROM address for fan config

;----------------------------------------------------------------------------------------
; Macro declarations
;----------------------------------------------------------------------------------------
BANK0 macro
  bcf STATUS, RP0           			; Goto Bank 0
  endm

BANK1 macro
  bsf STATUS, RP0           			; Goto Bank 1
  endm

READ_DATA macro ADDRESS
  movlw ADDRESS
  movwf eeprom_address
  call EEPROM_read
  endm

WRITE_DATA macro ADDRESS
  movlw ADDRESS
  movwf eeprom_address
  call EEPROM_write
  endm

SET_OPTION_REG macro DATA
  BANK1
  movlw DATA
  movwf OPTION_REG ^ 0x80
  BANK0
  endm

DELAY macro TIME
  movlw TIME					; load the time entered
  movwf delaycounter				; place it in the counter
  SET_OPTION_REG 0xD7				; highest prescalar ((f/4)/256)
  bsf DELAY_ISR					; say we want the delay ISR
  clrf TMR0					; clear TMR0
  bsf INTCON, T0IE				; enable TMR0 interrupt
  clrw						; clear W
  subwf delaycounter,f				; test for 0
  btfss STATUS, Z				; are we @ 0 yet?
    goto $-4					; if not, keep testing
  bcf INTCON, T0IE				; disable TMR0 interrupt
  bcf DELAY_ISR					; we don't need the delay ISR anymore
  endm

SPI_SEND macro DATA
  movlw DATA
  movwf spi_byte				; load spi byte
  call SPI_Xfer					; send...
  endm

SPI_BIT_OUT macro BITNUM
  bsf SPI_DATA					; assume HIGH
  btfss spi_byte,BITNUM				; if not..
    bcf SPI_DATA				; fix it...
  endm

LCD_OUT_SPLASH_SCREEN macro
  SPI_SEND 12					; clear display, reset cursor
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND 'M'
  SPI_SEND 'r'
  SPI_SEND '.'
  SPI_SEND ' '
  SPI_SEND 'B'
  SPI_SEND 'a'
  SPI_SEND 'y'
  SPI_SEND 'B'
  SPI_SEND 'u'
  SPI_SEND 's'
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND 'F'
  SPI_SEND 'a'
  SPI_SEND 'n'
  SPI_SEND ' '
  SPI_SEND 'S'
  SPI_SEND 'y'
  SPI_SEND 's'
  SPI_SEND 't'
  SPI_SEND 'e'
  SPI_SEND 'm'
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND 'v'
  SPI_SEND '1'
  SPI_SEND '.'
  SPI_SEND '0'
  endm

;----------------------------------------------------------------------------------------
; Program code
;----------------------------------------------------------------------------------------
  PAGE

 org     0
  bsf SPI_CS					; set SPI Chip Select HIGH initially
  bsf SPI_CLK					; set SPI Clock HIGH initially
  call Initialize				; initialize stuff
  goto   Main					; jump to main program code

 org     4
ISR						; Interrupt Service Routine
  btfss DELAY_ISR				; is it the Delay ISR?
    goto ISR_Timers_2				; if not, skip to next test
ISR_Timers_Delay				; delay timer ISR
  decf delaycounter, f				; delay cycle completed, counter--
  bcf INTCON, T0IF				; handle timer interrupt flag
  goto ISR_end					; goto end of ISR
ISR_Timers_2
  btfss SPI_ISR					; is it the SPI ISR?
    goto ISR_Timers_3				; if not, skip to next test
  movf spi_state, w				; load the current state
  incf spi_state, f				; increment state
  BANK1						; switch to BANK 1
  addwf PCL,f					; advance the PC to the correct state
  goto case_cs_low				; state 0
  goto case_bit_7				; state 1
  goto case_clock_low				; state 2
  goto case_clock_high				; state 3
  goto case_bit_6				; state 4
  goto case_clock_low				; state 5
  goto case_clock_high				; state 6
  goto case_bit_5				; state 7
  goto case_clock_low				; state 8
  goto case_clock_high				; state 9
  goto case_bit_4				; state 10
  goto case_clock_low				; state 11
  goto case_clock_high				; state 12
  goto case_bit_3				; state 13
  goto case_clock_low				; state 14
  goto case_clock_high				; state 15
  goto case_bit_2				; state 16
  goto case_clock_low				; state 17
  goto case_clock_high				; state 18
  goto case_bit_1				; state 19
  goto case_clock_low				; state 20
  goto case_clock_high				; state 21
  goto case_bit_0				; state 22
  goto case_clock_low				; state 23
  goto case_clock_high				; state 24
  BANK0
  bsf SPI_DONE					; state 25, we're done
  bsf SPI_CS					; raise the chip select
  goto ISR_SPI_end
case_cs_low					; initial state, drop the chip select
  BANK0
  bcf SPI_CS
  goto ISR_SPI_end
case_clock_low					
  BANK0
  bcf SPI_CLK
  goto ISR_SPI_end
case_clock_high
  BANK0
  bsf SPI_CLK
  goto ISR_SPI_end
case_bit_7					; bit sending states 7-0 (optimize?)
  BANK0
  SPI_BIT_OUT 7
  goto ISR_SPI_end
case_bit_6
  BANK0
  SPI_BIT_OUT 6
  goto ISR_SPI_end
case_bit_5
  BANK0
  SPI_BIT_OUT 5
  goto ISR_SPI_end
case_bit_4
  BANK0
  SPI_BIT_OUT 4
  goto ISR_SPI_end
case_bit_3
  BANK0
  SPI_BIT_OUT 3
  goto ISR_SPI_end
case_bit_2
  BANK0
  SPI_BIT_OUT 2
  goto ISR_SPI_end
case_bit_1
  BANK0
  SPI_BIT_OUT 1
  goto ISR_SPI_end
case_bit_0
  BANK0
  SPI_BIT_OUT 0
  goto ISR_SPI_end
ISR_SPI_end
  bcf INTCON, T0IF				; handle timer interrupt flag
  goto ISR_end					; get outta here
ISR_Timers_3
  goto ISR_end
ISR_end
  retfie					; return from interrupt

;----------------------------------------------------------------------------------------
; User function definitions
;----------------------------------------------------------------------------------------
Initialize					; Initialization function
  BANK1						; switch to BANK 1
  movlw 0xF8					; W <- 1111 1000
  movwf TRISA ^ 0x80				; PORTA[4:3] INPUT, PORTA[2:0] OUTPUT
  movlw 0x0F					; W <- 0000 1111
  movwf TRISB ^ 0x80				; PORTB[7:4] OUTPUT, PORTB[3:0] INPUT
  BANK0						; switch back to BANK 0
  clrf mode					; reset mode to 0
  clrf buttonflags				; clear out change flags
  READ_DATA FAN_ADDR				; load previous fan settings
  movwf fanstatus				; set it.
  btfss F1_STATUS				; set fan 1?
    goto F1_DISABLE				; if not, disable it
  bsf FAN_1					; turn it on
  goto F2_Inittest				; done
F1_DISABLE
  bcf FAN_1					; turn it off
F2_Inittest
  btfss F2_STATUS				; set fan 2?
    goto F2_DISABLE				; if not, disable it
  bsf FAN_2					; turn it on
  goto F3_Inittest				; done
F2_DISABLE
  bcf FAN_2					; turn it off
F3_Inittest
  btfss F3_STATUS				; set fan 3?
    goto F3_DISABLE				; if not, disable it
  bsf FAN_3					; turn it on
  goto F4_Inittest				; done
F3_DISABLE
  bcf FAN_3					; turn it off
F4_Inittest
  btfss F4_STATUS				; set fan 4?
    goto F4_DISABLE				; if not, disable it
  bsf FAN_4					; turn it on
  goto Done_Inittest				; done
F4_DISABLE
  bcf FAN_4					; turn it off
Done_Inittest
  bsf INTCON, GIE				; enable global interrupts
  DELAY _1					; delay to let LCD startup
  SPI_SEND 14					; backlight opcode
  READ_DATA BACKLIGHT_ADDR			; load previous backlight setting
  movwf backlight				; put in backlight variable
  movwf spi_byte				; load function variable
  call SPI_Xfer					; send spi_byte (backlight) to LCD
  SPI_SEND 15					; contrast opcode
  READ_DATA CONTRAST_ADDR			; load previous contrast setting
  movwf contrast				; put in contrast variable
  movwf spi_byte				; load function variable
  call SPI_Xfer					; send spi_byte (contrast) to LCD
  SPI_SEND 4					; NO CURSOR
  SPI_SEND 20					; SCROLL OFF
  return					; return to calling code

SPI_Xfer					; SPI Transfer function
  SET_OPTION_REG 0xD0				; no prescalar
  bcf SPI_DONE					; clear completion flag
  bsf SPI_ISR					; signify that we need the SPI ISR
  clrf spi_state				; reset state machine
  movlw 172					; TMR0 offset
  movwf TMR0					; load it up...
  bsf INTCON, T0IE				; enable timer interrupt
  btfss SPI_DONE				; are we done yet?
    goto $-1					; if not, loop til we are!
  bcf INTCON, T0IE				; disable timer interrupt
  bcf SPI_ISR					; done with the ISR for us...
  return					; return to calling code

BUTTONPRESS_1					; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_1				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_1				; wait for release
    goto $-1
  movlw 0x01					; load 0000 0001
  xorwf fanstatus,f				; toggle fanstatus bit
  movf fanstatus,w
  movwf eeprom_data				; load data
  WRITE_DATA FAN_ADDR				; save fan status
  call FAN_Update				; update the screen
  btfss F1_STATUS				; set fan 1?
    goto F1_DISABLEb				; if not, disable it
  bsf FAN_1					; turn it on
  return					; done
F1_DISABLEb
  bcf FAN_1					; turn it off
  return					; done

BUTTONPRESS_2					; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_2				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_2				; wait for release
    goto $-1
  movlw 0x02					; load 0000 0010
  xorwf fanstatus,f				; toggle fanstatus bit
  movf fanstatus,w
  movwf eeprom_data				; load data
  WRITE_DATA FAN_ADDR				; save fan status
  call FAN_Update				; update the screen
  btfss F2_STATUS				; set fan 2?
    goto F2_DISABLEb				; if not, disable it
  bsf FAN_2					; turn it on
  return					; done
F2_DISABLEb
  bcf FAN_2					; turn it off
  return					; done

BUTTONPRESS_3					; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_3				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_3				; wait for release
    goto $-1
  movlw 0x04					; load 0000 0100
  xorwf fanstatus,f				; toggle fanstatus bit
  movf fanstatus,w
  movwf eeprom_data				; load data
  WRITE_DATA FAN_ADDR				; save fan status
  call FAN_Update				; update the screen
  btfss F3_STATUS				; set fan 3?
    goto F3_DISABLEb				; if not, disable it
  bsf FAN_3					; turn it on
  return					; done
F3_DISABLEb
  bcf FAN_3					; turn it off
  return					; done

BUTTONPRESS_4					; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_4				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_4				; wait for release
    goto $-1  
  movlw 0x08					; load 0000 1000
  xorwf fanstatus,f				; toggle fanstatus bit
  movf fanstatus,w
  movwf eeprom_data				; load data
  WRITE_DATA FAN_ADDR				; save fan status
  call FAN_Update				; update the screen
  btfss F4_STATUS				; set fan 4?
    goto F4_DISABLEb				; if not, disable it
  bsf FAN_4					; turn it on
  return					; done
F4_DISABLEb
  bcf FAN_4					; turn it off
  return					; done

OPT_BL_DN					; decrease backlight level
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_1				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_1				; wait for release
    goto $-1
  clrw						; load 0
  subwf backlight,w				; test 0
  btfsc STATUS,Z				; are we @ 0?
    return					; if so, do nothin...
  movlw 10					; load 10
  subwf backlight,f				; backlight = backlight - 10
  SPI_SEND 14					; BACKLIGHT
  movf backlight,w				; load backlight for execution
  movwf spi_byte				; assign to function variable
  call SPI_Xfer					; spit out the value
  call OPTIONS_Update				; update the screen
  return

OPT_BL_UP					; increase backlight level
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_2				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_2				; wait for release
    goto $-1
  movlw 100					; load 100
  subwf backlight,w				; test 100
  btfsc STATUS,Z				; are we @ 100?
    return					; if so, do nothin...
  movlw 10					; load 10
  addwf backlight,f				; backlight = backlight + 10
  SPI_SEND 14					; BACKLIGHT
  movf backlight,w				; load backlight for execution
  movwf spi_byte				; assign to function variable
  call SPI_Xfer					; spit out the value
  call OPTIONS_Update				; update the screen
  return

OPT_CT_DN
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_3				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_3				; wait for release
    goto $-1
  clrw						; load 0
  subwf contrast,w				; test 0
  btfsc STATUS,Z				; are we @ 0?
    return					; if so, do nothin...
  movlw 10					; load 10
  subwf contrast,f				; contrast = contrast - 10
  SPI_SEND 15					; CONTRAST
  movf contrast,w				; load contrast for execution
  movwf spi_byte				; assign to function variable
  call SPI_Xfer					; spit out the value
  call OPTIONS_Update				; update the screen
  return

OPT_CT_UP
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_4				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_4				; wait for release
    goto $-1
  movlw 100					; load 100
  subwf contrast,w				; test 100
  btfsc STATUS,Z				; are we @ 0?
    return					; if so, do nothin...
  movlw 10					; load 10
  addwf contrast,f				; contrast = contrast + 10
  SPI_SEND 15					; CONTRAST
  movf contrast,w				; load contrast for execution
  movwf spi_byte				; assign to function variable
  call SPI_Xfer					; spit out the value
  call OPTIONS_Update				; update the screen
  return

BUTTONPRESS_OPTIONS				; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_OPTIONS				; are we still pushing?
    return					; if not, get out...
  movlw 0x01					; load 0000 0001
  xorwf buttonflags,f				; toggle CHANGE_OPTIONS
  btfss BUTTON_OPTIONS				; wait for release
    goto $-1
  call OPTIONS_Update				; update the display
OPTION_LOOP					; now we're in the option mode
  btfss BUTTON_1
    call OPT_BL_DN				; backlight--
  btfss BUTTON_2
    call OPT_BL_UP				; backlight++
  btfss BUTTON_3
    call OPT_CT_DN				; contrast--
  btfss BUTTON_4
    call OPT_CT_UP				; contrast++
  btfss BUTTON_OPTIONS
    goto END_OPTIONS				; done
  goto OPTION_LOOP				; keep watching for keypress
END_OPTIONS
  movf backlight,w
  movwf eeprom_data				; load variable
  WRITE_DATA BACKLIGHT_ADDR			; write backlight to EEPROM
  movf contrast,w
  movwf eeprom_data				; load variable
  WRITE_DATA CONTRAST_ADDR			; write contrast to EEPROM
  call FAN_Update
  return

FAN_OUT_ON
  SPI_SEND 'O'
  SPI_SEND 'n'
  SPI_SEND ' '
  SPI_SEND ' '
  return

FAN_OUT_OFF
  SPI_SEND 'O'
  SPI_SEND 'f'
  SPI_SEND 'f'
  SPI_SEND ' '
  return

FAN_Update
  SPI_SEND 12					; clear display
  SPI_SEND ' '
  SPI_SEND '1'
  SPI_SEND ':'
  btfss F1_STATUS
    goto F1_OFF
F1_ON
  call FAN_OUT_ON
  goto F2_Test
F1_OFF
  call FAN_OUT_OFF
F2_Test
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND '2'
  SPI_SEND ':'
  btfss F2_STATUS
    goto F2_OFF
F2_ON
  call FAN_OUT_ON
  goto F3_Test
F2_OFF
  call FAN_OUT_OFF
F3_Test
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND '3'
  SPI_SEND ':'
  btfss F3_STATUS
    goto F3_OFF
F3_ON
  call FAN_OUT_ON
  goto F4_Test
F3_OFF
  call FAN_OUT_OFF
F4_Test
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND '4'
  SPI_SEND ':'
  btfss F4_STATUS
    goto F4_OFF
F4_ON
  call FAN_OUT_ON
  return
F4_OFF
  call FAN_OUT_OFF
  return

OPTIONS_Update
  SPI_SEND 12					; clear screen
B_Start
  SPI_SEND 'B'
  SPI_SEND 'a'
  SPI_SEND 'c'
  SPI_SEND 'k'
  SPI_SEND 'l'
  SPI_SEND 'i'
  SPI_SEND 'g'
  SPI_SEND 'h'
  SPI_SEND 't'
  SPI_SEND ':'
  SPI_SEND ' '
  movlw 100
  subwf backlight,w
  btfss STATUS,C
    goto B_80
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  goto C_Start
B_80
  movlw 80
  subwf backlight,w
  btfss STATUS,C
    goto B_60
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  goto C_Start
B_60
  movlw 60
  subwf backlight,w
  btfss STATUS,C
    goto B_40
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  goto C_Start
B_40
  movlw 40
  subwf backlight,w
  btfss STATUS,C
    goto B_20
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  goto C_Start
B_20
  movlw 20
  subwf backlight,w
  btfss STATUS,C
    goto B_0
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  goto C_Start
B_0
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
C_Start
  SPI_SEND ' '
  SPI_SEND 'C'
  SPI_SEND 'o'
  SPI_SEND 'n'
  SPI_SEND 't'
  SPI_SEND 'r'
  SPI_SEND 'a'
  SPI_SEND 's'
  SPI_SEND 't'
  SPI_SEND ':'
  SPI_SEND ' '
  movlw 100
  subwf contrast,w
  btfss STATUS,C
    goto C_80
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  goto END_OPTIONS_Update
C_80
  movlw 80
  subwf contrast,w
  btfss STATUS,C
    goto C_60
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  goto END_OPTIONS_Update
C_60
  movlw 60
  subwf contrast,w
  btfss STATUS,C
    goto C_40
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  goto END_OPTIONS_Update
C_40
  movlw 40
  subwf contrast,w
  btfss STATUS,C
    goto C_20
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  goto END_OPTIONS_Update
C_20
  movlw 20
  subwf contrast,w
  btfss STATUS,C
    goto C_0
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  goto END_OPTIONS_Update
C_0
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
END_OPTIONS_Update
  return					; done

BAR_BLANK
  SPI_SEND ' '
  return

BAR_SOLID
  SPI_SEND 255
  return

EEPROM_read
  movf eeprom_address,w	
  movwf EEADR					; load address to read
  BANK1
  bsf EECON1 ^ 0x80,RD				; initiate read
  BANK0
  movf EEDATA,w					; retrieve data
  return

EEPROM_write
  movf eeprom_address,w
  movwf EEADR					; load address to write to
  movf eeprom_data,w
  movwf EEDATA					; load data to write to EEPROM
  BANK1
  bsf EECON1 ^ 0x80,WREN			; enable writing
  movlw 0x55					; send
  movwf EECON2 ^ 0x80				; required
  movlw 0xAA					; EEPROM
  movwf EECON2 ^ 0x80				; crap
  bsf EECON1 ^ 0x80,WR				; initiate the write
  bcf EECON1 ^ 0x80,WREN			; disable writing
  btfss EECON1 ^ 0x80,EEIF			; wait for completion
    goto $-1
  bcf EECON1 ^ 0x80,EEIF			; clear flag
  BANK0
  return

;----------------------------------------------------------------------------------------
; Main program code
;----------------------------------------------------------------------------------------
Main
  LCD_OUT_SPLASH_SCREEN				; initial splash screen
  DELAY _3
  call FAN_Update				; start with fan display

Mainloop
  btfss BUTTON_1
    call BUTTONPRESS_1
  btfss BUTTON_2
    call BUTTONPRESS_2
  btfss BUTTON_3
    call BUTTONPRESS_3
  btfss BUTTON_4
    call BUTTONPRESS_4
  btfss BUTTON_OPTIONS
    call BUTTONPRESS_OPTIONS
  goto Mainloop

 end

  LIST R=DEC
  INCLUDE "p16f84.inc"
 __CONFIG _CP_OFF & _WDT_OFF & _XT_OSC & _PWRTE_ON

;----------------------------------------------------------------------------------------
; Variable declarations
;----------------------------------------------------------------------------------------
 CBLOCK 0x0C
_w, _status					; interrupt push variables
spi_flag, spi_byte, spi_state			; spi transfer variables
delayvar, delaycounter				; delay routine variables
mode, buttonflags				; main program variables
backlight					; backlight setting
contrast					; contrast setting
fanstatus					; fan status flags
eeprom_address, eeprom_data			; EEPROM function variables
 ENDC

;----------------------------------------------------------------------------------------
; #define's
;----------------------------------------------------------------------------------------
#define SPI_ISR  spi_flag,1			; spi transfer signifier
#define SPI_DONE spi_flag,0			; spi transfer complete flag
#define SPI_CS   PORTA,0			; spi chip select line
#define SPI_CLK  PORTA,1			; spi clock line
#define SPI_DATA PORTA,2			; spi data line
#define DELAY_ISR delayvar,0			; delay routine signifier
#define _DEBOUNCE 2				; 131ms debounce delay time 
#define _STARTUP 7				; LCD startup delay
#define _1 15					; 1 second
#define _2 31					; 2 seconds
#define _3 46					; 3 seconds
#define _4 61					; 4 seconds
#define _5 76					; 5 seconds
#define BUTTON_1 PORTB,0			; 1 button
#define BUTTON_2 PORTB,1			; 2 button
#define BUTTON_3 PORTB,2			; 3 button
#define BUTTON_4 PORTB,3			; 4 button
#define FAN_1	 PORTB,4			; fan 1 line
#define FAN_2	 PORTB,5			; fan 2 line
#define FAN_3	 PORTB,6			; fan 3 line
#define FAN_4	 PORTB,7			; fan 4 line
#define BUTTON_OPTIONS PORTA,3			; options toggle button
#define CHANGE_OPTIONS buttonflags,0		; options toggle indicator
#define F1_STATUS fanstatus,0			; fan 1 status
#define F2_STATUS fanstatus,1			; fan 2 status
#define F3_STATUS fanstatus,2			; fan 3 status
#define F4_STATUS fanstatus,3			; fan 4 status
#define BACKLIGHT_ADDR 	0x00			; EEPROM address for backlight
#define CONTRAST_ADDR 	0x01			; EEPROM address for contrast
#define FAN_ADDR 	0x02			; EEPROM address for fan config

;----------------------------------------------------------------------------------------
; Macro declarations
;----------------------------------------------------------------------------------------
BANK0 macro
  bcf STATUS, RP0           			; Goto Bank 0
  endm

BANK1 macro
  bsf STATUS, RP0           			; Goto Bank 1
  endm

READ_DATA macro ADDRESS
  movlw ADDRESS
  movwf eeprom_address
  call EEPROM_read
  endm

WRITE_DATA macro ADDRESS
  movlw ADDRESS
  movwf eeprom_address
  call EEPROM_write
  endm

SET_OPTION_REG macro DATA
  BANK1
  movlw DATA
  movwf OPTION_REG ^ 0x80
  BANK0
  endm

DELAY macro TIME
  movlw TIME					; load the time entered
  movwf delaycounter				; place it in the counter
  SET_OPTION_REG 0xD7				; highest prescalar ((f/4)/256)
  bsf DELAY_ISR					; say we want the delay ISR
  clrf TMR0					; clear TMR0
  bsf INTCON, T0IE				; enable TMR0 interrupt
  clrw						; clear W
  subwf delaycounter,f				; test for 0
  btfss STATUS, Z				; are we @ 0 yet?
    goto $-4					; if not, keep testing
  bcf INTCON, T0IE				; disable TMR0 interrupt
  bcf DELAY_ISR					; we don't need the delay ISR anymore
  endm

SPI_SEND macro DATA
  movlw DATA
  movwf spi_byte				; load spi byte
  call SPI_Xfer					; send...
  endm

SPI_BIT_OUT macro BITNUM
  bsf SPI_DATA					; assume HIGH
  btfss spi_byte,BITNUM				; if not..
    bcf SPI_DATA				; fix it...
  endm

LCD_OUT_SPLASH_SCREEN macro
  SPI_SEND 12					; clear display, reset cursor
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND 'M'
  SPI_SEND 'r'
  SPI_SEND '.'
  SPI_SEND ' '
  SPI_SEND 'B'
  SPI_SEND 'a'
  SPI_SEND 'y'
  SPI_SEND 'B'
  SPI_SEND 'u'
  SPI_SEND 's'
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND 'F'
  SPI_SEND 'a'
  SPI_SEND 'n'
  SPI_SEND ' '
  SPI_SEND 'S'
  SPI_SEND 'y'
  SPI_SEND 's'
  SPI_SEND 't'
  SPI_SEND 'e'
  SPI_SEND 'm'
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND 'v'
  SPI_SEND '1'
  SPI_SEND '.'
  SPI_SEND '0'
  endm

;----------------------------------------------------------------------------------------
; Program code
;----------------------------------------------------------------------------------------
  PAGE

 org     0
  bsf SPI_CS					; set SPI Chip Select HIGH initially
  bsf SPI_CLK					; set SPI Clock HIGH initially
  call Initialize				; initialize stuff
  goto   Main					; jump to main program code

 org     4
ISR						; Interrupt Service Routine
  btfss DELAY_ISR				; is it the Delay ISR?
    goto ISR_Timers_2				; if not, skip to next test
ISR_Timers_Delay				; delay timer ISR
  decf delaycounter, f				; delay cycle completed, counter--
  bcf INTCON, T0IF				; handle timer interrupt flag
  goto ISR_end					; goto end of ISR
ISR_Timers_2
  btfss SPI_ISR					; is it the SPI ISR?
    goto ISR_Timers_3				; if not, skip to next test
  movf spi_state, w				; load the current state
  incf spi_state, f				; increment state
  BANK1						; switch to BANK 1
  addwf PCL,f					; advance the PC to the correct state
  goto case_cs_low				; state 0
  goto case_bit_7				; state 1
  goto case_clock_low				; state 2
  goto case_clock_high				; state 3
  goto case_bit_6				; state 4
  goto case_clock_low				; state 5
  goto case_clock_high				; state 6
  goto case_bit_5				; state 7
  goto case_clock_low				; state 8
  goto case_clock_high				; state 9
  goto case_bit_4				; state 10
  goto case_clock_low				; state 11
  goto case_clock_high				; state 12
  goto case_bit_3				; state 13
  goto case_clock_low				; state 14
  goto case_clock_high				; state 15
  goto case_bit_2				; state 16
  goto case_clock_low				; state 17
  goto case_clock_high				; state 18
  goto case_bit_1				; state 19
  goto case_clock_low				; state 20
  goto case_clock_high				; state 21
  goto case_bit_0				; state 22
  goto case_clock_low				; state 23
  goto case_clock_high				; state 24
  BANK0
  bsf SPI_DONE					; state 25, we're done
  bsf SPI_CS					; raise the chip select
  goto ISR_SPI_end
case_cs_low					; initial state, drop the chip select
  BANK0
  bcf SPI_CS
  goto ISR_SPI_end
case_clock_low					
  BANK0
  bcf SPI_CLK
  goto ISR_SPI_end
case_clock_high
  BANK0
  bsf SPI_CLK
  goto ISR_SPI_end
case_bit_7					; bit sending states 7-0 (optimize?)
  BANK0
  SPI_BIT_OUT 7
  goto ISR_SPI_end
case_bit_6
  BANK0
  SPI_BIT_OUT 6
  goto ISR_SPI_end
case_bit_5
  BANK0
  SPI_BIT_OUT 5
  goto ISR_SPI_end
case_bit_4
  BANK0
  SPI_BIT_OUT 4
  goto ISR_SPI_end
case_bit_3
  BANK0
  SPI_BIT_OUT 3
  goto ISR_SPI_end
case_bit_2
  BANK0
  SPI_BIT_OUT 2
  goto ISR_SPI_end
case_bit_1
  BANK0
  SPI_BIT_OUT 1
  goto ISR_SPI_end
case_bit_0
  BANK0
  SPI_BIT_OUT 0
  goto ISR_SPI_end
ISR_SPI_end
  bcf INTCON, T0IF				; handle timer interrupt flag
  goto ISR_end					; get outta here
ISR_Timers_3
  goto ISR_end
ISR_end
  retfie					; return from interrupt

;----------------------------------------------------------------------------------------
; User function definitions
;----------------------------------------------------------------------------------------
Initialize					; Initialization function
  BANK1						; switch to BANK 1
  movlw 0xF8					; W <- 1111 1000
  movwf TRISA ^ 0x80				; PORTA[4:3] INPUT, PORTA[2:0] OUTPUT
  movlw 0x0F					; W <- 0000 1111
  movwf TRISB ^ 0x80				; PORTB[7:4] OUTPUT, PORTB[3:0] INPUT
  BANK0						; switch back to BANK 0
  clrf mode					; reset mode to 0
  clrf buttonflags				; clear out change flags
  READ_DATA FAN_ADDR				; load previous fan settings
  movwf fanstatus				; set it.
  btfss F1_STATUS				; set fan 1?
    goto F1_DISABLE				; if not, disable it
  bsf FAN_1					; turn it on
  goto F2_Inittest				; done
F1_DISABLE
  bcf FAN_1					; turn it off
F2_Inittest
  btfss F2_STATUS				; set fan 2?
    goto F2_DISABLE				; if not, disable it
  bsf FAN_2					; turn it on
  goto F3_Inittest				; done
F2_DISABLE
  bcf FAN_2					; turn it off
F3_Inittest
  btfss F3_STATUS				; set fan 3?
    goto F3_DISABLE				; if not, disable it
  bsf FAN_3					; turn it on
  goto F4_Inittest				; done
F3_DISABLE
  bcf FAN_3					; turn it off
F4_Inittest
  btfss F4_STATUS				; set fan 4?
    goto F4_DISABLE				; if not, disable it
  bsf FAN_4					; turn it on
  goto Done_Inittest				; done
F4_DISABLE
  bcf FAN_4					; turn it off
Done_Inittest
  bsf INTCON, GIE				; enable global interrupts
  DELAY _1					; delay to let LCD startup
  SPI_SEND 14					; backlight opcode
  READ_DATA BACKLIGHT_ADDR			; load previous backlight setting
  movwf backlight				; put in backlight variable
  movwf spi_byte				; load function variable
  call SPI_Xfer					; send spi_byte (backlight) to LCD
  SPI_SEND 15					; contrast opcode
  READ_DATA CONTRAST_ADDR			; load previous contrast setting
  movwf contrast				; put in contrast variable
  movwf spi_byte				; load function variable
  call SPI_Xfer					; send spi_byte (contrast) to LCD
  SPI_SEND 4					; NO CURSOR
  SPI_SEND 20					; SCROLL OFF
  return					; return to calling code

SPI_Xfer					; SPI Transfer function
  SET_OPTION_REG 0xD0				; no prescalar
  bcf SPI_DONE					; clear completion flag
  bsf SPI_ISR					; signify that we need the SPI ISR
  clrf spi_state				; reset state machine
  movlw 172					; TMR0 offset
  movwf TMR0					; load it up...
  bsf INTCON, T0IE				; enable timer interrupt
  btfss SPI_DONE				; are we done yet?
    goto $-1					; if not, loop til we are!
  bcf INTCON, T0IE				; disable timer interrupt
  bcf SPI_ISR					; done with the ISR for us...
  return					; return to calling code

BUTTONPRESS_1					; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_1				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_1				; wait for release
    goto $-1
  movlw 0x01					; load 0000 0001
  xorwf fanstatus,f				; toggle fanstatus bit
  movf fanstatus,w
  movwf eeprom_data				; load data
  WRITE_DATA FAN_ADDR				; save fan status
  call FAN_Update				; update the screen
  btfss F1_STATUS				; set fan 1?
    goto F1_DISABLEb				; if not, disable it
  bsf FAN_1					; turn it on
  return					; done
F1_DISABLEb
  bcf FAN_1					; turn it off
  return					; done

BUTTONPRESS_2					; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_2				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_2				; wait for release
    goto $-1
  movlw 0x02					; load 0000 0010
  xorwf fanstatus,f				; toggle fanstatus bit
  movf fanstatus,w
  movwf eeprom_data				; load data
  WRITE_DATA FAN_ADDR				; save fan status
  call FAN_Update				; update the screen
  btfss F2_STATUS				; set fan 2?
    goto F2_DISABLEb				; if not, disable it
  bsf FAN_2					; turn it on
  return					; done
F2_DISABLEb
  bcf FAN_2					; turn it off
  return					; done

BUTTONPRESS_3					; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_3				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_3				; wait for release
    goto $-1
  movlw 0x04					; load 0000 0100
  xorwf fanstatus,f				; toggle fanstatus bit
  movf fanstatus,w
  movwf eeprom_data				; load data
  WRITE_DATA FAN_ADDR				; save fan status
  call FAN_Update				; update the screen
  btfss F3_STATUS				; set fan 3?
    goto F3_DISABLEb				; if not, disable it
  bsf FAN_3					; turn it on
  return					; done
F3_DISABLEb
  bcf FAN_3					; turn it off
  return					; done

BUTTONPRESS_4					; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_4				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_4				; wait for release
    goto $-1  
  movlw 0x08					; load 0000 1000
  xorwf fanstatus,f				; toggle fanstatus bit
  movf fanstatus,w
  movwf eeprom_data				; load data
  WRITE_DATA FAN_ADDR				; save fan status
  call FAN_Update				; update the screen
  btfss F4_STATUS				; set fan 4?
    goto F4_DISABLEb				; if not, disable it
  bsf FAN_4					; turn it on
  return					; done
F4_DISABLEb
  bcf FAN_4					; turn it off
  return					; done

OPT_BL_DN					; decrease backlight level
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_1				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_1				; wait for release
    goto $-1
  clrw						; load 0
  subwf backlight,w				; test 0
  btfsc STATUS,Z				; are we @ 0?
    return					; if so, do nothin...
  movlw 10					; load 10
  subwf backlight,f				; backlight = backlight - 10
  SPI_SEND 14					; BACKLIGHT
  movf backlight,w				; load backlight for execution
  movwf spi_byte				; assign to function variable
  call SPI_Xfer					; spit out the value
  call OPTIONS_Update				; update the screen
  return

OPT_BL_UP					; increase backlight level
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_2				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_2				; wait for release
    goto $-1
  movlw 100					; load 100
  subwf backlight,w				; test 100
  btfsc STATUS,Z				; are we @ 100?
    return					; if so, do nothin...
  movlw 10					; load 10
  addwf backlight,f				; backlight = backlight + 10
  SPI_SEND 14					; BACKLIGHT
  movf backlight,w				; load backlight for execution
  movwf spi_byte				; assign to function variable
  call SPI_Xfer					; spit out the value
  call OPTIONS_Update				; update the screen
  return

OPT_CT_DN
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_3				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_3				; wait for release
    goto $-1
  clrw						; load 0
  subwf contrast,w				; test 0
  btfsc STATUS,Z				; are we @ 0?
    return					; if so, do nothin...
  movlw 10					; load 10
  subwf contrast,f				; contrast = contrast - 10
  SPI_SEND 15					; CONTRAST
  movf contrast,w				; load contrast for execution
  movwf spi_byte				; assign to function variable
  call SPI_Xfer					; spit out the value
  call OPTIONS_Update				; update the screen
  return

OPT_CT_UP
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_4				; are we still pushing?
    return					; if not, get out...
  btfss BUTTON_4				; wait for release
    goto $-1
  movlw 100					; load 100
  subwf contrast,w				; test 100
  btfsc STATUS,Z				; are we @ 0?
    return					; if so, do nothin...
  movlw 10					; load 10
  addwf contrast,f				; contrast = contrast + 10
  SPI_SEND 15					; CONTRAST
  movf contrast,w				; load contrast for execution
  movwf spi_byte				; assign to function variable
  call SPI_Xfer					; spit out the value
  call OPTIONS_Update				; update the screen
  return

BUTTONPRESS_OPTIONS				; function for handling a DOWN press
  DELAY _DEBOUNCE				; debounce to make sure
  btfsc BUTTON_OPTIONS				; are we still pushing?
    return					; if not, get out...
  movlw 0x01					; load 0000 0001
  xorwf buttonflags,f				; toggle CHANGE_OPTIONS
  btfss BUTTON_OPTIONS				; wait for release
    goto $-1
  call OPTIONS_Update				; update the display
OPTION_LOOP					; now we're in the option mode
  btfss BUTTON_1
    call OPT_BL_DN				; backlight--
  btfss BUTTON_2
    call OPT_BL_UP				; backlight++
  btfss BUTTON_3
    call OPT_CT_DN				; contrast--
  btfss BUTTON_4
    call OPT_CT_UP				; contrast++
  btfss BUTTON_OPTIONS
    goto END_OPTIONS				; done
  goto OPTION_LOOP				; keep watching for keypress
END_OPTIONS
  movf backlight,w
  movwf eeprom_data				; load variable
  WRITE_DATA BACKLIGHT_ADDR			; write backlight to EEPROM
  movf contrast,w
  movwf eeprom_data				; load variable
  WRITE_DATA CONTRAST_ADDR			; write contrast to EEPROM
  call FAN_Update
  return

FAN_OUT_ON
  SPI_SEND 'O'
  SPI_SEND 'n'
  SPI_SEND ' '
  SPI_SEND ' '
  return

FAN_OUT_OFF
  SPI_SEND 'O'
  SPI_SEND 'f'
  SPI_SEND 'f'
  SPI_SEND ' '
  return

FAN_Update
  SPI_SEND 12					; clear display
  SPI_SEND ' '
  SPI_SEND '1'
  SPI_SEND ':'
  btfss F1_STATUS
    goto F1_OFF
F1_ON
  call FAN_OUT_ON
  goto F2_Test
F1_OFF
  call FAN_OUT_OFF
F2_Test
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND '2'
  SPI_SEND ':'
  btfss F2_STATUS
    goto F2_OFF
F2_ON
  call FAN_OUT_ON
  goto F3_Test
F2_OFF
  call FAN_OUT_OFF
F3_Test
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND '3'
  SPI_SEND ':'
  btfss F3_STATUS
    goto F3_OFF
F3_ON
  call FAN_OUT_ON
  goto F4_Test
F3_OFF
  call FAN_OUT_OFF
F4_Test
  SPI_SEND ' '
  SPI_SEND ' '
  SPI_SEND '4'
  SPI_SEND ':'
  btfss F4_STATUS
    goto F4_OFF
F4_ON
  call FAN_OUT_ON
  return
F4_OFF
  call FAN_OUT_OFF
  return

OPTIONS_Update
  SPI_SEND 12					; clear screen
B_Start
  SPI_SEND 'B'
  SPI_SEND 'a'
  SPI_SEND 'c'
  SPI_SEND 'k'
  SPI_SEND 'l'
  SPI_SEND 'i'
  SPI_SEND 'g'
  SPI_SEND 'h'
  SPI_SEND 't'
  SPI_SEND ':'
  SPI_SEND ' '
  movlw 100
  subwf backlight,w
  btfss STATUS,C
    goto B_80
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  goto C_Start
B_80
  movlw 80
  subwf backlight,w
  btfss STATUS,C
    goto B_60
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  goto C_Start
B_60
  movlw 60
  subwf backlight,w
  btfss STATUS,C
    goto B_40
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  goto C_Start
B_40
  movlw 40
  subwf backlight,w
  btfss STATUS,C
    goto B_20
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  goto C_Start
B_20
  movlw 20
  subwf backlight,w
  btfss STATUS,C
    goto B_0
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  goto C_Start
B_0
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
C_Start
  SPI_SEND ' '
  SPI_SEND 'C'
  SPI_SEND 'o'
  SPI_SEND 'n'
  SPI_SEND 't'
  SPI_SEND 'r'
  SPI_SEND 'a'
  SPI_SEND 's'
  SPI_SEND 't'
  SPI_SEND ':'
  SPI_SEND ' '
  movlw 100
  subwf contrast,w
  btfss STATUS,C
    goto C_80
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  goto END_OPTIONS_Update
C_80
  movlw 80
  subwf contrast,w
  btfss STATUS,C
    goto C_60
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  goto END_OPTIONS_Update
C_60
  movlw 60
  subwf contrast,w
  btfss STATUS,C
    goto C_40
  call BAR_SOLID
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  goto END_OPTIONS_Update
C_40
  movlw 40
  subwf contrast,w
  btfss STATUS,C
    goto C_20
  call BAR_SOLID
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  goto END_OPTIONS_Update
C_20
  movlw 20
  subwf contrast,w
  btfss STATUS,C
    goto C_0
  call BAR_SOLID
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  goto END_OPTIONS_Update
C_0
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
  call BAR_BLANK
END_OPTIONS_Update
  return					; done

BAR_BLANK
  SPI_SEND ' '
  return

BAR_SOLID
  SPI_SEND 255
  return

EEPROM_read
  movf eeprom_address,w	
  movwf EEADR					; load address to read
  BANK1
  bsf EECON1 ^ 0x80,RD				; initiate read
  BANK0
  movf EEDATA,w					; retrieve data
  return

EEPROM_write
  movf eeprom_address,w
  movwf EEADR					; load address to write to
  movf eeprom_data,w
  movwf EEDATA					; load data to write to EEPROM
  BANK1
  bsf EECON1 ^ 0x80,WREN			; enable writing
  movlw 0x55					; send
  movwf EECON2 ^ 0x80				; required
  movlw 0xAA					; EEPROM
  movwf EECON2 ^ 0x80				; crap
  bsf EECON1 ^ 0x80,WR				; initiate the write
  bcf EECON1 ^ 0x80,WREN			; disable writing
  btfss EECON1 ^ 0x80,EEIF			; wait for completion
    goto $-1
  bcf EECON1 ^ 0x80,EEIF			; clear flag
  BANK0
  return

;----------------------------------------------------------------------------------------
; Main program code
;----------------------------------------------------------------------------------------
Main
  LCD_OUT_SPLASH_SCREEN				; initial splash screen
  DELAY _3
  call FAN_Update				; start with fan display

Mainloop
  btfss BUTTON_1
    call BUTTONPRESS_1
  btfss BUTTON_2
    call BUTTONPRESS_2
  btfss BUTTON_3
    call BUTTONPRESS_3
  btfss BUTTON_4
    call BUTTONPRESS_4
  btfss BUTTON_OPTIONS
    call BUTTONPRESS_OPTIONS
  goto Mainloop

 end
