; ----------------------------------------------------------------------------
; These STM8 assembly language routines implement the flash erase/write routines
; that need to be executed from RAM
; They are equivalent to the C routines in lib/flash.c
; ----------------------------------------------------------------------------

        .module flash
        .globl _rom_eeprom_flash_erase
        .globl _rom_eeprom_flash_write_page

        .area DATA
TEMP1:  .ds 1

        .area RAM_SEG1

FLASH_CR1     .equ 0x505A
FLASH_CR2     .equ 0x505B
FLASH_NCR2    .equ 0x505C
FLASH_IAPSR   .equ 0x505f
FLASH_PUKR    .equ 0x5062

EEPROM_KEY1   .equ 0xAE
EEPROM_KEY2   .equ 0x56

; ----------------------------------------------------------------------------
; bool ram_eeprom_flash_erase( // Return A=0 for false, A=1 for true
;     uint16_t addr) // X =the address of the data (must be within flash address space 0x8000...0xFFFF)
_rom_eeprom_flash_erase:
        ; Get the parameters from the stack
        LDW       X, (0x03, sp) ; Calling convention for SDCC

        ; Setup the operation
        MOV       FLASH_PUKR, #EEPROM_KEY2
        MOV       FLASH_PUKR, #EEPROM_KEY1
        MOV       FLASH_CR1, #0x0
        MOV       FLASH_CR2, #0x20
        MOV       FLASH_NCR2, #0xdf

        ; Write a zero word to the flash, triggering the erase
        LD        A, #0
        LD        (X), A
        LD        (1,X), A
        LD        (2,X), A
        LD        (3,X), A

        ; Was this written to a write protected area?
        BTJT      FLASH_IAPSR, #0x0, efe_fail
        ; Wait for writing to start
efe_1:
        BTJT      FLASH_IAPSR, #0x6, efe_1
        ; Wait for writing to finish
efe_2:
        BTJF      FLASH_IAPSR, #0x2, efe_2

        ; Success
        BRES      FLASH_IAPSR, #0x3
        LD        A, #1
        RET

efe_fail:
        ; Failure
        BRES      FLASH_IAPSR, #0x3
        LD        A, #0
        RET


        .area RAM_SEG2

; ----------------------------------------------------------------------------
; bool ram_eeprom_flash_write_page( // Return A
;     uint16_t addr, // x = The address of the data (must be within flash address space 0x8000...0xFFFF)
;     const uint8_t *data, // y = The data to write
;     bool bEraseFirst) // a = Should we erase it as well as program it
_rom_eeprom_flash_write_page:
        ; Get the parameters from the stack
        LDW       X, (3, sp) ; Calling convention for SDCC
        LDW       Y, (5, sp) ; Calling convention for SDCC
        LD        A, (7, sp) ; Calling convention for SDCC

        MOV       TEMP1, #0x80

        ; Setup the operation
        MOV       FLASH_PUKR, #EEPROM_KEY2
        MOV       FLASH_PUKR, #EEPROM_KEY1
        MOV       FLASH_CR1, #0x0

        TNZ       A
        JREQ      efwp_0

        ; Erase and program it
        MOV       FLASH_CR2, #0x1
        MOV       FLASH_NCR2, #0xfe
        JRA       efwp_loop

        ; Just program it
efwp_0:
        MOV       FLASH_CR2, #0x10
        MOV       FLASH_NCR2, #0xef

		; Main loop. Go through here exactly 128 times.
efwp_loop:
        LD        A, (Y)
        LD        (X), A
        ADDW      Y, #1
        ADDW      X, #1
        DEC       TEMP1
        JRNE      efwp_loop

        ; Was this written to a write protected area?
        BTJT      FLASH_IAPSR, #0, efwp_fail
        ; Wait for writing to start
efwp_4:
        BTJF      FLASH_IAPSR, #6, efwp_4
        ; Wait for writing to finish
efwp_5:
        BTJF      FLASH_IAPSR, #2, efwp_5

        ; Succeed
        BRES      FLASH_IAPSR, #3
        LD        A, #1
        RET

efwp_fail:
        ; Fail
        BRES      FLASH_IAPSR, #3
        LD        A, #0
        RET
