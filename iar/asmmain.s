        MODULE  asmmain

        PUBLIC  __iar_program_start
        PUBLIC  main

        EXTERN  CSTACK$$Limit

        SECTION `.near_func.text`:CODE:NOROOT(0)

__iar_program_start:
        LDW     X, #CSTACK$$Limit-1     ; Set stackpointer
        LDW     SP, X

main:
        HALT                            ; End of program
        NOP

        END

