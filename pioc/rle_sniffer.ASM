;==============================================================================
; can_rle_gen_ring30.ASM - 30-slot experimental variant of can_rle_gen.ASM.
;
; The wire protocol and every PIOC hot path are identical to can_rle_gen.ASM.
; Only the shared-data layout changes:
;   DATA_REG0 = CNT, DATA_REG1 = HEAD, DATA_REG2..31 = 30-slot FIFO.
;
; Starting at DATA_REG2 lets the indirect pointer reach 0x40 after slot 31.
; BTSC bit6 detects that wrap in exactly the same two instructions as the old
; 16-slot bit4 wrap, so this variant must not reduce the capture blind margin.
; The CPU side deliberately uses a separate physical cursor: 30 is not a power
; of two and HEAD's 8-bit wrap cannot be masked into a slot index.
;==============================================================================

INCLUDE             PIOC_INC.ASM

                    ORG   0X0000
                    DW    0X0000
                    JMP   MCU_START
                    DW    0X0FFF

CNT                 EQU   SFR_DATA_REG0         ; run-length counter (ticks)
HEAD                EQU   SFR_DATA_REG1         ; published logical byte count
RING_BASE           EQU   SFR_DATA_REG2         ; physical slots DATA_REG2..31
CAP_BIT             EQU   7                     ; cap at 2^7 = 128 ticks

MCU_START:
                    CLR   SFR_PORT_DIR          ; all IOs = INPUT -> passive sniffer (data on IO1=PC19)
                    CLR   HEAD
                    CLR   CNT
                    MOVIA RING_BASE              ; ring pointer -> DATA_REG2

; ---- level 1 run (line high) : count, exit on falling edge or at the cap ----
HIGH:
                    BCTC  BI_PORT_IN1           ; C = data line (IO1 = PC19, unified across modes)
                    BTSC  SFR_STATUS_REG, SB_FLAG_C
                    JMP   HI_RUN
                    MOV   CNT, A
                    MOVA  SFR_INDIR_PORT2
                    INC   HEAD
                    BTSC  SFR_INDIR_ADDR2, 6    ; ptr reached 0x40 after REG31?
                    MOVIA RING_BASE
                    CLR   CNT
                    JMP   LOW
HI_RUN:
                    INC   CNT
                    BTSC  CNT, CAP_BIT
                    JMP   HI_CAP
                    JMP   HIGH
HI_CAP:
                    MOV   CNT, A
                    MOVA  SFR_INDIR_PORT2
                    INC   HEAD
                    BTSC  SFR_INDIR_ADDR2, 6
                    MOVIA RING_BASE
                    CLR   CNT
                    JMP   HIGH                  ; cap-continuation: same level

; ---- level 0 run (line low) : count, exit on rising edge or at the cap ------
LOW:
                    BCTC  BI_PORT_IN1           ; C = data line (IO1 = PC19, unified across modes)
                    BTSC  SFR_STATUS_REG, SB_FLAG_C
                    JMP   RISE
                    INC   CNT
                    BTSC  CNT, CAP_BIT
                    JMP   LO_CAP
                    JMP   LOW
RISE:
                    MOV   CNT, A
                    MOVA  SFR_INDIR_PORT2
                    INC   HEAD
                    BTSC  SFR_INDIR_ADDR2, 6
                    MOVIA RING_BASE
                    CLR   CNT
                    JMP   HIGH
LO_CAP:
                    MOV   CNT, A
                    MOVA  SFR_INDIR_PORT2
                    INC   HEAD
                    BTSC  SFR_INDIR_ADDR2, 6
                    MOVIA RING_BASE
                    CLR   CNT
                    JMP   LOW                   ; cap-continuation: same level
