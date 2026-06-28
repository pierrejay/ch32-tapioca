;==============================================================================
; mdio_master.ASM - active MDIO (Clause-22) master, BIT-BANGED MDC.
;
; MDC (IO0/PC18) is toggled by hand via OUT0, so every transaction emits a deterministic,
; byte-aligned 64-clock frame (32 preamble + 32). Each bit is one MDC cycle, FALL-FIRST:
; MDC idles HIGH, the bit makes its own falling then rising edge. MDIO changes just after
; the falling edge; the PHY samples at the rising edge; on a read we sample the PHY's data
; late in the low phase (well settled).
;
; Pins:  IO0 (PC18) = MDC  (output, bit-banged via OUT0)
;        IO1 (PC19) = MDIO (bidirectional; BP2F drive / BG2F,BCTC sample)
; Timing: HALF is a NOP delay (CLK_10 idiom from the WCH PIOC_IIC example). MDC has no
; minimum clock and the LAN8720 max is 2.5 MHz, so a slow clock always works; the
; half-period (NOP count in HALF) is the only tunable.
;
; MEASURED MDC = ~1.10 MHz (907 ns/cycle), timed over 640000 cycles (bespoke probe).
; Each cycle = 39 instructions (BC + BP2F + CALL HALF + BS + CALL HALF) = ~43.5 eMCU cycles
; @48 MHz, which pins the timing: NOP/BS/BC/BP2F = 1 cycle, CALL/RET = 2. One NOP in HALF
; ~= 2 ns/MDC-cycle; ~14-15 NOPs would target exactly 1.00 MHz.
;
; One synchronous transaction at a time, via a host doorbell (no ring buffer):
; host writes CMD0..3 + CTRL(op=bit0, GO=bit7); blob runs; blob writes
; RES_H/RES_L then STATUS *last* (1=ok / 0x80=no-resp); host polls STATUS.
;
; Clause-22 post-preamble frame = exactly 32 bits / 4 bytes, MSB-first:
;   CMD0 = ST1 ST0 OP1 OP0 PH4 PH3 PH2 PH1
;   CMD1 = PH0 RG4 RG3 RG2 RG1 RG0 TA1 TA0
;   CMD2 = D15..D8     CMD3 = D7..D0     (write: driven; read: sampled)
;==============================================================================

INCLUDE             PIOC_INC.ASM

                    ORG   0X0000
                    DW    0X0000
                    JMP   MCU_START
                    DW    0X0FFF

CMD0                EQU   SFR_DATA_REG2     ; ST OP PHYAD[4:1]
CMD1                EQU   SFR_DATA_REG3     ; PHYAD[0] REGAD TA
CMD2                EQU   SFR_DATA_REG4     ; DATA[15:8] (write)
CMD3                EQU   SFR_DATA_REG5     ; DATA[7:0]  (write)
CTRL                EQU   SFR_DATA_REG6     ; host doorbell: bit0 op(1=read), bit7 GO
STATUS              EQU   SFR_DATA_REG7     ; blob->host: 0 busy, 1 ok, 0x80 no-resp
RES_H               EQU   SFR_DATA_REG8     ; read DATA[15:8]
RES_L               EQU   SFR_DATA_REG9     ; read DATA[7:0]

MCU_START:
                    CLR   SFR_PORT_DIR                  ; both pins INPUT to start (safe)
                    BS    SFR_PORT_DIR,SB_PORT_PU1      ; pull-up MDIO -> released line high (no-PHY -> 0xFFFF)

; ---- wait for the host doorbell (CTRL bit7 = GO) ----------------------------
WAIT_GO:            BTSS  CTRL,7
                    JMP   WAIT_GO
                    BC    CTRL,7                        ; consume GO (one-shot)
                    CLR   STATUS                        ; mark busy

; ---- bring up pins: MDC output idle HIGH, MDIO output high ------------------
                    BS    SFR_PORT_IO,SB_PORT_OUT0      ; latch MDC high BEFORE output -> no bring-up pulse
                    BS    SFR_PORT_IO,SB_PORT_OUT1      ; latch MDIO high BEFORE output
                    BS    SFR_PORT_DIR,SB_PORT_DIR0     ; MDC  = output (drives the high latch cleanly)
                    BS    SFR_PORT_DIR,SB_PORT_DIR1     ; MDIO = output
                    ; MDC idles HIGH (here and in STOP_CLK) so the 1st cycle's BC is the 1st edge ->
                    ; exactly 64 clocks, no extra bring-up rising edge / phantom preamble bit.

; ---- preamble: 32 ones ------------------------------------------------------
                    MOVL  0XFF
                    MOVA  SFR_DATA_EXCH
                    CALL  DRIVE8
                    CALL  DRIVE8
                    CALL  DRIVE8
                    CALL  DRIVE8

; ---- CMD0 (ST OP PHYAD[4:1]) - driven for both read and write --------------
                    MOV   CMD0,A
                    MOVA  SFR_DATA_EXCH
                    CALL  DRIVE8
                    BTSC  CTRL,0                        ; op bit0 set => read
                    JMP   RD_PATH

; ================= WRITE: drive CMD1 (incl TA=10), then 16 data bits =========
                    MOV   CMD1,A
                    MOVA  SFR_DATA_EXCH
                    CALL  DRIVE8
                    MOV   CMD2,A
                    MOVA  SFR_DATA_EXCH
                    CALL  DRIVE8
                    MOV   CMD3,A
                    MOVA  SFR_DATA_EXCH
                    CALL  DRIVE8
                    JMP   DONE_OK

; ================= READ: drive 6 bits of CMD1, turnaround, sample 16 ========
RD_PATH:            MOV   CMD1,A
                    MOVA  SFR_DATA_EXCH
                    ; drive bits 7..2 (PHYAD[0] + REGAD), leaving the 2 TA bits
                    BC    SFR_PORT_IO,SB_PORT_OUT0      ; bit7: fall
                    BP2F  BO_PORT_OUT1,7
                    CALL  HALF
                    BS    SFR_PORT_IO,SB_PORT_OUT0      ; rise
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0      ; bit6
                    BP2F  BO_PORT_OUT1,6
                    CALL  HALF
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0      ; bit5
                    BP2F  BO_PORT_OUT1,5
                    CALL  HALF
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0      ; bit4
                    BP2F  BO_PORT_OUT1,4
                    CALL  HALF
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0      ; bit3
                    BP2F  BO_PORT_OUT1,3
                    CALL  HALF
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0      ; bit2
                    BP2F  BO_PORT_OUT1,2
                    CALL  HALF
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF

                    ; --- TA1 (Z): release MDIO, one empty MDC cycle ---
                    BC    SFR_PORT_DIR,SB_PORT_DIR1     ; release MDIO (input) for turnaround
                    BC    SFR_PORT_IO,SB_PORT_OUT0      ; TA1 fall
                    CALL  HALF
                    BS    SFR_PORT_IO,SB_PORT_OUT0      ; rise
                    CALL  HALF
                    ; --- TA0: PHY drives 0; sample late-low (no-PHY check) ---
                    BC    SFR_PORT_IO,SB_PORT_OUT0      ; TA0 fall (PHY changes here)
                    CALL  HALF
                    BCTC  BI_PORT_IN1                   ; sample TA0 late-low (settled)
                    BTSC  SFR_STATUS_REG,SB_FLAG_C      ; carry set (line high) = no PHY answered
                    BS    SFR_STATUS_REG,SB_GP_BIT_X    ;   -> latch the no-resp flag
                    BS    SFR_PORT_IO,SB_PORT_OUT0      ; rise (complete TA0); MDC high -> SAMPLE8 starts at D15
                    CALL  HALF

                    ; --- sample 16 data bits -> RES_H, RES_L ---
                    CALL  SAMPLE8
                    MOV   SFR_DATA_EXCH,A
                    MOVA  RES_H
                    CALL  SAMPLE8
                    MOV   SFR_DATA_EXCH,A
                    MOVA  RES_L
                    BTSC  SFR_STATUS_REG,SB_GP_BIT_X    ; no-resp latched? -> error path (STATUS = 0x80)
                    JMP   DONE_ERR

; ---- publish result + status (status LAST), then re-arm ---------------------
DONE_OK:            CALL  STOP_CLK
                    MOVL  0X01
                    MOVA  STATUS
                    BC    SFR_STATUS_REG,SB_GP_BIT_X
                    JMP   WAIT_GO
DONE_ERR:           CALL  STOP_CLK
                    MOVL  0X80
                    MOVA  STATUS
                    BC    SFR_STATUS_REG,SB_GP_BIT_X
                    JMP   WAIT_GO

; ---- subroutines -----------------------------------------------------------
; idle the bus after a transaction: re-drive MDIO high, leave MDC HIGH (so the next
; transaction's bring-up adds no extra rising edge -> the 64-clock contract holds).
STOP_CLK:           BS    SFR_PORT_IO,SB_PORT_OUT1      ; latch MDIO high BEFORE re-driving (no low glitch)
                    BS    SFR_PORT_DIR,SB_PORT_DIR1     ; re-drive MDIO (was input if a read)
                    BS    SFR_PORT_IO,SB_PORT_OUT0      ; MDC idle HIGH
                    RET

; drive SFR_DATA_EXCH bits 7..0 onto MDIO, MSB first. Per bit (fall-first): fall +
; set MDIO, low phase, rise (PHY samples), high phase. MDC enters/leaves HIGH.
DRIVE8:             BC    SFR_PORT_IO,SB_PORT_OUT0
                    BP2F  BO_PORT_OUT1,7
                    CALL  HALF
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0
                    BP2F  BO_PORT_OUT1,6
                    CALL  HALF
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0
                    BP2F  BO_PORT_OUT1,5
                    CALL  HALF
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0
                    BP2F  BO_PORT_OUT1,4
                    CALL  HALF
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0
                    BP2F  BO_PORT_OUT1,3
                    CALL  HALF
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0
                    BP2F  BO_PORT_OUT1,2
                    CALL  HALF
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0
                    BP2F  BO_PORT_OUT1,1
                    CALL  HALF
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0
                    BP2F  BO_PORT_OUT1,0
                    CALL  HALF
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    RET

; sample MDIO into SFR_DATA_EXCH bits 7..0, MSB first. Per bit (fall-first): fall
; (PHY changes), low phase (settle), sample late-low, rise, high phase.
SAMPLE8:            BC    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BG2F  BI_PORT_IN1,7
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BG2F  BI_PORT_IN1,6
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BG2F  BI_PORT_IN1,5
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BG2F  BI_PORT_IN1,4
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BG2F  BI_PORT_IN1,3
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BG2F  BI_PORT_IN1,2
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BG2F  BI_PORT_IN1,1
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BC    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    BG2F  BI_PORT_IN1,0
                    BS    SFR_PORT_IO,SB_PORT_OUT0
                    CALL  HALF
                    RET

; ---- half MDC period: a fixed NOP delay (CLK_10 idiom). The ONLY timing knob.
; 16 NOPs -> ~1.10 MHz MDC measured (see the header). Each NOP = 1 eMCU cycle = ~2 ns
; per MDC cycle; ~14-15 NOPs would land on 1.00 MHz. Slow is always safe (no MDIO
; minimum; a 64-clock frame is < 1 ms even at 100 kHz, well under the 5 ms host timeout).
HALF:               NOP
                    NOP
                    NOP
                    NOP
                    NOP
                    NOP
                    NOP
                    NOP
                    NOP
                    NOP
                    NOP
                    NOP
                    NOP
                    NOP
                    NOP
                    NOP
                    RET
