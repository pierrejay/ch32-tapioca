;==============================================================================
; clocked_sniffer.ASM - DMA-style clocked byte FIFO into the host-shared data regs.
;
; The eMCU never stops: it packs 8 captured bits into one byte and stores it into
; a 16-slot ring (DATA_REG0..15) using the auto-incrementing indirect port
; (SFR_INDIR_PORT2 / SFR_INDIR_ADDR2). After each store it publishes a monotonic
; byte counter in HEAD (DATA_REG31) as the LAST write -> the host drains only
; slots strictly behind HEAD, never the one being written = no torn read, no need
; to park the eMCU. Host keeps a tail; avail = (HEAD - tail) & 0xFF; slot = tail&15.
;
; Mid-bit sampling (wait rising then falling, sample after falling) - the
; speed/phase-robust point validated in the window experiments.
;
; Ring = 16 (power of two, divides 256) so the host's (HEAD&15) slot index stays
; consistent across HEAD's 8-bit wrap. No hardware ring-wrap: when the indirect
; pointer reaches 0x30 (= DATA_REG16, just past the ring) we reload it to 0x20.
;
; Per bit: WAITB rising, WAITB falling, BCTC, RCL = 2 executed instr (very light).
; Per byte boundary: store + publish + wrap-check ~= 6 instr.
;==============================================================================

INCLUDE             PIOC_INC.ASM

                    ORG   0X0000
                    DW    0X0000
                    JMP   MCU_START
                    DW    0X0FFF

ACC                 EQU   SFR_DATA_REG30        ; bit accumulator (not in the ring)
HEAD                EQU   SFR_DATA_REG31        ; published byte counter (host reads)

MCU_START:
                    CLR   SFR_PORT_DIR           ; both PIOC IO pins = INPUT -> passive tap, NEVER drive the bus
                    CLR   HEAD
                    MOVIA SFR_DATA_REG0          ; INDIR_ADDR2 = 0x20 (ring slot 0)

BYTE_LOOP:
                    ; seed ACC with a sentinel bit (bit0). RCL rotates it up one slot per
                    ; sample, so after k samples the seed sits at bit k -> the CPU can read
                    ; ACC at idle and recover the partial-bit count (its highest set bit) to
                    ; flush a mid-byte without loss. On the 8th sample the seed rotates out via
                    ; carry, so a COMPLETE byte is bit-for-bit unchanged (ring stays identical).
                    MOVL  0X01
                    MOVA  ACC
                    ; --- 8 bits, unrolled, mid-bit sampling -------------------
                    WAITB WB_PORT_XOR0_1
                    WAITB WB_PORT_XOR0_0
                    BCTC  BI_PORT_IN1
                    RCL   ACC
                    WAITB WB_PORT_XOR0_1
                    WAITB WB_PORT_XOR0_0
                    BCTC  BI_PORT_IN1
                    RCL   ACC
                    WAITB WB_PORT_XOR0_1
                    WAITB WB_PORT_XOR0_0
                    BCTC  BI_PORT_IN1
                    RCL   ACC
                    WAITB WB_PORT_XOR0_1
                    WAITB WB_PORT_XOR0_0
                    BCTC  BI_PORT_IN1
                    RCL   ACC
                    WAITB WB_PORT_XOR0_1
                    WAITB WB_PORT_XOR0_0
                    BCTC  BI_PORT_IN1
                    RCL   ACC
                    WAITB WB_PORT_XOR0_1
                    WAITB WB_PORT_XOR0_0
                    BCTC  BI_PORT_IN1
                    RCL   ACC
                    WAITB WB_PORT_XOR0_1
                    WAITB WB_PORT_XOR0_0
                    BCTC  BI_PORT_IN1
                    RCL   ACC
                    WAITB WB_PORT_XOR0_1
                    WAITB WB_PORT_XOR0_0
                    BCTC  BI_PORT_IN1
                    RCL   ACC
                    ; --- store the byte, publish head, wrap pointer ----------
                    MOV   ACC,A                  ; A = assembled byte
                    MOVA  SFR_INDIR_PORT2         ; ring[ptr] = A ; ptr++ (auto-inc)
                    INC   HEAD                    ; publish AFTER the store
                    BTSC  SFR_INDIR_ADDR2,4       ; ptr reached 0x30 (bit4 set)?
                    MOVIA SFR_DATA_REG0           ;   yes -> wrap ptr back to 0x20
                    JMP   BYTE_LOOP
