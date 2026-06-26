@echo off
rem ==========================================================================
rem Build a PIOC program: .ASM -> .BIN -> _inc.h (C blob)
rem The WCH tools are Windows-only; run this from a Windows shell, in this pioc/ folder.
rem (Day-to-day you don't need it - assemble.py builds the blobs natively. This is only
rem  for ground-truthing a brand-new instruction's encoding; see pioc/README.md.)
rem Usage:  build.bat            (defaults to clocked_sniffer)
rem         build.bat myprog     (assembles myprog.ASM)
rem
rem NOTE: the WCH tools do NOT return a clean exit code, so we don't gate on
rem errorlevel. Source of truth for assembler errors = the generated *.LST
rem (its summary lines). If %NAME%_inc.h was (re)written, assembly succeeded.
rem ==========================================================================

set NAME=%1
if "%NAME%"=="" set NAME=clocked_sniffer

echo [1/2] WASM53B  %NAME%.ASM  -^>  %NAME%.BIN
WASM53B %NAME%

echo [2/2] BIN_HEX  %NAME%.BIN  -^>  %NAME%_inc.h
BIN_HEX %NAME%.BIN %NAME%_inc.h /C

echo.
echo Done. Check the tail of %NAME%.LST for "error" if anything looks off,
echo then rebuild/flash the firmware (sniffer) and run the host tools.
pause
