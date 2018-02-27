@echo off
rem This script file causes the 2018 Streaming Tx to be built using SDCC compiler.
rem required installed programs: SDCC (win32 3.7.1), CygWin (sed, cat, make, gcc, bash), Python (2.7)
rem Note that SDCC 3.6.0 declares "SDCC" macro but 3.7.1 does not; Tridge assumes it is not declared.
rem Note that SDCC x64 (release) 3.6.0 works as installed,
rem      but SDCC x64 (snapshot 10240) 3.7.1 requires rare and exotic dll "libgcc_s_seh-1.dll"
rem      So we can use SDCC win32 (snapshot 10240) 3.7.1 instead

setlocal 
set CYGWIN=C:\cygwin64\bin\
set SDCC_DIR="c:\Program Files (x86)\SDCC\bin\"
set PYTHON_DIR=/cygdrive/c/Python27
set PYTHONPATH=c:\Python27;c:\Python27\Scripts;
set oldpath=%PATH%

set PATH=%SDCC_DIR%;%CYGWIN%;%PYTHONPATH%
bash.exe -c "echo $PATH"
bash.exe -c "echo $PYTHONPATH"
rem bash.exe -c "make clean"
bash.exe -c "make all BRD_RADIO_TYPE=3 WINDOWS=1"
PATH=%oldpath%
