@echo off
rem This script file causes the 2018 Streaming Tx to be built using SDCC compiler.
rem required installed programs: SDCC, CygWin (sed, cat, make, gcc, bash), Python (2.7)

setlocal 
set CYGWIN=C:\cygwin64\bin\
set SDCC="c:\Program Files\SDCC\bin\"
set PYTHON_DIR=/cygdrive/c/Python27
set PYTHONPATH=c:\Python27;c:\Python27\Scripts;
set oldpath=%PATH%

set PATH=%SDCC%;%CYGWIN%;%PYTHONPATH%
bash.exe -c "make clean"
PATH=%oldpath%
