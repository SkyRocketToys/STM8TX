@echo off
set oldpath=%path%
path=%SystemRoot%;%SystemRoot%/system32;%ProgramFiles%/doxygen/bin;%ProgramFiles(x86)%/MiKTeX 2.9/miktex/bin

doxygen Doxyfile
cd StreamingTx\latex
call make.bat pdf
cd ..\..
copy StreamingTx\latex\refman.pdf StreamingTx\StreamingTx.pdf

path=%oldpath%
