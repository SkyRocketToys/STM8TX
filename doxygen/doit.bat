@echo off
set oldpath=%path%
path=%SystemRoot%;%SystemRoot%/system32;%ProgramFiles%/doxygen/bin;%ProgramFiles(x86)%/MiKTeX 2.9/miktex/bin

doxygen Doxyfile

path=%oldpath%
