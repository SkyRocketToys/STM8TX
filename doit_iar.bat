@echo off
setlocal

set PRODUCT=Beken
set ProductDir=Streaming2018\
set BUILDTYPE=PD
set IARBUILD="%ProgramFiles(x86)%\IAR Systems\Embedded Workbench 7.3\common\bin\IarBuild.exe"

svn info | grep "Last Changed Rev" >info.tmp
for /f "usebackq tokens=4" %%a in (`find "Last Changed Rev" info.tmp`) do (set REVISION=%%a)
echo SVN revision = %REVISION%

rem use wmic to get the month name
for /f "delims=" %%a in ('wmic OS Get localdatetime ^| find "."') do set dt=%%a
set year=%dt:~0,4%
set month=%dt:~4,2%
set day=%dt:~6,2%
if %month%==01 set month=jan
if %month%==02 set month=feb
if %month%==03 set month=mar
if %month%==04 set month=apr
if %month%==05 set month=may
if %month%==06 set month=jun
if %month%==07 set month=jul
if %month%==08 set month=aug
if %month%==09 set month=sep
if %month%==10 set month=oct
if %month%==11 set month=nov
if %month%==12 set month=dec

set FW_BOOTNAME=BootRelease_%PRODUCT%
set FW_BUILDNAME=Debug_%PRODUCT%

set FW_NAME=FW_TX_%PRODUCT%_%BUILDTYPE%_%REVISION%_%day%%month%%year%.hex
set FW_INFILE=iar_combined.ihx
set FW_OUTFILE=T:\SkyRocketToys\firmware2018\%ProductDir%%FW_NAME%

rem Build the firmware
%IARBUILD% iar/BootLoader.ewp -build  %FW_BOOTNAME% -log info
IF %ERRORLEVEL% NEQ 0 goto err2
%IARBUILD% iar/StreamingTransmitter.ewp -build  %FW_BUILDNAME% -log info
IF %ERRORLEVEL% NEQ 0 goto err1

rem Merge the boot and firmware files
WinTools\cheese.exe dat2dat iar\BootRelease_Beken\Exe\BootLoader.hex iar_boot.bin
del iar_boot.h
WinTools\cheese.exe dat2dat iar\Debug_Beken\Exe\StreamingTransmitter.hex iar_txmain.bin
del iar_txmain.h
WinTools\cheese.exe extract iar_combined.bin -pad 255 -i iar_boot.bin 0 $8700 -i iar_txmain.bin $8700 $3900 -i iar_txmain.bin $86fa $3900
del iar_combined.h
WinTools\cheese.exe dat2dat iar_combined.bin iar_combined.ihx -outrange $8000 $4000
del iar_combined.h
WinTools\cheese.exe dat2dat iar_txmain.bin iar_chk.bin -outrange $8700-6 $c000-$8700+6 -checksum [crc16ardupilot] -checkdst $8700-4 -checksrc $8700+$3900/2 $3900/2 -checksrc $8700 $3900/2 -setbyte $8700-6 $39 -setbyte $8700-5 0
del iar_chk.h
WinTools\cheese.exe extract iar_txmain.img -i iar_chk.bin $8700-6 -1

rem Move it to the right location
copy /b %FW_INFILE% %FW_OUTFILE%
echo "Firmware copied from " %FW_INFILE% " to " %FW_OUTFILE%
goto end

:err2
echo "There was a problem building the boot loader."
goto end

:err1
echo "There was a problem building the firmware".
:end
