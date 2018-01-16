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

set FW_NAME=FW_TX_%PRODUCT%_%BUILDTYPE%_%REVISION%_%day%%month%%year%.s19
set FW_INFILE=EWSTM8\%FW_BUILDNAME%\Exe\StreamingTransmitter.s19
rem set FW_OUTFILE=T:\SkyRocketToys\firmware2018\%ProductDir%%FW_NAME%

rem Build the firmware
%IARBUILD% iar/BootLoader.ewp -build  %FW_BOOTNAME% -log info
IF %ERRORLEVEL% NEQ 0 goto err2
%IARBUILD% iar/StreamingTransmitter.ewp -build  %FW_BUILDNAME% -log info
IF %ERRORLEVEL% NEQ 0 goto err1
rem Merge the boot and firmware files
rem ...
rem Move it to the right location
rem copy /b %FW_INFILE% %FW_OUTFILE%
rem echo "Firmware copied from " %FW_INFILE% " to " %FW_OUTFILE%
goto end

:err2
echo "There was a problem building the boot loader."
goto end

:err1
echo "There was a problem building the firmware".
:end
