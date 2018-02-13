StreamingGPS Tx
===============

This is the firmware for the SkyRocket StreamingGPS transmitter

  http://sky-viper.com/product/info/V2450GPS-sd


This firmware is released under the GNU GPL version 3 or later. See
the file COPYING for details.

---

This branch is for the Beken version


To build the firmware from Linux using SDCC
===========================================
You need a recent version of sdcc.
To flash to a board you need the stm8flash tool.

run ./doit_sdcc.sh

Programs assumed to be installed on this machine:
* sdcc
	* sudo apt-get install sdcc
* python
* pip
* intelhex
	* sudo -H pip install intelhex
* stm8flash


To build the firmware from Windows using SDCC
=============================================
run doit.bat

Programs assumed to be installed on this machine:
* sdcc
* cygwin with
	* make
	* sed
	* cat
	* gcc
* Python with pip and intelhex
	* pip install intelhex


To build the firmware from Windows using IAR
=============================================
This is an internal build for development purposes, for the svn version not github.

change doit_iar.bat to use directories valid on your installation.
* IARBUILD
* FW_OUTFILE
run doit_iar.bat

Programs assumed to be installed on this machine:
* IAR embedded workbench for STM8
* svn (for build numbers)
* cheese (HotGen tool included in the svn repository)
