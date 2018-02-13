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
run doit_iar.bat

Programs assumed to be installed on this machine:
* iar
