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
It works best with version 3.7.1 or better (e.g. snapshot version 10240 or better)
To flash to a board you need the stm8flash tool.

run ./doit_sdcc.sh

Programs assumed to be installed on this machine:
* sdcc
	* sudo apt-get install sdcc (will install 3.6.0)
	* Or build 3.7.1 by installing/building gputils and libboost and sdcc
	* Get gputils from https://sourceforge.net/projects/gputils/files/latest/download?source=typ_redirect
		* ./configure
		* make 
		* make install
	* sudo apt-get install libboost-all-dev
	* Build sdcc via
		* svn checkout svn://svn.code.sf.net/p/sdcc/code/trunk/sdcc sdcc
		* cd sdcc
		* ./configure
		* make
		* make install
* python (2.7)
* pip
* intelhex
	* sudo -H pip install intelhex
* stm8flash


To build the firmware from Windows using SDCC
=============================================
run doit.bat

Programs assumed to be installed on this machine:
* sdcc
	* Easiest to install the win32 (snapshot 10240) build of 3.7.1 since the x64 version requires a rare dll (libgcc_s_seh-1.dll)
* cygwin with
	* make
	* sed
	* cat
	* gcc
* Python with pip and intelhex
	* pip install intelhex


To build the firmware from Windows using IAR
=============================================
This is an internal build for development purposes.

change doit_iar.bat to use directories valid on your installation.
* IARBUILD
* FW_OUTFILE
run doit_iar.bat

Programs assumed to be installed on this machine:
* IAR embedded workbench for STM8 (commercial compiler)
* svn (for build numbers)
* cheese (HotGen tool included in the svn repository)
