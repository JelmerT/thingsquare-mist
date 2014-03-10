sniffer-help:
	@echo "Compile and upload serial-sniffer.ihex."
	@echo " $$ make TARGET=<mist-platform> serial-sniffer.upload"
	@echo "Start Wireshark and connect to sniffer:"
	@echo " $$ make sniff"


PROJECT = serial-sniffer
CONTIKI_SOURCEFILES += sniffer.c framer-sniffer-802154.c

MIST=../../

CFLAGS+=-DPROJECT_CONF_H=\"project-conf.h\"

include $(MIST)/Makefile.mist

ifndef CONVERTTOBINARY
CONVERTTOBINARY=./convert-to-binary
endif

ifndef HOST_OS
  ifeq ($(OS),Windows_NT)
  ## TODO: detect more specific Windows set-ups,
  ## e.g. CygWin, MingW, VisualC, Watcom, Interix
    HOST_OS := Windows
  else
    HOST_OS := $(shell uname)
  endif
endif

ifeq ($(HOST_OS),Windows)
WIRESHARK=/cygdrive/c/Program\ Files/Wireshark/wireshark.exe
sniff:
ifndef COMPORT
	@echo Must define COMPORT
	@echo Example:
	@echo make sniff COMPORT=/dev/com11
endif
	make login COMPORT=$(COMPORT) | $(CONVERTTOBINARY) | $(WIRESHARK) -k -i -

%.pcap:
ifndef COMPORT
	@echo Must define COMPORT
	@echo Example:
	@echo make file.pcap COMPORT=/dev/com11
endif
	make login COMPORT=$(COMPORT) | $(CONVERTTOBINARY) > $@
else # Windows

ifndef MOTE
MOTE=1
endif

WIRESHARK=wireshark
sniff:
	(sudo make TARGET=$(TARGET) login MOTE=$(MOTE)) | $(CONVERTTOBINARY) | $(WIRESHARK) -k -i -

endif # Windows
