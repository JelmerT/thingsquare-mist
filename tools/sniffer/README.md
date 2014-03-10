This is a simple sniffer. The sniffer runs a device that is connected
to a local USB/serial port and prints out sniffed packets as
pcap-compatible data that can be fed directly into Wireshark. Allows
real-time sniffing.

How to use the sniffer:


* Compile and upload the sniffer binary to the device. This is
  normally done with `make serial-sniffer.upload TARGET=x`, where `x`
  is the device platform.

* Connect the device to a serial port.
