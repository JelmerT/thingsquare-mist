/*
 * Copyright (c) 2012, Thingsquare, http://www.thingsquare.com/.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CC1101_CONFIG_H
#define CC1101_CONFIG_H

#include "contiki-conf.h"

/* Define either CC1101_CONF_ETSI or CC1101_CONF_FCC to 1 to enable
   ETSI (Europe) or FCC (USA) configurations. */
#ifndef CC1101_CONF_ETSI
#define CC1101_CONF_ETSI 1
#endif /* CC1101_CONF_ETSI */

#ifndef CC1101_CONF_FCC
#define CC1101_CONF_FCC 0
#endif /* CC1101_CONF_FCC */

#if CC1101_CONF_ETSI
// Sync word qualifier mode = 30/32 sync word bits detected 
// CRC autoflush = false 
// Channel spacing = 199.951172 
// Data format = Normal mode 
// Data rate = 249.939 
// RX filter BW = 541.666667 
// PA ramping = false 
// Preamble count = 4 
// Whitening = false 
// Address config = No address check 
// Carrier frequency = 867.999939 
// Device address = 0 
// TX power = 0 
// Manchester enable = false 
// CRC enable = true 
// Deviation = 126.953125 
// Packet length mode = Variable packet length mode. Packet length configured by the first byte after sync word 
// Packet length = 255 
// Modulation format = GFSK 
// Base frequency = 867.999939 
// Modulated = true 
// Channel number = 0 
#define CC1101_SETTING_IOCFG2             0x29
#define CC1101_SETTING_IOCFG1             0x2E
#define CC1101_SETTING_IOCFG0             0x06
#define CC1101_SETTING_FIFOTHR            0x07
#define CC1101_SETTING_SYNC1              0xD3
#define CC1101_SETTING_SYNC0              0x91
#define CC1101_SETTING_PKTLEN             0xFF
#define CC1101_SETTING_PKTCTRL1           0x04
#define CC1101_SETTING_PKTCTRL0           0x05
#define CC1101_SETTING_ADDR               0x00
#define CC1101_SETTING_CHANNR             0x00
#define CC1101_SETTING_FSCTRL1            0x0C
#define CC1101_SETTING_FSCTRL0            0x00
#define CC1101_SETTING_FREQ2              0x21
#define CC1101_SETTING_FREQ1              0x62
#define CC1101_SETTING_FREQ0              0x76
#define CC1101_SETTING_MDMCFG4            0x2D
#define CC1101_SETTING_MDMCFG3            0x3B
#define CC1101_SETTING_MDMCFG2            0x13
#define CC1101_SETTING_MDMCFG1            0x22
#define CC1101_SETTING_MDMCFG0            0xF8
#define CC1101_SETTING_DEVIATN            0x62
#define CC1101_SETTING_MCSM2              0x07
#define CC1101_SETTING_MCSM1              0x30
#define CC1101_SETTING_MCSM0              0x18
#define CC1101_SETTING_FOCCFG             0x1D
#define CC1101_SETTING_BSCFG              0x1C
#define CC1101_SETTING_AGCCTRL2           0xC7
#define CC1101_SETTING_AGCCTRL1           0x00
#define CC1101_SETTING_AGCCTRL0           0xB0
#define CC1101_SETTING_WOREVT1            0x87
#define CC1101_SETTING_WOREVT0            0x6B
#define CC1101_SETTING_WORCTRL            0xFB
#define CC1101_SETTING_FREND1             0xB6
#define CC1101_SETTING_FREND0             0x10
#define CC1101_SETTING_FSCAL3             0xEA
#define CC1101_SETTING_FSCAL2             0x2A
#define CC1101_SETTING_FSCAL1             0x00
#define CC1101_SETTING_FSCAL0             0x1F
#define CC1101_SETTING_RCCTRL1            0x41
#define CC1101_SETTING_RCCTRL0            0x00
#define CC1101_SETTING_FSTEST             0x59
#define CC1101_SETTING_PTEST              0x7F
#define CC1101_SETTING_AGCTEST            0x3F
#define CC1101_SETTING_TEST2              0x88
#define CC1101_SETTING_TEST1              0x31
#define CC1101_SETTING_TEST0              0x09
#define CC1101_SETTING_PARTNUM            0x00
#define CC1101_SETTING_VERSION            0x04
#define CC1101_SETTING_FREQEST            0x00
#define CC1101_SETTING_LQI                0x00
#define CC1101_SETTING_RSSI               0x00
#define CC1101_SETTING_MARCSTATE          0x00
#define CC1101_SETTING_WORTIME1           0x00
#define CC1101_SETTING_WORTIME0           0x00
#define CC1101_SETTING_PKTSTATUS          0x00
#define CC1101_SETTING_VCO_VC_DAC         0x00
#define CC1101_SETTING_TXBYTES            0x00
#define CC1101_SETTING_RXBYTES            0x00
#define CC1101_SETTING_RCCTRL1_STATUS     0x00
#define CC1101_SETTING_RCCTRL0_STATUS     0x00

#endif /* CC1101_CONF_ETSI */

#if CC1101_CONF_FCC

// Sync word qualifier mode = 30/32 sync word bits detected 
// CRC autoflush = false 
// Channel spacing = 199.951172 
// Data format = Normal mode 
// Data rate = 249.939 
// RX filter BW = 541.666667 
// PA ramping = false 
// Preamble count = 4 
// Whitening = false 
// Address config = No address check 
// Carrier frequency = 905.998993 
// Device address = 0 
// TX power = 0 
// Manchester enable = false 
// CRC enable = true 
// Deviation = 126.953125 
// Packet length mode = Variable packet length mode. Packet length configured by the first byte after sync word 
// Packet length = 255 
// Modulation format = GFSK 
// Base frequency = 901.999969 
// Modulated = true 
// Channel number = 20 

#define CC1101_SETTING_IOCFG2             0x29
#define CC1101_SETTING_IOCFG1             0x2E
#define CC1101_SETTING_IOCFG0             0x06
#define CC1101_SETTING_FIFOTHR            0x07
#define CC1101_SETTING_SYNC1              0xD3
#define CC1101_SETTING_SYNC0              0x91
#define CC1101_SETTING_PKTLEN             0xFF
#define CC1101_SETTING_PKTCTRL1           0x04
#define CC1101_SETTING_PKTCTRL0           0x05
#define CC1101_SETTING_ADDR               0x00
#define CC1101_SETTING_CHANNR             0x14
#define CC1101_SETTING_FSCTRL1            0x0C
#define CC1101_SETTING_FSCTRL0            0x00
#define CC1101_SETTING_FREQ2              0x22
#define CC1101_SETTING_FREQ1              0xB1
#define CC1101_SETTING_FREQ0              0x3B
#define CC1101_SETTING_MDMCFG4            0x2D
#define CC1101_SETTING_MDMCFG3            0x3B
#define CC1101_SETTING_MDMCFG2            0x13
#define CC1101_SETTING_MDMCFG1            0x22
#define CC1101_SETTING_MDMCFG0            0xF8
#define CC1101_SETTING_DEVIATN            0x62
#define CC1101_SETTING_MCSM2              0x07
#define CC1101_SETTING_MCSM1              0x30
#define CC1101_SETTING_MCSM0              0x18
#define CC1101_SETTING_FOCCFG             0x1D
#define CC1101_SETTING_BSCFG              0x1C
#define CC1101_SETTING_AGCCTRL2           0xC7
#define CC1101_SETTING_AGCCTRL1           0x00
#define CC1101_SETTING_AGCCTRL0           0xB0
#define CC1101_SETTING_WOREVT1            0x87
#define CC1101_SETTING_WOREVT0            0x6B
#define CC1101_SETTING_WORCTRL            0xFB
#define CC1101_SETTING_FREND1             0xB6
#define CC1101_SETTING_FREND0             0x10
#define CC1101_SETTING_FSCAL3             0xEA
#define CC1101_SETTING_FSCAL2             0x2A
#define CC1101_SETTING_FSCAL1             0x00
#define CC1101_SETTING_FSCAL0             0x1F
#define CC1101_SETTING_RCCTRL1            0x41
#define CC1101_SETTING_RCCTRL0            0x00
#define CC1101_SETTING_FSTEST             0x59
#define CC1101_SETTING_PTEST              0x7F
#define CC1101_SETTING_AGCTEST            0x3F
#define CC1101_SETTING_TEST2              0x88
#define CC1101_SETTING_TEST1              0x31
#define CC1101_SETTING_TEST0              0x09
#define CC1101_SETTING_PARTNUM            0x00
#define CC1101_SETTING_VERSION            0x04
#define CC1101_SETTING_FREQEST            0x00
#define CC1101_SETTING_LQI                0x00
#define CC1101_SETTING_RSSI               0x00
#define CC1101_SETTING_MARCSTATE          0x00
#define CC1101_SETTING_WORTIME1           0x00
#define CC1101_SETTING_WORTIME0           0x00
#define CC1101_SETTING_PKTSTATUS          0x00
#define CC1101_SETTING_VCO_VC_DAC         0x00
#define CC1101_SETTING_TXBYTES            0x00
#define CC1101_SETTING_RXBYTES            0x00
#define CC1101_SETTING_RCCTRL1_STATUS     0x00
#define CC1101_SETTING_RCCTRL0_STATUS     0x00

#endif /* CC1101_CONF_FCC */

/* bit-field and other definitions */
#define PKTCTRL1_CRC_AUTOFLUSH BV(3)
#define PKTCTRL1_APPEND_STATUS BV(2)
#define PKTCTRL0_CRC_EN BV(2)
#define PKTCTRL0_PKTLEN_VARIABLE BV(0)

#define FIFOTHR_RXFIFO33_TXFIFO32 7
#define FIFOTHR_RXFIFO12_TXFIFO53 2

#define CCA_MODE 3 /* Below threshold or receiving packet */
#define RXOFF_MODE 3 /* Return to RX after packet received */
#define TXOFF_MODE 3 /* Return to RX after packet transmitted */

/* GDO configuration options */
#define IOCFG0_RXFIFO_EOF_PACKET  0x01
#define IOCFG0_RX_OVERFLOW        0x04
#define IOCFG0_TX_UNDERFLOW       0x05
#define IOCFG0_SYNC_WORD          0x06
#define IOCFG0_RXFIFO_CRC_OK      0x07
#define IOCFG0_CCA                0x09

/* re-define some settings */
#undef CC1101_SETTING_FIFOTHR
#define CC1101_SETTING_FIFOTHR    3

#undef CC1101_SETTING_PKTLEN
#define CC1101_SETTING_PKTLEN     0xff

#undef CC1101_SETTING_PKTCTRL1
#define CC1101_SETTING_PKTCTRL1   PKTCTRL1_APPEND_STATUS

#undef CC1101_SETTING_PKTCTRL0
#define CC1101_SETTING_PKTCTRL0   (PKTCTRL0_CRC_EN | PKTCTRL0_PKTLEN_VARIABLE)

#undef CC1101_SETTING_IOCFG2
#define CC1101_SETTING_IOCFG2     IOCFG0_RX_OVERFLOW

#undef CC1101_SETTING_IOCFG0
#define CC1101_SETTING_IOCFG0     IOCFG0_RXFIFO_EOF_PACKET

#undef CC1101_SETTING_MCSM1
#define CC1101_SETTING_MCSM1 (TXOFF_MODE) | (RXOFF_MODE << 2) | (CCA_MODE << 4)
#endif /* CC1101_CONFIG_H */
