/**
 * \defgroup sniffer Sniffer
 * @{
 */
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

/**
 * \file
 *      Sniffer Network sniffer
 *      The Sniffer network sniffer is used by first calling sniffer_init()
 *      with a pointer to a callback or NULL.  * When a packet is captured,
 *      it is first prepared ie given a PCAP format header. Then, the callback
 *      is invoked, unless the callback is set to NULL in which case the pcap is
 *      automatically sent to sniffer_output(). The purpose with the callback is
 *      that a user might want to eg store with cfs instead of using the Sniffer
 *      default output, which is output on the serial port.
 * \author
 *         Marcus Lunden <marcus@thingsquare.com>
 */

#ifndef __SNIFFER_H__
#define __SNIFFER_H__

#include <stdio.h>
#include <stdlib.h>
#include "contiki.h"
#include "contiki-net.h"
/*--------------------------------------------------------------------------*/
/* set buffer length, ie the maximum packet length (snaplen) */
#define CAPTURE_MAX_LEN     400

/* A selection of link layer types, refer to this webpage for more types and for
 * info on how to aquire new ones: http://www.tcpdump.org/linktypes.html
 */
enum PCAP_LINKTYPES {
  LINKTYPE_ETHERNET =                 1,
  LINKTYPE_SLIP =                     8,
  LINKTYPE_PPP =                      9,
  /* Raw IP; IPv4 or IPv6, "version" indicating IPv4 or IPv6. */
  LINKTYPE_RAW_IPvx =                 101,
  LINKTYPE_IEEE802_11 =               105,
  /* best fit for current Mist */
  LINKTYPE_IEEE802_15_4 =             195,
  LINKTYPE_IEEE802_15_4_NONASK_PHY =  215,
  LINKTYPE_IPV4 =                     228,
  LINKTYPE_IPV6 =                     229,
  LINKTYPE_IEEE802_15_4_NOFCS =       230,
};

/* Sniffer states */
enum SNIFFER_STATES {
  SNIFFER_INACTIVE = 0,
  SNIFFER_PAUSED,     /* paused - has not returned control of radio but is not listening */
  SNIFFER_ACTIVE,     /* listening for traffic, empty capture buffer */
  SNIFFER_CAPTURED,   /* has a capture in buffer which is not printed */
  SNIFFER_PRINTING,   /* is currently busy printing/outputting the capture */
  SNIFFER_INVALID,    /* invalid state */
};

/*--------------------------------------------------------------------------*/
/* PCAP Global header */
typedef struct pcap_hdr_s {
  uint32_t magic_number;   /* magic number */
  uint16_t version_major;  /* major version number */
  uint16_t version_minor;  /* minor version number */
  int32_t  thiszone;       /* GMT to local correction */
  uint32_t sigfigs;        /* accuracy of timestamps */
  uint32_t snaplen;        /* max length of captured packets, in octets */
  uint32_t network;        /* data link type */
} pcap_hdr_t;              /* 24 Bytes in total */

/* PCAP Packet header for each recorded packet in the capture */
typedef struct pcaprec_hdr_s {
  uint32_t ts_sec;         /* timestamp seconds */
  uint32_t ts_usec;        /* timestamp microseconds */
  uint32_t incl_len;       /* number of octets of packet saved in file */
  uint32_t orig_len;       /* actual length of packet */
} pcap_caphdr_t;           /* 16 Bytes in total */

/* buffer for one raw captured packet */
typedef struct pcap_capbuf_s {
  uint8_t u8[CAPTURE_MAX_LEN];
} pcap_capbuf_t;

/* the full PCAP to print/send */
typedef struct pcap_cap_s {
  /*  pcap_hdr_t      hdr;*/
  pcap_caphdr_t   caphdr;
  pcap_capbuf_t   buf;
} pcap_t;

/* Define a capture callback function */
typedef void (*cap_callback_t)(pcap_t *pcapbuf, uint16_t len);
/*--------------------------------------------------------------------------*/
/* Init Sniffer network sniffer: take control of radio and start listening
 * Argument: a pointer to a callback which will be invoked on capture.
 * makes it possible to hook in to the capture and process/read before
 * output, enabling eg filtering.
 */
uint8_t sniffer_init(cap_callback_t capcb);

/* start listening for traffic */
void sniffer_start(cap_callback_t capcb);

/* put Sniffer in paused state, ie don't listen for traffic but maintain control
 * of radio
 */
void sniffer_stop(void);

/* Drop whatever is in the buffer */
void sniffer_drop(void);

/* Output the capture buffer. By default prints it on the serial port. */
void sniffer_output(pcap_t *pcapbuf, uint16_t len);

/* called from radio driver to signal that capture occurred and which link
 * layer type
 */
void sniffer_input(uint32_t linktype, void* src, uint16_t len);

/*--------------------------------------------------------------------------*/
/* Some possible extensions to write:
 * uint8_t sniffer_filter_undoadd(void);
 * uint8_t sniffer_filter_add(enum SNIFFER_FILTER_TYPE, void* val);
 * uint8_t sniffer_filter_clear(void);
 * 
 * Filter types suggestions
 * address equal to
 * address not equal to
 * type equal to: IPv4, IPv6, 802.15.4, Rime
 */
/*--------------------------------------------------------------------------*/

#endif // __SNIFFER_H__
/** @} */
