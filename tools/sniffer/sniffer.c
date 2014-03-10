/**
 * \addtogroup sniffer
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
 *         Sniffer, the network sniffer. Outputs packet captures in PCAP format
 * \author
 *         Marcus Lunden <marcus@thingsquare.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "contiki-net.h"
#include "dev/leds.h"
#include "sniffer.h"
#include "sniffer-conf.h"

/*---------------------------------------------------------------------------*/
/* not ready yet */
#if INCLUDE_SHELLCONF
  #include "shell.h"
  #include "serial-shell.h"
#endif

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif
/*--------------------------------------------------------------------------*/
PROCESS(sniffer_process, "Sniffer Process");
static uint8_t sstate = SNIFFER_INACTIVE;
cap_callback_t cap_cb = NULL;
static pcap_t pcap;
static uint16_t capdatalen = 0;
/*--------------------------------------------------------------------------*/
/**
 * \brief      Init the Sniffer network sniffer
 * \param capcb    Pointer to packet capture handling callback 
 * \return        always returns 0
 */
uint8_t
sniffer_init(cap_callback_t capcb)
{
  pcap_hdr_t hdr;
#if INCLUDE_SHELLCONF
  shell_register_command(&snifferconf_command);
#endif
  PRINTF("Sniffer starting\n");
  memset(&pcap, 0, sizeof(pcap_t));
  sniffer_start(capcb);
  process_start(&sniffer_process, NULL);


#if 0
  /* Print out PCAP header */
  /* PCAP header */
  hdr.magic_number = 0xa1b2c3d4;
  hdr.version_major = 2;   /* current pcap format is 2.4 */
  hdr.version_minor = 4;
  hdr.sigfigs = 0;         /* in practice always 0 */
  hdr.snaplen = CAPTURE_MAX_LEN; 

  /* Set the time of this capture; TODO: set up with timesynch */
  hdr.thiszone = 0;        /* assume GMT */
  hdr.network = LINKTYPE_IEEE802_15_4;

  sniffer_output((pcap_t *)&hdr, sizeof(hdr));
#endif
  return 0;
}
/*--------------------------------------------------------------------------*/
/**
 * \brief      Start capturing packets; turns off radio duty cycling
 * \param capcb   Pointer to packet capture handling callback 
 */
void
sniffer_start(cap_callback_t capcb)
{
  if(sstate != SNIFFER_ACTIVE) {
    cap_cb = capcb;
    sstate = SNIFFER_ACTIVE;
    
    /* set the radio and network layer in promiscous mode and always on. */
#ifdef SNIFFER_CONF_RADIO_SET_PROMISCUOUS
    SNIFFER_CONF_RADIO_SET_PROMISCUOUS(1);
#endif /* SNIFFER_CONF_RADIO_SET_PROMISCUOUS */
    NETSTACK_MAC.off(1);
    /* when multichan_driver is off, it stops following the channel hops */
#if NETSTACK_RDC != multichan_driver
    NETSTACK_RDC.off(1);
#endif
  }
}
/*--------------------------------------------------------------------------*/
/**
 * \brief      Stop network sniffing and return to normal operations
 */
void
sniffer_stop(void)
{
  if(sstate == SNIFFER_ACTIVE) {
    sstate = SNIFFER_INACTIVE;
#ifdef SNIFFER_CONF_RADIO_SET_PROMISCUOUS
    SNIFFER_CONF_RADIO_SET_PROMISCUOUS(0);
#endif /* SNIFFER_CONF_RADIO_SET_PROMISCUOUS */

    NETSTACK_MAC.on();
#if NETSTACK_RDC != multichan_driver
    NETSTACK_RDC.on();
#endif
  }
}

/*--------------------------------------------------------------------------*/
/**
 * \brief      Drop a capture
 */
void
sniffer_drop(void)
{
  if(sstate == SNIFFER_CAPTURED) {
    sstate = SNIFFER_ACTIVE;
  }
}
/*--------------------------------------------------------------------------*/
/**
 * \brief      Output (send, transmit, sth) a pcap capture.
 * \param pcapbuf    pointer to a PCAP capture buffer
 * \param len    lenght of the total PCAP capture
 */
void
sniffer_output(pcap_t *pcapbuf, uint16_t len)
{
  uint16_t k;
  /* Print out the capture in hex */
  sstate = SNIFFER_PRINTING;
  //  printf("Sniffer PCAP, len %u:", len);
  //  printf("%02x", len);

  /* Print out "magic string" for command line parser */
  printf("PCAP ");

  /* Print out the PCAP capture */
  for(k = 0; k < len; k += 1) {
    printf("%02x",  ((uint8_t*)pcapbuf)[k]);
  }
  printf("\n");
  sstate = SNIFFER_ACTIVE;
}
/*--------------------------------------------------------------------------*/
/* Prepare a capture for output as a pcap formatted frame. Assumes packet is in
 * packetbuf/uipbuf. Fills out PCAP headers and copies it to a buffer. The
 * PCAP global header is output by an external script as it must be written once
 * and once only.
 */
static void
pcap_prepare(pcap_t *cap, uint32_t linktype, void *src, uint16_t len)
{
  uint32_t tempclkfine;
  PRINTF("SnifferPrep: len %u t %lu\n", len, linktype);

  /* time deviation from thiszone in seconds */
  cap->caphdr.ts_sec = clock_seconds();

  /* time deviation from (thiszone+seconds) in microseconds */
  tempclkfine = clock_time() % CLOCK_SECOND;
  tempclkfine = (1000000 * tempclkfine)/CLOCK_SECOND;
/*  printf("Fine: %u %u %lu\n", clock_seconds(), clock_time(), tempclkfine);*/
  cap->caphdr.ts_usec = tempclkfine;

  /* Capture header and packet */
  if(len > CAPTURE_MAX_LEN) {
    /* packet was too large, so it got capped to snaplen bytes. */
    PRINTF("Sniffer: len is large: %u (%u)\n", len, CAPTURE_MAX_LEN);
    cap->caphdr.incl_len = CAPTURE_MAX_LEN;
    cap->caphdr.orig_len = len;
  } else {
    cap->caphdr.incl_len = len;
    cap->caphdr.orig_len = len;
  }
  memcpy(&(cap->buf), src, cap->caphdr.incl_len);

  /* Total size of data to output. */
  capdatalen = sizeof(pcap_caphdr_t) + cap->caphdr.incl_len;
}
/*--------------------------------------------------------------------------*/
/**
 * \brief      A packet has been captured, copy it early but handle output later
 * \param linktype    indicating what link layer capture occurred at
 * \param src    source, where the raw capture (no headers) resides
 * \param len    length of the data in the src buffer
 */
void
sniffer_input(uint32_t linktype, void* src, uint16_t len)
{
  PRINTF("SnifferPrep: %lu len %u\n", linktype, len);
  if(cap_cb == NULL || sstate != SNIFFER_ACTIVE) {
    /* Not active or no callback so just drop the capture. */
    PRINTF("Sniffer dropped as not active/NULL\n");
    return;
  }

  /* Copy the capture so that it won't get overwritten before we have a chance
   * to handle it.
   */
  pcap_prepare(&pcap, linktype, src, len);
#if 0
  process_poll(&sniffer_process);
#else
  if(cap_cb != NULL) {
    cap_cb(&pcap, capdatalen);
  } else {
    sniffer_output(&pcap, capdatalen);
  }
#endif
}
/*--------------------------------------------------------------------------*/
/* A packet is captured, invoke either a user registrered callback or
 * the default sniffer one.
 */
static void
pollhandler(void)
{
  if(cap_cb != NULL) {
    cap_cb(&pcap, capdatalen);
  } else {
    sniffer_output(&pcap, capdatalen);
  }
}
/*--------------------------------------------------------------------------*/
PROCESS_THREAD(sniffer_process, ev, data)
{
  PROCESS_EXITHANDLER(sniffer_stop(););
  PROCESS_POLLHANDLER(pollhandler(););
  PROCESS_BEGIN();
  PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_EXIT);  
  PROCESS_END();
}
/*--------------------------------------------------------------------------*/

#if INCLUDE_SHELLCONF
PROCESS(shell_snifferconf_process, "Sniffer Confpr.");
SHELL_COMMAND(snifferconf_command,
	      "sniffconf",
	      "sniffconf: Configure Sniffer. Use 'sniffconf ?' for available commands.",
	      &shell_snifferconf_process);
/*---------------------------------------------------------------------------*/
/* This process handles Contiki Shell conf command for Sniffer */
PROCESS_THREAD(shell_snifferconf_process, ev, data)
{
  const char *args;
  PROCESS_BEGIN();
  args = data;
  if(args == NULL) {
    PROCESS_EXIT();
  }

  if(args[0] == 's' || args[0] == 'S') {
    printf("Start sniffer\n");
    sniffer_start(sniffer_output);
  } else if(args[0] == 'q' || args[0] == 'Q') {
    printf("Stop sniffer\n");
    sniffer_stop();
  } else {
    printf("Sniffer\nUsage:\n\ts start\n\tq stop\n");
  }
  PROCESS_END();
}

#endif
/*---------------------------------------------------------------------------*/
/** @} */
