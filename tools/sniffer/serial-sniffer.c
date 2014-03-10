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
 *         Network sniffer that prints out sniffed packets on the serial port.
 * \author
 *         Marcus Lunden <marcus@thingsquare.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include "contiki.h"
#include "contiki-net.h"
#include "dev/leds.h"
#include "sniffer.h"
/* -------------------------------------------------------------------------- */
PROCESS(serial_sniffer_process, "Serial sniffer process");
AUTOSTART_PROCESSES(&serial_sniffer_process);
/* -------------------------------------------------------------------------- */
/* This callback is invoked at packet capture, to enable additional
 * measures such as filtering or saving to file. In this case, no
 * filtering is done, but the packet is simply printed to the serial
 * port.
 *
 * Example of filtering on link layer type:
 *
 *    if(c->hdr.network == LINKTYPE_IPV4) {
 *      cfs_write(fd, c, len);
 *    }
 *
 * Example of saving to local CFS file:
 *
 *    fd = cfs_open("my_captures.hex", CFS_WRITE | CFS_APPEND);
 *    cfs_write(fd, c, sizeof(pcap_t));
 *    cfs_close(fd);
 */
static void
capture_callback(pcap_t *c, uint16_t len)
{
  printf("# Sniffer PCAP, length: %u \n", len);
  /* Filtering etc can be done here, but we just want to print it
     out. */
  sniffer_output(c, len);
}

/* -------------------------------------------------------------------------- */
/* Application process, just start the Sniffer here. */
PROCESS_THREAD(serial_sniffer_process, ev, data)
{
  PROCESS_BEGIN();
  /* if called with NULL instead, sniffer_output() is automatically
   * invoked on capture
   */
  sniffer_init(capture_callback);

  /* If there is no traffic, here are examples of captured and printed
     packets */
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/** @} */
