/**
 * \addtogroup netsynch
 * @{
 */


/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: netsynch.c,v 1.11 2010/12/16 22:47:38 adamdunkels Exp $
 */

/**
 * \file
 *         A simple time synchronization mechanism
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "contiki-net.h"
#include "lib/random.h"
#include <string.h>
#include <limits.h>

#define AUTHORITY_LEVEL_UNSYNCHED 200
static int authority_level = AUTHORITY_LEVEL_UNSYNCHED;
static signed long offset;

#define UDP_PORT  1077

struct netsynch_msg {
  uint16_t authority_level;
  int16_t authority_offset;
  clock_time_t clock_time;
};

#define MIN_INTERVAL CLOCK_SECOND * 8
#define MAX_INTERVAL CLOCK_SECOND * 60 * 5
/*---------------------------------------------------------------------------*/
PROCESS(netsynch_process, "Netsynch process");
/*---------------------------------------------------------------------------*/
int
netsynch_authority_level(void)
{
  return authority_level;
}
/*---------------------------------------------------------------------------*/
int
netsynch_is_synched(void)
{
  return authority_level < AUTHORITY_LEVEL_UNSYNCHED;
}
/*---------------------------------------------------------------------------*/
void
netsynch_set_authority_level(int level)
{
  int old_level = authority_level;

  authority_level = level;

  if(old_level != authority_level) {
    /* Restart the netsynch process to restart with a low
       transmission interval. */
    process_exit(&netsynch_process);
    process_start(&netsynch_process, NULL);
  }
}
/*---------------------------------------------------------------------------*/
clock_time_t
netsynch_time(void)
{
  return clock_time() + offset;
}
/*---------------------------------------------------------------------------*/
clock_time_t
netsynch_time_to_clock_time(clock_time_t synched_time)
{
  return synched_time - offset;
}
/*---------------------------------------------------------------------------*/
clock_time_t
netsynch_clock_time_to_time(clock_time_t time)
{
  return time + offset;
}
/*---------------------------------------------------------------------------*/
clock_time_t
netsynch_offset(void)
{
  return offset;
}
/*---------------------------------------------------------------------------*/
static void
adjust_offset(clock_time_t authoritative_time, clock_time_t local_time)
{
  offset = authoritative_time - local_time;
}
/*---------------------------------------------------------------------------*/
static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
  struct netsynch_msg msg;

  memcpy(&msg, uip_appdata, sizeof(msg));

  /* We check the authority level of the sender of the incoming
       packet. If the sending node has a lower authority level than we
       have, we synchronize to the time of the sending node and set our
       own authority level to be one more than the sending node. */
  if(msg.authority_level < authority_level) {
    /*    adjust_offset(msg.timestamp + msg.authority_offset,
	  packetbuf_attr(PACKETBUF_ATTR_TIMESTAMP));*/
    adjust_offset(msg.clock_time + msg.authority_offset,
		  clock_time());

    netsynch_set_authority_level(msg.authority_level + 1);
  }
}

static struct simple_udp_connection broadcast_connection;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(netsynch_process, ev, data)
{
  static struct etimer sendtimer, intervaltimer;
  static clock_time_t interval;
  struct netsynch_msg msg;
  uip_ipaddr_t addr;


  PROCESS_BEGIN();

  simple_udp_register(&broadcast_connection, UDP_PORT,
                      NULL, UDP_PORT, receiver);

  interval = MIN_INTERVAL;

  while(1) {
    etimer_set(&intervaltimer, interval);
    etimer_set(&sendtimer, random_rand() % interval);

    PROCESS_WAIT_UNTIL(etimer_expired(&sendtimer));

    msg.authority_level = authority_level;
    msg.authority_offset = offset;
    msg.clock_time = clock_time();
    /*    packetbuf_copyfrom(&msg, sizeof(msg));
    packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,
    PACKETBUF_ATTR_PACKET_TYPE_TIMESTAMP);*/
    /*    broadcast_send(&broadcast);*/
    uip_create_linklocal_allnodes_mcast(&addr);
    simple_udp_sendto(&broadcast_connection, &msg, sizeof(msg), &addr);

    PROCESS_WAIT_UNTIL(etimer_expired(&intervaltimer));
    interval *= 2;
    if(interval >= MAX_INTERVAL) {
      interval = MAX_INTERVAL;
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
netsynch_init(void)
{
  process_start(&netsynch_process, NULL);
}
/*---------------------------------------------------------------------------*/

/** @} */
