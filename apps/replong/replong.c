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
/* Replong: receive a data packet over a link-layer broadcast. The
   data packet starts with an IPv6 address of the final
   receiver. We relay the packet through a unicast connection. */

#include "contiki.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "net/uip.h"
#include "net/uip-ds6.h"
#include "net/uip-debug.h"

#include "net/simple-udp.h"

#include "lib/list.h"
#include "lib/memb.h"
#include "lib/random.h"

#include "replong.h"

#include <stdio.h>
#include <string.h>

#define DATA_LENGTH 32

enum {
  REPLONG_WHOTHERE,
  REPLONG_IMHERE,
  REPLONG_DATA,
};

struct replong_format {
  uip_ip6addr_t receiver;
  uip_ip6addr_t relay;
  uip_ip6addr_t sender;
  uint8_t replong_type;
  uint8_t data_len_or_wait_time;
  uint8_t data[DATA_LENGTH];
};

#define REPLONG_BROADCAST_PORT 53610
#define REPLONG_UNICAST_PORT   53611

static struct simple_udp_connection broadcast_connection;
static struct simple_udp_connection unicast_connection;

#define MAX_NEIGHBORS 8

MEMB(neighborsmemb, struct replong_neighbor, MAX_NEIGHBORS);
LIST(neighborlist);

static uip_ipaddr_t who_wants_to_know;
static struct ctimer whothere_reply_timer;

#define NEIGHBOR_LIFETIME (CLOCK_SECOND * 60)

static uint8_t delayed_data[DATA_LENGTH];
static int delayed_data_len;
static uip_ipaddr_t delayed_data_receiver;
static struct ctimer send_delayed_data_timer;

static void (* data_callback)(const uint8_t *data,
				 int data_len,
				 const uip_ipaddr_t *from,
				 const uip_ipaddr_t *via);

/*---------------------------------------------------------------------------*/
PROCESS(replong_process, "UDP broadcast example process");
/*---------------------------------------------------------------------------*/
void
replong_set_callback(void (* callback)(const uint8_t *data,
				       int data_len,
				       const uip_ipaddr_t *from,
				       const uip_ipaddr_t *via))
{
  data_callback = callback;
}
/*---------------------------------------------------------------------------*/
static void
whothere_reply(void *dummy)
{
  struct replong_format fmt;
  memset(&fmt, 0, sizeof(struct replong_format));
  fmt.replong_type = REPLONG_IMHERE;

  simple_udp_sendto(&unicast_connection, &fmt,
		    sizeof(struct replong_format), &who_wants_to_know);
}
/*---------------------------------------------------------------------------*/
static void
kill_neighbor(void *nptr)
{
  struct replong_neighbor *n = nptr;

  list_remove(neighborlist, n);
  memb_free(&neighborsmemb, n);
}
/*---------------------------------------------------------------------------*/
static void
unicast_receiver(struct simple_udp_connection *c,
		 const uip_ipaddr_t *sender_addr,
		 uint16_t sender_port,
		 const uip_ipaddr_t *receiver_addr,
		 uint16_t receiver_port,
		 const uint8_t *data,
		 uint16_t datalen)
{
  struct replong_format fmt;
  struct replong_neighbor *n;

  printf("Replong unicast received\n");
  if(uip_datalen() != sizeof(struct replong_format)) {
    printf("replong broadcast_receiver: packet too short, should be %d was %d\n",
	   sizeof(struct replong_format), uip_datalen());
  }
  memcpy(&fmt, uip_appdata, sizeof(struct replong_format));
  /* Unicast replongs can only be data or imhere messages. */
  switch(fmt.replong_type) {
  case REPLONG_IMHERE:
    printf("Replong: Got imhere from ");
    uip_debug_ipaddr_print(sender_addr);
    printf("\n");
    n = memb_alloc(&neighborsmemb);
    if(n != NULL) {
      uip_ipaddr_copy(&n->ipaddr, sender_addr);
      ctimer_set(&n->killme_timer, NEIGHBOR_LIFETIME,
		 kill_neighbor, n);
      list_add(neighborlist, n);
    }
    break;
  case REPLONG_DATA:
    printf("Replong unicast_receiver got %s\n", fmt.data);
    if(data_callback) {
      data_callback(fmt.data, fmt.data_len_or_wait_time,
		       &fmt.sender, &fmt.relay);
    }
    break;
  }
}

/*---------------------------------------------------------------------------*/
static void
broadcast_receiver(struct simple_udp_connection *c,
		   const uip_ipaddr_t *sender_addr,
		   uint16_t sender_port,
		   const uip_ipaddr_t *receiver_addr,
		   uint16_t receiver_port,
		   const uint8_t *data,
		   uint16_t datalen)
{
  struct replong_format fmt;
  uip_ds6_addr_t *ds6addr;

  printf("Replong broadcast received\n");
  if(uip_datalen() != sizeof(struct replong_format)) {
    printf("replong broadcast_receiver: packet too short, should be %d was %d\n",
	   sizeof(struct replong_format), uip_datalen());
  }
  memcpy(&fmt, uip_appdata, sizeof(struct replong_format));

  /* Check message type. */

  switch(fmt.replong_type) {
  case REPLONG_WHOTHERE:
    printf("Replong: Got whothere\n");
    ctimer_set(&whothere_reply_timer,
	       ((random_rand()) / 7) % (CLOCK_SECOND * fmt.data_len_or_wait_time),
	       whothere_reply, NULL);
    uip_ipaddr_copy(&who_wants_to_know, sender_addr);
    break;
  case REPLONG_DATA:
    uip_ip6addr_copy(&fmt.sender, sender_addr);

    ds6addr = uip_ds6_get_global(ADDR_PREFERRED);
    if(ds6addr != NULL) {
      uip_ip6addr_copy(&fmt.relay, &ds6addr->ipaddr);
    }

    printf("Replong receiver ");
    uip_debug_ipaddr_print(&fmt.receiver);
    printf(" sender ");
    uip_debug_ipaddr_print(&fmt.sender);
    printf(" relay ");
    uip_debug_ipaddr_print(&fmt.relay);
    printf("\n");
    simple_udp_sendto(&unicast_connection, &fmt, sizeof(struct replong_format),
		      &fmt.receiver);
    break;
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(replong_process, ev, data)
{
  static struct etimer periodic_timer;
  static struct etimer send_timer;
  uip_ipaddr_t addr;

  PROCESS_BEGIN();

  simple_udp_register(&broadcast_connection, REPLONG_BROADCAST_PORT,
                      NULL, REPLONG_BROADCAST_PORT,
                      broadcast_receiver);

  simple_udp_register(&unicast_connection, REPLONG_UNICAST_PORT,
		      NULL, REPLONG_UNICAST_PORT,
                      unicast_receiver);

  while(1) {
    PROCESS_WAIT_EVENT();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
replong_init(void)
{
  list_init(neighborlist);
  memb_init(&neighborsmemb);
  process_start(&replong_process, NULL);
}
/*---------------------------------------------------------------------------*/
int
replong_send_data(const uint8_t *data, int data_len,
		     const uip_ip6addr_t *receiver,
		     const uip_ip6addr_t *relay)
{
  struct replong_format fmt;

  memset(&fmt, 0, sizeof(struct replong_format));
  uip_ip6addr_copy(&fmt.receiver, receiver);
  uip_ip6addr_copy(&fmt.relay, relay);
  if(data_len > DATA_LENGTH) {
    data_len = DATA_LENGTH;
  }
  fmt.data_len_or_wait_time = data_len;
  memcpy(&fmt.data, data, data_len);
  fmt.replong_type = REPLONG_DATA;
  simple_udp_sendto(&broadcast_connection, &fmt,
		    sizeof(struct replong_format), relay);
  return data_len;
}
/*---------------------------------------------------------------------------*/
const struct replong_neighbor *
replong_neighbors(void)
{
  return list_head(neighborlist);
}
/*---------------------------------------------------------------------------*/
void
replong_send_whothere(int answer_within_seconds)
{
  struct replong_format fmt;
  uip_ipaddr_t bcaddr;

  memset(&fmt, 0, sizeof(struct replong_format));
  fmt.replong_type = REPLONG_WHOTHERE;
  fmt.data_len_or_wait_time = answer_within_seconds;
  uip_create_linklocal_allnodes_mcast(&bcaddr);
  simple_udp_sendto(&broadcast_connection, &fmt,
		    sizeof(struct replong_format), &bcaddr);

  printf("Whothere sent\n");
}
/*---------------------------------------------------------------------------*/
static void
send_delayed_data(void *dummy)
{
  const struct replong_neighbor *n;

  n = replong_neighbors();

  /* We should have received a bunch of neighbors in response to our
     whothere request. We'll send the data to the first of them. */
  if(n != NULL) {
    replong_send_data(delayed_data, delayed_data_len,
			 &delayed_data_receiver,
			 &n->ipaddr);
  }
}
/*---------------------------------------------------------------------------*/
int
replong_send_data_within(int seconds,
			    const uint8_t *data, int data_len,
			    const uip_ip6addr_t *receiver)
{
  if(data_len > DATA_LENGTH) {
    data_len = DATA_LENGTH;
  }
  memcpy(delayed_data, data, data_len);
  delayed_data_len = data_len;
  uip_ipaddr_copy(&delayed_data_receiver, receiver);
  replong_send_whothere(seconds);
  ctimer_set(&send_delayed_data_timer, CLOCK_SECOND * seconds,
	     send_delayed_data, NULL);
  return data_len;
}
/*---------------------------------------------------------------------------*/
