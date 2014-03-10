/*
 * Copyright (c) 2012, Thingsquare, www.thingsquare.com.
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
 */

#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "net/uip.h"
#include "net/uip-ds6.h"
#include "dev/leds.h"

#include "net/simple-udp.h"
#include "node-id.h"

#include "replong.h"

#include "net/rime/rimestats.h"

#include <stdio.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
#define UDP_PORT 124
#define SEND_INTERVAL (CLOCK_SECOND / 2)

#ifdef UDP_UNICAST_DESTINATION_IP
#define _QUOTEME(x) #x
#define QUOTEME(x) _QUOTEME(x)
#define DESTINATION_IP QUOTEME(UDP_UNICAST_DESTINATION_IP)
#endif /* UDP_UNICAST_DESTINATION_IP */

/* Configuration */
#define MIN_SIZE 1
#define MAX_SIZE 110
#define PRINT_BLOCK_SIZE 10

#define NR_LOGS (MAX_SIZE - MIN_SIZE + 1)

#if (MIST_CONF_NETSTACK & MIST_CONF_MULTICHAN)
void multichan_force_auth(int become_auth); /* multichan.c */
#endif /* (MIST_CONF_NETSTACK & MIST_CONF_MULTICHAN) */
#if (MIST_CONF_NETSTACK & MIST_CONF_DROWSIE_MULTICHANNEL)
void drowsie_multichannel_force_auth(int become_auth); /* drowsie-multichannel.c */
#endif /* (MIST_CONF_NETSTACK & MIST_CONF_DROWSIE_MULTICHANNEL) */

/*---------------------------------------------------------------------------*/
static struct simple_udp_connection unicast_connection;
/*---------------------------------------------------------------------------*/
PROCESS(udp_unicast_process, "Test UDP unicast process");
AUTOSTART_PROCESSES(&udp_unicast_process);
/*---------------------------------------------------------------------------*/
static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr, uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr, uint16_t receiver_port,
         const uint8_t *data, uint16_t datalen)
{
  /*printf("RX port %d from port %d with length %d\n", receiver_port,
     sender_port, datalen);*/
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_unicast_process, ev, data)
{
  static struct etimer periodic_timer;
  static uip_ipaddr_t addr;

  PROCESS_BEGIN();

  /* Note: set destination to NULL, to allow receiving packets from anyone */
  simple_udp_register(&unicast_connection, UDP_PORT, NULL, UDP_PORT,
                      receiver);

#if (MIST_CONF_NETSTACK & MIST_CONF_MULTICHAN)
  multichan_force_auth(1);
#endif /* (MIST_CONF_NETSTACK & MIST_CONF_MULTICHAN) */
#if (MIST_CONF_NETSTACK & MIST_CONF_DROWSIE_MULTICHANNEL)
  drowsie_multichannel_force_auth(1);
#endif /* (MIST_CONF_NETSTACK & MIST_CONF_DROWSIE_MULTICHANNEL) */

  replong_init();

#ifdef DESTINATION_IP
  uiplib_ip6addrconv(DESTINATION_IP, &addr);
#else /* DESTINATION_IP */

  const struct replong_neighbor *n;

  do {
    /* Use replong to find a random neighbor to send to. */
#define NEIGHBOR_COLLECTION_TIME 10
    replong_send_whothere(NEIGHBOR_COLLECTION_TIME);
    etimer_set(&periodic_timer, CLOCK_SECOND * NEIGHBOR_COLLECTION_TIME);
    PROCESS_WAIT_UNTIL(etimer_expired(&periodic_timer));

    /* Check that we have received at least one neighbor. If so, we
       pick the first one. If no neighbors were found, we repeat the
       replong. */
    n = replong_neighbors();
  } while(n == NULL);
  printf("Found a destination: ");
  uip_debug_ipaddr_print(&n->ipaddr);
  printf("\n");
  uip_ipaddr_copy(&addr, &n->ipaddr);

#endif /* DESTINATION_IP */

  etimer_set(&periodic_timer, SEND_INTERVAL);
  while(1) {
    static char buf[MAX_SIZE + 1];
    static int current_size = MIN_SIZE;

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
    etimer_reset(&periodic_timer);


    int i;

    current_size++;
    if(current_size > MAX_SIZE) {
      current_size = MIN_SIZE;
    }

    for(i = 0; i < current_size; i++) {
      buf[i] = 'a';
    }
    buf[current_size] = '\0';

#if RIMESTATS_CONF_ON
    printf("ACKS: %ld/%ld %d%%\n", rimestats.ackrx, rimestats.reliabletx, (int) ((100*rimestats.ackrx)/rimestats.reliabletx));
#endif
    printf("TX[%02d]: '%s'\n", current_size, buf);

      /*{
        int i;
        printf("Sending unicast to: ");
        for(i = 0; i < 7; ++i) {
          printf("%02x%02x:", addr.u8[i * 2], addr.u8[i * 2 + 1]);
        }
        printf("%02x%02x\n", addr.u8[7 * 2], addr.u8[7 * 2 + 1]);
      }*/

    leds_toggle(LEDS_GREEN);
    simple_udp_sendto(&unicast_connection, buf, current_size, &addr);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
