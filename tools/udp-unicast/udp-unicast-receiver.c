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
  static uint32_t ok[NR_LOGS] = { 0 }, fails[NR_LOGS] = { 0 };
  static int expected_len = -1;
  static char buf[MAX_SIZE + 1];
  int i;

  if(datalen < MIN_SIZE || datalen > MAX_SIZE) {
    printf("warning: bad datalen: %d\n", datalen);
    return;
  }

  leds_toggle(LEDS_RED);

  /* Print received packet */
  memcpy(buf, data, datalen);
  buf[datalen] = '\0';
  printf("RX[%02d]: '%s'\n", datalen, buf);

  /* Estimate success ratio */
  if(expected_len >= MIN_SIZE && expected_len != datalen) {
    /* Estimate number of missed packets */
    int guess = expected_len;
    while(guess != datalen) {
      printf("missed packet\n");
      fails[guess - MIN_SIZE]++;
      guess++;
      if(guess > MAX_SIZE) {
        guess = MIN_SIZE;
      }
    }
  }
  ok[datalen - MIN_SIZE]++;

  /* Print stats */
  for(i = MIN_SIZE; i <= MAX_SIZE; i += PRINT_BLOCK_SIZE) {
    int from, to;
    uint32_t ok_sum, fails_sum;
    int j;

    from = i;
    to = from + PRINT_BLOCK_SIZE;
    if(to > MAX_SIZE) {
      to = MAX_SIZE + 1;
    }
    ok_sum = 0;
    fails_sum = 0;
    for(j = from; j < to; j++) {
      ok_sum += ok[j - MIN_SIZE];
      fails_sum += fails[j - MIN_SIZE];
    }

    if(datalen >= from && datalen <= to - 1) {
      printf(" *");
    } else {
      printf("  ");
    }
    printf("Results %02d-%02d: ", from, to-1);
    if(ok_sum + fails_sum == 0) {
      printf("----/----: --%%");
    } else {
      printf("%04d/%04d: %03d%%", (int)ok_sum, (int)(ok_sum + fails_sum),
             (int)((100L * (long)ok_sum) / ((long)ok_sum + (long)fails_sum)));
    }

    printf("\n");
  }

  /* Guess next packet length */
  expected_len = datalen + 1;
  if(expected_len > MAX_SIZE) {
    expected_len = MIN_SIZE;
  }

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

  replong_init();


  while(1) {
    PROCESS_WAIT_EVENT();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
