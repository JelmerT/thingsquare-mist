/*
 * Copyright (c) 2013, Thingsquare, http://www.thingsquare.com/.
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

#include "mist.h"
#include "propple-socket.h"


struct propple_hdr {
  uint16_t datalen;
  uint8_t seqno;
  uint8_t dummy;
};

struct propple_msg {
  struct propple_hdr h;
  uint8_t data[PROPPLE_MAXDATALEN];
};

static int run_trickle(struct propple_socket *s);

#define SEQNO_LT(a, b) ((signed char)((a) - (b)) < 0)

/*---------------------------------------------------------------------------*/
static void
send(void *ptr)
{
  struct propple_socket *s = ptr;
  struct propple_msg m;
  uip_ipaddr_t addr;

  m.h.seqno = s->seqno;
  m.h.datalen = s->datalen;
  memcpy(m.data, s->data, s->datalen);

  uip_create_linklocal_allnodes_mcast(&addr);

  printf("propple-socket: send sending\n");
  if(udp_socket_sendto(&s->s,
                       &m, sizeof(struct propple_hdr) + s->datalen,
                       &addr, s->port) == 0) {
    printf("propple-socket: send: failed\n");
  } else {
    printf("propple-socket: send: sending %d bytes\n",
           sizeof(struct propple_hdr) + s->datalen);
  }
}
/*---------------------------------------------------------------------------*/
static void
timer_callback(void *ptr)
{
  struct propple_socket *s = ptr;
  run_trickle(s);
}
/*---------------------------------------------------------------------------*/
static void
reset_interval(struct propple_socket *s)
{
  PT_INIT(&s->pt);
  run_trickle(s);
}
/*---------------------------------------------------------------------------*/
static void
set_timer(struct propple_socket *s,
          struct ctimer *t, clock_time_t i)
{
  ctimer_set(t, i, timer_callback, s);
}
/*---------------------------------------------------------------------------*/
static int
run_trickle(struct propple_socket *s)
{
  clock_time_t interval;
  PT_BEGIN(&s->pt);

  while(1) {
    interval = s->interval << s->interval_scaling;
    if(interval > s->interval_max) {
      interval = s->interval_max;
    }
    set_timer(s, &s->interval_timer, interval);
    set_timer(s, &s->t, interval / 2 + (random_rand() % (interval / 2)));

    s->duplicates = 0;
    PT_YIELD(&s->pt); /* Wait until listen timeout */
    if(s->duplicates < s->duptheshold) {
      send(s);
    }
    interval = s->interval << s->interval_scaling;
    if(interval < s->interval_max) {
      s->interval_scaling++;
    }
    PT_YIELD(&s->pt); /* Wait until interval timer expired. */
  }

  PT_END(&s->pt);
}
/*---------------------------------------------------------------------------*/
static void
recv(struct udp_socket *c, void *ptr,
     const uip_ipaddr_t *source_addr, uint16_t source_port,
     const uip_ipaddr_t *dest_addr, uint16_t dest_port,
     const uint8_t *data, uint16_t datalen)
{
  struct propple_socket *s = ptr;
  struct propple_msg *m = (struct propple_msg *)data;
  struct propple_hdr h;

  memcpy((uint8_t *)&h, (uint8_t *)&m->h, sizeof(h));

  uint16_t seqno = h.seqno;

  printf("propple-socket: recv seqno %d our %d data len %d\n",
	 seqno,
         s->seqno,
	 datalen);

  if(seqno == s->seqno) {
    /*    c->cb->recv(c);*/
    ++s->duplicates;
  } else if(SEQNO_LT(seqno, s->seqno)) {
    s->interval_scaling = 0;
    send(s);
  } else { /* hdr->seqno > c->seqno */
    /* Sanity check datalen */
    if(h.datalen <= PROPPLE_MAXDATALEN) {
      s->seqno = seqno;
      /* Remember the incoming data */
      memcpy(s->data, m->data, h.datalen);
      s->datalen = h.datalen;
      s->interval_scaling = 0;
      reset_interval(s);
      ctimer_set(&s->first_transmission_timer,
                 random_rand() % s->interval,
                 send, s);

      if(s->callback != NULL) {
        s->callback(s, s->ptr, s->seqno,
                    s->data, s->datalen);
      }
      /*    c->cb->recv(c);*/
    }
  }
}
/*---------------------------------------------------------------------------*/
int
propple_socket_open(struct propple_socket *s,
                    void *ptr,
                    propple_socket_input_callback_t callback,
                    uint16_t port,
                    clock_time_t interval,
                    clock_time_t interval_max,
                    uint8_t duptheshold)
{
  s->ptr = ptr;
  s->callback = callback;
  s->interval = interval;
  s->interval_max = interval_max;
  s->duptheshold = duptheshold;
  s->port = port;

  s->duplicates = 0;
  s->interval_scaling = 0;

  /* Register UDP socket callback */
  if(udp_socket_register(&s->s, s, recv) == 0) {
    printf("propple_socket_open: udp_socket_register failed\n");
    return 0;
  }

  /* Bind UDP socket to local port */
  if(udp_socket_bind(&s->s, port) == 0) {
    printf("propple_socket_open: udp_socket_bind failed\n");
    return 0;
  }

  /* Connect UDP socket to remote port */
  if(udp_socket_connect(&s->s, NULL, port) == 0) {
    printf("propple_socket_open: udp_socket_connect failed\n");
    return 0;
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
int
propple_socket_send(struct propple_socket *s,
                    const uint8_t *data,
                    uint16_t datalen)

{
  if(datalen > PROPPLE_MAXDATALEN) {
    datalen = PROPPLE_MAXDATALEN;
  }
  memcpy(s->data, data, datalen);
  s->datalen = datalen;
  s->seqno++;
  reset_interval(s);
  send(s);
  printf("propple_socket_send seqno %d, datalen %d data '%s'\n", s->seqno,
         s->datalen, data);
  return datalen;
}
/*---------------------------------------------------------------------------*/

