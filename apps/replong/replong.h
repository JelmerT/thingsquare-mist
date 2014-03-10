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
#ifndef REPLONG_H
#define REPLONG_H

#include "net/uip.h"

void replong_init(void);

/* Request neighbors, wait for "seconds" seconds to collect responses,
   and send data to specified receiver. */
int replong_send_data_within(int seconds,
				const uint8_t *data, int data_len,
				const uip_ip6addr_t *receiver);

/* Specify a callback to be called when a data arrives. */
void replong_set_callback(void (* callback)(const uint8_t *data,
					    int data_len,
					    const uip_ipaddr_t *from,
					    const uip_ipaddr_t *via));

/* Send data to specified receiver via a known relay node. */
int replong_send_data(const uint8_t *data, int data_len,
			 const uip_ip6addr_t *receiver,
			 const uip_ip6addr_t *relay);

/* Send a whothere request for responses from neighbors. */
void replong_send_whothere(int answer_within_seconds);


/* Get the list of neighbors. */
struct replong_neighbor {
  struct replong_neighbor *next;
  uip_ipaddr_t ipaddr;
  struct ctimer killme_timer;
};
const struct replong_neighbor *replong_neighbors(void);

#endif /* REPLONG_H */
