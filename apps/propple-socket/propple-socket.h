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

#ifndef PROPPLE_SOCKET_H
#define PROPPLE_SOCKET_H

#include "udp-socket.h"

struct propple_socket;

typedef void (* propple_socket_input_callback_t)(struct propple_socket *c,
                                                 void *ptr,
                                                 uint8_t seqno,
                                                 const uint8_t *data,
                                                 uint16_t datalen);
#define PROPPLE_MAXDATALEN 80

struct propple_socket {
  struct udp_socket s;
  uint16_t port;
  propple_socket_input_callback_t callback;
  void *ptr;
  struct ctimer t, interval_timer, first_transmission_timer;
  struct pt pt;
  clock_time_t interval, interval_max;
  uint8_t seqno;
  uint8_t interval_scaling;
  uint8_t duplicates;
  uint8_t duptheshold;
  uint16_t datalen;
  uint8_t data[PROPPLE_MAXDATALEN];
};

int propple_socket_open(struct propple_socket *s,
                        void *ptr,
                        propple_socket_input_callback_t callback,
                        uint16_t port,
                        clock_time_t interval,
                        clock_time_t interval_max,
                        uint8_t duptheshold);

int propple_socket_send(struct propple_socket *s,
                        const uint8_t *data,
                        uint16_t datalen);

#endif /* PROPPLE_SOCKET_H */
