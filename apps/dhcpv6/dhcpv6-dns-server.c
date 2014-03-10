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
 * \addtogroup dhcpv6-dns-server
 *
 * @{
 */

/**
 * \file
 *         Rudimentary DHCPv6 server for setting DNS server
 * \author
 *         Adam Dunkels <adam@dunkels.com>
 *
 */

#include "dhcpv6-dns.h"
#include "dhcpv6-dns-server.h"
#include "contiki-net.h"

static uip_ipaddr_t dns_server;
static uint8_t dns_server_specified;

static struct simple_udp_connection server_connection;

/*---------------------------------------------------------------------------*/
void
dhcpv6_dns_server_set_server(const uip_ipaddr_t *ipaddr)
{
  if(ipaddr == NULL) {
    dns_server_specified = 0;
  } else {
    dns_server_specified = 1;
    uip_ipaddr_copy(&dns_server, ipaddr);
  }
}
/*---------------------------------------------------------------------------*/
static void
receive(struct simple_udp_connection *c,
	const uip_ipaddr_t *sender_addr,
	uint16_t sender_port,
	const uip_ipaddr_t *receiver_addr,
	uint16_t receiver_port,
	const uint8_t *data,
	uint16_t datalen)
{
  printf("Data received on port %d from port %d with length %d\n",
         receiver_port, sender_port, datalen);
}
/*---------------------------------------------------------------------------*/
void
dhcpv6_dns_server_init(void)
{
  dns_server_specified = 0;
  simple_udp_register(&server_connection,
		      DHCPV6_SERVER_PORT,
                      NULL,
		      DHCPV6_CLIENT_PORT,
                      receive);
}
/*---------------------------------------------------------------------------*/
/** @} */

/*

RFC 3315                     DHCP for IPv6                     July 2003

  1.2. Client-server Exchanges Involving Two Messages

   When a DHCP client does not need to have a DHCP server assign it IP
   addresses, the client can obtain configuration information such as a
   list of available DNS servers [20] or NTP servers [21] through a
   single message and reply exchanged with a DHCP server.  To obtain
   configuration information the client first sends an
   Information-Request message to the All_DHCP_Relay_Agents_and_Servers
   multicast address.  Servers respond with a Reply message containing
   the configuration information for the client.

   This message exchange assumes that the client requires only
   configuration information and does not require the assignment of any
   IPv6 addresses.

   When a server has IPv6 addresses and other configuration information
   committed to a client, the client and server may be able to complete
   the exchange using only two messages, instead of four messages as
   described in the next section.  In this case, the client sends a
   Solicit message to the All_DHCP_Relay_Agents_and_Servers requesting
   the assignment of addresses and other configuration information.
   This message includes an indication that the client is willing to
   accept an immediate Reply message from the server.  The server that
   is willing to commit the assignment of addresses to the client
   immediately responds with a Reply message.  The configuration
   information and the addresses in the Reply message are then
   immediately available for use by the client.

   Each address assigned to the client has associated preferred and
   valid lifetimes specified by the server.  To request an extension of
   the lifetimes assigned to an address, the client sends a Renew
   message to the server.  The server sends a Reply message to the
   client with the new lifetimes, allowing the client to continue to use
   the address without interruption.
*/
