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

/*
 *  \file
 *      contiki-conf.h
 *   \author
 *      Marcus Lunden <marcus@thingsquare.com>>
 *   \desc
 *      Contiki configuration for STM32L152VB + Spirit1 platform.
 *      
 */

#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

#ifdef INCLUDE_SUBPLATFORM_CONF
#include "subplatform-conf.h"
#else /* INCLUDE_SUBPLATFORM_CONF */
#include "platform-conf.h"
#endif /* INCLUDE_SUBPLATFORM_CONF */

#include "mist-conf-const.h"

#ifndef MIST_CONF_NETSTACK
#define MIST_CONF_NETSTACK (MIST_CONF_NULLRDC | MIST_CONF_AES)
#endif /* MIST_CONF_NETSTACK */

#include "mist-default-conf.h"

#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 8
#define NULLRDC_CONF_802154_AUTOACK 1
#define NETSTACK_CONF_FRAMER  framer_802154
#define NETSTACK_CONF_NETWORK sicslowpan_driver

#define RIMESTATS_CONF_ENABLED                      1
#define RIMESTATS_CONF_ON                      1


#if WITH_UIP6
/* Network setup for IPv6 */
#define NETSTACK_CONF_NETWORK sicslowpan_driver

/* Specify a minimum packet size for 6lowpan compression to be
   enabled. This is needed for ContikiMAC, which needs packets to be
   larger than a specified size, if no ContikiMAC header should be
   used. */
#define SICSLOWPAN_CONF_COMPRESSION_THRESHOLD 0
//#define SICSLOWPAN_CONF_MAC_MAX_PAYLOAD        40
#define CONTIKIMAC_CONF_WITH_CONTIKIMAC_HEADER 0

#define CXMAC_CONF_ANNOUNCEMENTS         0
#define XMAC_CONF_ANNOUNCEMENTS          0

#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM                8
#endif

#else /* WITH_UIP6 */

/* Network setup for non-IPv6 (rime). */

#error This platform is not yet tested without IPv6

#define NETSTACK_CONF_NETWORK rime_driver

#define COLLECT_CONF_ANNOUNCEMENTS       1
#define CXMAC_CONF_ANNOUNCEMENTS         0
#define XMAC_CONF_ANNOUNCEMENTS          0
#define CONTIKIMAC_CONF_ANNOUNCEMENTS    0

#define CONTIKIMAC_CONF_COMPOWER         1
#define XMAC_CONF_COMPOWER               1
#define CXMAC_CONF_COMPOWER              1

#ifndef COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS
#define COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS     32
#endif /* COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS */

#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM                8
#endif /* QUEUEBUF_CONF_NUM */

#ifndef TIMESYNCH_CONF_ENABLED
#define TIMESYNCH_CONF_ENABLED           0
#endif /* TIMESYNCH_CONF_ENABLED */

#endif /* WITH_UIP6 */

/* A trick to resolve a compilation error with IAR. */
#ifdef __ICCARM__
#define UIP_CONF_DS6_AADDR_NBU              1
#endif

/* radio driver blocks until ACK is received */
#define NULLRDC_CONF_ACK_WAIT_TIME          (0)

#ifndef RF_CHANNEL
#define RF_CHANNEL              26
#endif /* RF_CHANNEL */

#define CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT 0

#define IEEE802154_CONF_PANID       0xABCD

#define SHELL_VARS_CONF_RAM_BEGIN 0x1100
#define SHELL_VARS_CONF_RAM_END 0x2000

#define PROFILE_CONF_ON 0
#ifndef ENERGEST_CONF_ON
#define ENERGEST_CONF_ON 1
#endif /* ENERGEST_CONF_ON */

#define ELFLOADER_CONF_TEXT_IN_ROM 0
#ifndef ELFLOADER_CONF_DATAMEMORY_SIZE
#define ELFLOADER_CONF_DATAMEMORY_SIZE 0x400
#endif /* ELFLOADER_CONF_DATAMEMORY_SIZE */
#ifndef ELFLOADER_CONF_TEXTMEMORY_SIZE
#define ELFLOADER_CONF_TEXTMEMORY_SIZE 0x800
#endif /* ELFLOADER_CONF_TEXTMEMORY_SIZE */


#define AODV_COMPLIANCE
#define AODV_NUM_RT_ENTRIES 32

#define WITH_ASCII 1

#define PROCESS_CONF_NUMEVENTS 8
#define PROCESS_CONF_STATS 1
/*#define PROCESS_CONF_FASTPOLL    4*/

#ifdef WITH_UIP6

#define RIMEADDR_CONF_SIZE              8

#define UIP_CONF_LL_802154              1
#define UIP_CONF_LLH_LEN                0

#define UIP_CONF_ROUTER                 1
#ifndef UIP_CONF_IPV6_RPL
#define UIP_CONF_IPV6_RPL               1
#endif /* UIP_CONF_IPV6_RPL */

/* configure number of neighbors and routes */
#ifndef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_CONF_MAX_NEIGHBORS     30
#endif /* NBR_TABLE_CONF_MAX_NEIGHBORS */
#ifndef UIP_CONF_DS6_ROUTE_NBU
#define UIP_CONF_DS6_ROUTE_NBU   30
#endif /* UIP_CONF_DS6_ROUTE_NBU */

#define UIP_CONF_ND6_SEND_RA    0
#define UIP_CONF_ND6_REACHABLE_TIME     600000
#define UIP_CONF_ND6_RETRANS_TIMER      10000

#define UIP_CONF_IPV6                   1
#ifndef UIP_CONF_IPV6_QUEUE_PKT
#define UIP_CONF_IPV6_QUEUE_PKT         0
#endif /* UIP_CONF_IPV6_QUEUE_PKT */
#define UIP_CONF_IPV6_CHECKS            1
#define UIP_CONF_IPV6_REASSEMBLY        0
#define UIP_CONF_NETIF_MAX_ADDRESSES    3
#define UIP_CONF_ND6_MAX_PREFIXES       3
#define UIP_CONF_ND6_MAX_NEIGHBORS      4
#define UIP_CONF_ND6_MAX_DEFROUTERS     2
#define UIP_CONF_IP_FORWARD             0
#ifndef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE    280
#endif

#define SICSLOWPAN_CONF_COMPRESSION_IPV6        0
#define SICSLOWPAN_CONF_COMPRESSION_HC1         1
#define SICSLOWPAN_CONF_COMPRESSION_HC01        2
#define SICSLOWPAN_CONF_COMPRESSION             SICSLOWPAN_COMPRESSION_HC06
#ifndef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG                    1

/* Unit: 1/16th second. 4 => 0.25s timeout */
#undef SICSLOWPAN_CONF_MAXAGE
#define SICSLOWPAN_CONF_MAXAGE                  4

#if (MIST_CONF_NETSTACK & MIST_CONF_DROWSIE_MULTICHANNEL)
/* We need to increase the fragmentation timeout, as the multichannel protocol may transmit
 * the same fragment on two channels, causing up to 0.6s delay inbetween fragments. */
#undef SICSLOWPAN_CONF_MAXAGE
#define SICSLOWPAN_CONF_MAXAGE                  12
#endif /* (MIST_CONF_NETSTACK & MIST_CONF_DROWSIE_MULTICHANNEL) */

#endif /* SICSLOWPAN_CONF_FRAG */
#define SICSLOWPAN_CONF_CONVENTIONAL_MAC  1
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS       2
#ifndef SICSLOWPAN_CONF_MAX_MAC_TRANSMISSIONS
#define SICSLOWPAN_CONF_MAX_MAC_TRANSMISSIONS   5
#endif /* SICSLOWPAN_CONF_MAX_MAC_TRANSMISSIONS */
#else /* WITH_UIP6 */
#define UIP_CONF_IP_FORWARD      1
#define UIP_CONF_BUFFER_SIZE     108
#endif /* WITH_UIP6 */

#define UIP_CONF_ICMP_DEST_UNREACH 1

#define UIP_CONF_DHCP_LIGHT
#define UIP_CONF_LLH_LEN         0
#ifndef UIP_CONF_RECEIVE_WINDOW
#define UIP_CONF_RECEIVE_WINDOW  150
#endif
#ifndef UIP_CONF_TCP_MSS
#define UIP_CONF_TCP_MSS         UIP_CONF_RECEIVE_WINDOW
#endif
#define UIP_CONF_MAX_CONNECTIONS 4
#define UIP_CONF_MAX_LISTENPORTS 8
#define UIP_CONF_UDP_CONNS       12
#define UIP_CONF_FWCACHE_SIZE    30
#define UIP_CONF_BROADCAST       1
#define UIP_ARCH_IPCHKSUM        0
#define UIP_CONF_UDP             1
#define UIP_CONF_UDP_CHECKSUMS   1
#define UIP_CONF_PINGADDRCONF    0
#define UIP_CONF_LOGGING         0

#define UIP_CONF_TCP_SPLIT       0

/* include the project config */
/* PROJECT_CONF_H might be defined in the project Makefile */
#ifdef PROJECT_CONF_H
#include PROJECT_CONF_H
#endif /* PROJECT_CONF_H */

#if ((MIST_CONF_NETSTACK) & MIST_CONF_AES)
#ifndef NETSTACK_AES_KEY
#define NETSTACK_AES_KEY "thingsquare mist" /* 16 bytes */
#define NETSTACK_AES_KEY_DEFAULT 1
#endif /* NETSTACK_AES_KEY */
#endif /* ((MIST_CONF_NETSTACK) & MIST_CONF_AES) */

#define UART1_CONF_TX_WITH_INTERRUPT        0
#define WITH_SERIAL_LINE_INPUT              1
#undef ENERGEST_CONF_ON
#define ENERGEST_CONF_ON                    0
#define TELNETD_CONF_NUMLINES               6
#define NETSTACK_CONF_RADIO                 spirit_radio_driver

#undef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE            300
#undef UIP_CONF_TCP_MSS
#define UIP_CONF_TCP_MSS                (UIP_CONF_BUFFER_SIZE-70)
#undef UIP_CONF_RECEIVE_WINDOW
#define UIP_CONF_RECEIVE_WINDOW         (UIP_CONF_BUFFER_SIZE-70)
#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_CONF_MAX_NEIGHBORS            8
#undef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES             8
#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM               10
#undef PACKETBUF_CONF_SIZE
#define PACKETBUF_CONF_SIZE             UIP_CONF_BUFFER_SIZE
#define NETSTACK_RADIO_MAX_PAYLOAD_LEN  PACKETBUF_CONF_SIZE /* spirit1-config.h */


#endif /* CONTIKI_CONF_H */
