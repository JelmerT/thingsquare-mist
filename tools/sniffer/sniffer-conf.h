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
 *         Configuration file for the sniffer.
 * \author
 *         Marcus Lunden <marcus@thingsquare.com>
 */

#ifndef SNIFFER_CONF_H
#define SNIFFER_CONF_H

#include "contiki-conf.h"

#if CC11xx_H || CONTIKI_TARGET_TRXEB1120 || CONTIKI_TARGET_ETH1120
#define SNIFFER_CONF_RADIO_SET_PROMISCUOUS cc11xx_set_promiscuous
#endif /* CC11xx_H */

#ifdef CC1101_H
#define SNIFFER_CONF_RADIO_SET_PROMISCUOUS cc1101_set_promiscuous
#endif /* CC1101_H */

#if CONTIKI_TARGET_MIST_MB851
void stm32w_radio_set_promiscous(uint8_t on);
#define SNIFFER_CONF_RADIO_SET_PROMISCUOUS stm32w_radio_set_promiscous
#endif /* CONTIKI_TARGET_MIST_MB851 */

#ifdef MIST_CONF_NETSTACK
#if (MIST_CONF_NETSTACK & MIST_CONF_MULTICHAN)
#define SNIFFER_STRIP_MULTICHAN_HEADER 1
#endif /* (MIST_CONF_NETSTACK & MIST_CONF_MULTICHAN) */
#endif /* MIST_CONF_NETSTACK */

#endif /* SNIFFER_CONF_H */
/** @} */
