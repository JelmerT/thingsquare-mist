#ifndef PLATFORM_CONF_H
#define PLATFORM_CONF_H

#undef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE 240

#undef UIP_CONF_RECEIVE_WINDOW
#define UIP_CONF_RECEIVE_WINDOW  48
//#define UIP_CONF_RECEIVE_WINDOW  (UIP_CONF_BUFFER_SIZE - 60)

#undef UIP_CONF_TCP_MSS
#define UIP_CONF_TCP_MSS         48
//#define UIP_CONF_TCP_MSS       (UIP_CONF_BUFFER_SIZE - 60)

#undef UIP_CONF_TCP_SPLIT
#define UIP_CONF_TCP_SPLIT 1
#undef UIP_SPLIT_CONF_SIZE
#define UIP_SPLIT_CONF_SIZE 8


/* Unit: 1/16th second. 4 => 0.25s timeout */
#undef SICSLOWPAN_CONF_MAXAGE
#define SICSLOWPAN_CONF_MAXAGE                  4

#undef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG                    1

#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM                       8

#undef QUEUEBUF_CONF_REF_NUM
#define QUEUEBUF_CONF_REF_NUM                   0

#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_CONF_MAX_NEIGHBORS                    8

#undef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES                     8

#undef RPL_CONF_MAX_PARENTS_PER_DAG
#define RPL_CONF_MAX_PARENTS_PER_DAG            4

#undef RPL_CONF_MAX_INSTANCES
#define RPL_CONF_MAX_INSTANCES                  1

#undef RPL_CONF_MAX_DAG_PER_INSTANCE
#define RPL_CONF_MAX_DAG_PER_INSTANCE           1

#undef UIP_CONF_ND6_MAX_NEIGHBORS
#define UIP_CONF_ND6_MAX_NEIGHBORS              8

#undef UIP_CONF_MAX_CONNECTIONS
#define UIP_CONF_MAX_CONNECTIONS                8

#undef UIP_CONF_DS6_MADDR_NBU
#define UIP_CONF_DS6_MADDR_NBU 2

#define NETSTACK_RADIO_MAX_PAYLOAD_LEN 125

#include "mist-conf-const.h"

#undef MIST_CONF_NETSTACK
#define MIST_CONF_NETSTACK (MIST_CONF_NULLRDC | MIST_CONF_AES)

#include "mist-default-conf.h"

#if ((MIST_CONF_NETSTACK) & MIST_CONF_AES)
#ifndef NETSTACK_AES_KEY
#define NETSTACK_AES_KEY "thingsquare mist" /* 16 bytes */
#define NETSTACK_AES_KEY_DEFAULT 1
#endif /* NETSTACK_AES_KEY */
#endif /* CONTIKI_CONF_H */

#undef UIP_FALLBACK_INTERFACE
#define UIP_FALLBACK_INTERFACE ip64_uip_fallback_interface

#endif /* PLATFORM_CONF_H */
