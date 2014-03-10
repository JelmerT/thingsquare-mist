#ifndef PLATFORM_CONF_H
#define PLATFORM_CONF_H

#undef UIP_CONF_RECEIVE_WINDOW
#define UIP_CONF_RECEIVE_WINDOW  (UIP_CONF_BUFFER_SIZE - 60)

#undef UIP_CONF_TCP_MSS
#define UIP_CONF_TCP_MSS       576

/* Make HTTP/TCP connection way faster. */
#undef UIP_CONF_TCP_SPLIT
#define UIP_CONF_TCP_SPLIT 0
#undef UIP_SPLIT_CONF_SIZE
#define UIP_SPLIT_CONF_SIZE 8

#undef UIP_FALLBACK_INTERFACE
#define UIP_FALLBACK_INTERFACE ip64_uip_fallback_interface

#include "mist-conf-const.h"
#define MIST_CONF_NETSTACK (MIST_CONF_DROWSIE)

#include "mist-default-conf.h"

#define NETSTACK_RADIO_MAX_PAYLOAD_LEN 125


#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM 16

#endif /* PLATFORM_CONF_H */
