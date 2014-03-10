#ifndef PLATFORM_CONF_H
#define PLATFORM_CONF_H

#undef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE 1300

#undef UIP_CONF_TCP_MSS
#define UIP_CONF_TCP_MSS     1000

#undef UIP_CONF_RECEIVE_WINDOW
#define UIP_CONF_RECEIVE_WINDOW 1000

#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_CONF_MAX_NEIGHBORS                 64

#undef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES                  64

/* Make HTTP/TCP connection way faster. */
#undef UIP_CONF_TCP_SPLIT
#define UIP_CONF_TCP_SPLIT 1
#undef UIP_SPLIT_CONF_SIZE
#define UIP_SPLIT_CONF_SIZE 8

#include "mist-conf-const.h"

#ifndef MIST_CONF_NETSTACK
//#define MIST_CONF_NETSTACK      MIST_CONF_NULLRDC
#define MIST_CONF_NETSTACK      MIST_CONF_DROWSIE
//#define MIST_CONF_NETSTACK      (MIST_CONF_AES | MIST_CONF_DROWSIE)

#endif /* MIST_CONF_NETSTACK */

#include "mist-default-conf.h"

#define NETSTACK_RADIO_MAX_PAYLOAD_LEN 125

#define NETSTACK_AES_KEY "thingsquare mist"

#undef UIP_FALLBACK_INTERFACE
#define UIP_FALLBACK_INTERFACE ip64_uip_fallback_interface

#define IP64_ADDRMAP_CONF_ENTRIES 64

/*#undef IEEE_ADDR_CONF_HARDCODED
#define IEEE_ADDR_CONF_HARDCODED             1

#undef IEEE_ADDR_CONF_IN_FLASH
#define IEEE_ADDR_CONF_IN_FLASH              0*/

#endif /* PLATFORM_CONF_H */
