#undef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER framer_sniffer_802154

#undef NETSTACK_AES_KEY
#define NETSTACK_AES_KEY "thingsquare mist"

/* The RF channel must be between 11 and 26 */
#undef RF_CHANNEL
#define RF_CHANNEL                              26
