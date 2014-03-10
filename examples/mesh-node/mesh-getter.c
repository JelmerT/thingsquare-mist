#include "mist.h"
#include "http-client.h"

#define SEND_INTERVAL		(60 * CLOCK_SECOND)

static uip_ipaddr_t google_ipv4_dns_server = {
  .u8 = {
    /* Google's IPv4 DNS in mapped in an IPv6 address (::ffff:8.8.8.8) */
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xff, 0xff,
    0x08, 0x08, 0x08, 0x08,
  }
};


/*---------------------------------------------------------------------------*/
PROCESS(mesh_node_process, "Mesh node");
AUTOSTART_PROCESSES(&mesh_node_process);
/*---------------------------------------------------------------------------*/
static void
route_callback(int event, uip_ipaddr_t *route, uip_ipaddr_t *ipaddr,
               int numroutes)
{
  if(event == UIP_DS6_NOTIFICATION_DEFRT_ADD) {
    leds_off(LEDS_ALL);
    printf("Got a RPL route\n");
  }
}
/*---------------------------------------------------------------------------*/
static void
callback_http(const struct http_client_state *s,
	      http_client_result r)
{
  printf("HTTP call done, result %d\n", r);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(mesh_node_process, ev, data)
{
  static struct uip_ds6_notification n;
  static struct etimer periodic_timer;
  static struct http_client_state s;
  PROCESS_BEGIN();

  leds_on(LEDS_ALL);
  uip_ds6_notification_add(&n, route_callback);
  mdns_init();
  mdns_conf(&google_ipv4_dns_server);

  /* Turn off AES */
  netstack_aes_set_active(0);

  etimer_set(&periodic_timer, SEND_INTERVAL);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
  
  printf("Doing GET http://www.thingsquare.com/\n");
  http_client_get(&s, "http://www.thingsquare.com/", callback_http);

  while(1) {
    etimer_set(&periodic_timer, SEND_INTERVAL);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
  }


  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
