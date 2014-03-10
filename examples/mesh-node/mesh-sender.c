#include "mist.h"

#define PORT 12345

#define SEND_INTERVAL		(30 * CLOCK_SECOND)


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
receiver(struct udp_socket *c,
         void *ptr,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
  printf("Data received on port %d from port %d with length %d, '%s'\n",
         receiver_port, sender_port, datalen, data);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(mesh_node_process, ev, data)
{
  static struct uip_ds6_notification n;
  static struct udp_socket s;
  static uip_ip4addr_t v4addr;
  static uip_ip6addr_t v6addr;
  static struct etimer periodic_timer;

  PROCESS_BEGIN();

  leds_on(LEDS_ALL);
  uip_ds6_notification_add(&n, route_callback);

  /* Register UDP socket callback */
  udp_socket_register(&s, NULL, receiver);

  /* Bind UDP socket to local port */
  udp_socket_bind(&s, PORT);

  /* Connect UDP socket to remote port */
  udp_socket_connect(&s, NULL, PORT);

  while(1) {
    etimer_set(&periodic_timer, SEND_INTERVAL);

    PROCESS_WAIT_UNTIL(etimer_expired(&periodic_timer));


    uip_ipaddr(&v4addr, 172,16,0,1);
    ip64_addr_4to6(&v4addr, &v6addr);

    printf("Sending to 172.16.0.1\n");
    udp_socket_sendto(&s,
                      "hello", 5,
                      &v6addr, PORT);

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
