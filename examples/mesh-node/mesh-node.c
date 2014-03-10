#include "mist.h"

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
PROCESS_THREAD(mesh_node_process, ev, data)
{
  static struct uip_ds6_notification n;
  PROCESS_BEGIN();

  leds_on(LEDS_ALL);
  uip_ds6_notification_add(&n, route_callback);

  while(1) {
    PROCESS_WAIT_EVENT();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
