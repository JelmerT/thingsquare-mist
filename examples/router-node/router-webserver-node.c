#include "mist.h"
#include "ip64-webserver.h"

/*---------------------------------------------------------------------------*/
PROCESS(router_node_process, "Router node");
PROCESS(blinker_process, "Blinker process");
AUTOSTART_PROCESSES(&router_node_process,
                    &blinker_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(blinker_process, ev, data)
{
  static struct etimer et;
  static uint8_t red, green;
  PROCESS_BEGIN();

  etimer_set(&et, CLOCK_SECOND / 2);
  while(1) {
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    etimer_reset(&et);
    if(0) {
      leds_on(LEDS_RED);
      red = 1;
    } else {
      red = 0;
    }
  if(!ip64_hostaddr_is_configured()) {
      leds_on(LEDS_GREEN);
      green = 1;
    } else {
      green = 0;
    }
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    etimer_reset(&et);
    if(red) {
      leds_off(LEDS_RED);
    }
    if(green) {
      leds_off(LEDS_GREEN);
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(router_node_process, ev, data)
{
  PROCESS_BEGIN();

  /* Set us up as a RPL root node. */
  simple_rpl_init_dag();

  /* Initialize the IP64 module so we'll start translating packets */
  ip64_init();

  /* Initialize the IP64 webserver */
  ip64_webserver_init();

  /* ... and do nothing more. */
  while(1) {
    PROCESS_WAIT_EVENT();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
