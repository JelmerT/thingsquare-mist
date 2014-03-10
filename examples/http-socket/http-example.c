#include "mist.h"

#include "http-socket.h"

static struct http_socket s;

/*---------------------------------------------------------------------------*/
PROCESS(http_example_process, "HTTP Example");
AUTOSTART_PROCESSES(&http_example_process);
/*---------------------------------------------------------------------------*/
static void
callback(struct http_socket *s, void *ptr,
         http_socket_event_t e,
         const uint8_t *data, uint16_t datalen)
{
  printf("callback e %d datalen %d data '%s'\n", e, datalen, data);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(http_example_process, ev, data)
{
  PROCESS_BEGIN();

  static struct etimer et;
  etimer_set(&et, CLOCK_SECOND);

  http_socket_get(&s, "http://172.16.0.1:8000/firmware",
                  callback, NULL);

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    etimer_reset(&et);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
