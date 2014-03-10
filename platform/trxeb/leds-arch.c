
#include "contiki-conf.h"
#include "dev/leds.h"

/* LED pins */
#define LEDS_CONF_RED    (1<<0)
#define LEDS_CONF_GREEN  (1<<2)
#define LEDS_CONF_YELLOW (1<<1)

/* LEDs mapping:
 * P4.0 LEDS_RED
 * P4.1 LEDS_YELLOW
 * P4.2 LEDS_GREEN
 * P4.3 not used
 */

/*---------------------------------------------------------------------------*/
void
leds_arch_init(void)
{
  P4DIR |= LEDS_CONF_RED;
  P4DIR |= LEDS_CONF_GREEN;
  P4DIR |= LEDS_CONF_YELLOW;
}
/*---------------------------------------------------------------------------*/
unsigned char
leds_arch_get(void)
{
  return
      (!(P4OUT & LEDS_CONF_RED) ? 0 : LEDS_RED)
    | (!(P4OUT & LEDS_CONF_GREEN) ? 0 : LEDS_GREEN)
    | (!(P4OUT & LEDS_CONF_YELLOW) ? 0 : LEDS_YELLOW);
}
/*---------------------------------------------------------------------------*/
void
leds_arch_set(unsigned char leds)
{
  P4OUT = (P4OUT & ~LEDS_CONF_RED) | ((leds & LEDS_RED) ? 0 : LEDS_CONF_RED);
  P4OUT = (P4OUT & ~LEDS_CONF_GREEN) | ((leds & LEDS_GREEN) ? 0 : LEDS_CONF_GREEN);
  P4OUT = (P4OUT & ~LEDS_CONF_YELLOW) | ((leds & LEDS_YELLOW) ? 0 : LEDS_CONF_YELLOW);
}
/*---------------------------------------------------------------------------*/
