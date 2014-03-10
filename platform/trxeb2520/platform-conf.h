
#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__

/* CPU target speed in Hz; works fine at 8, 16, 18 MHz but not higher. */
#define F_CPU 16000000uL

/* Our clock resolution, this is the same as Unix HZ. */
#define CLOCK_CONF_SECOND 128UL

#define BAUD2UBR(baud) ((F_CPU/baud))

#define CCIF
#define CLIF

#define HAVE_STDINT_H
#define MSP430_MEMCPY_WORKAROUND 1
#if defined(__MSP430__) && defined(__GNUC__) && MSP430_MEMCPY_WORKAROUND
#else /* __GNUC__ &&  __MSP430__ && MSP430_MEMCPY_WORKAROUND */
#define w_memcpy memcpy
#endif /* __GNUC__ &&  __MSP430__ && MSP430_MEMCPY_WORKAROUND */
#include "msp430def.h"

/* Types for clocks and uip_stats */
typedef unsigned short uip_stats_t;
typedef unsigned long clock_time_t;
#define CLOCK_LT(a,b)  ((signed long)((a)-(b)) < 0)
typedef unsigned long off_t;

/* DCO speed resynchronization for more robust UART, etc. */
/* Not needed from MSP430x5xx since it make use of the FLL */
#define DCOSYNCH_CONF_ENABLED 0
#define DCOSYNCH_CONF_PERIOD 30

#define ROM_ERASE_UNIT_SIZE  512
#define XMEM_ERASE_UNIT_SIZE (64*1024L)

#define CFS_CONF_OFFSET_TYPE    long

/* Use the first 64k of external flash for node configuration */
#define NODE_ID_XMEM_OFFSET     (0 * XMEM_ERASE_UNIT_SIZE)

/* Use the second 64k of external flash for codeprop. */
#define EEPROMFS_ADDR_CODEPROP  (1 * XMEM_ERASE_UNIT_SIZE)

#define CFS_XMEM_CONF_OFFSET    (2 * XMEM_ERASE_UNIT_SIZE)
#define CFS_XMEM_CONF_SIZE      (1 * XMEM_ERASE_UNIT_SIZE)

#define CFS_RAM_CONF_SIZE 4096

/* SPI input/output registers. */
#define SPI_TXBUF UCB0TXBUF
#define SPI_RXBUF UCB0RXBUF

/* USART0 Tx ready? */
#define SPI_WAITFOREOTx() while ((UCB0STAT & UCBUSY) != 0)
/* USART0 Rx ready? */
#define SPI_WAITFOREORx() while ((UCB0IFG & UCRXIFG) == 0)
/* USART0 Tx buffer ready? */
#define SPI_WAITFORTxREADY() while ((UCB0IFG & UCTXIFG) == 0)

#define MOSI           1  /* P3.1 - Output: SPI Master out - slave in (MOSI) */
#define MISO           2  /* P3.2 - Input:  SPI Master in - slave out (MISO) */
#define SCK            3  /* P3.3 - Output: SPI Serial Clock (SCLK) */

/* SPI CC2520 */
#define CC2520_CONF_SYMBOL_LOOP_COUNT 2604      /* 326us msp430X @ 16MHz */

/* See:
 * http://www.ti.com/lit/ds/swrs068/swrs068.pdf p 35
 * CC2520EM_2_1.pdf
 * http://www.ti.com/lit/ug/swru294a/swru294a.pdf p 24 */

/* Default GPIO <-> MSP mapping:
 *  FIFOP @ GPIO2: P3.5
 *  FIFO @ GPIO1: P3.4
 *  CCA @ GPIO3: P1.3
 *  SFD @ GPIO4: P1.2
 *  CSN 3.0
 *  VREG 1.7
 *  RESET 8.0
 *
 * With modified GPIO config:
 *  FIFOP @ GPIO0: P1.4 (INTERRUPT)
 *  FIFOP @ GPIO2: P3.5
 *  FIFO @ GPIO1: P3.4
 *  CCA @ GPIO3: P1.3
 *  SFD @ GPIO4: P1.2
 *  CSN 3.0
 *  VREG 1.7
 *  RESET 8.0
 * */

#define CC2520_CONF_GPIO0_AS_FIFOP 1
#define CONFIGURE_CC2520 1

#define CC2520_FIFOP_PORT(type)    P1##type
#define CC2520_FIFOP_PIN           4
#define CC2520_FIFO_PORT(type)     P3##type
#define CC2520_FIFO_PIN            4
#define CC2520_CCA_PORT(type)      P1##type
#define CC2520_CCA_PIN             3
#define CC2520_SFD_PORT(type)      P1##type
#define CC2520_SFD_PIN             2
#define CC2520_CSN_PORT(type)      P3##type
#define CC2520_CSN_PIN             0
#define CC2520_VREG_PORT(type)     P1##type
#define CC2520_VREG_PIN            7
#define CC2520_RESET_PORT(type)    P8##type
#define CC2520_RESET_PIN           0

#define CC2520_IRQ_VECTOR PORT1_VECTOR

/* Pin status. */
#define CC2520_FIFOP_IS_1 (!!(CC2520_FIFOP_PORT(IN) & BV(CC2520_FIFOP_PIN)))
#define CC2520_FIFO_IS_1  (!!(CC2520_FIFO_PORT(IN) & BV(CC2520_FIFO_PIN)))
#define CC2520_CCA_IS_1   (!!(CC2520_CCA_PORT(IN) & BV(CC2520_CCA_PIN)))
#define CC2520_SFD_IS_1   (!(CC2520_SFD_PORT(IN) & BV(CC2520_SFD_PIN)))

/* The CC2520 reset pin. */
#define SET_RESET_INACTIVE()   (CC2520_RESET_PORT(OUT) |=  BV(CC2520_RESET_PIN))
#define SET_RESET_ACTIVE()     (CC2520_RESET_PORT(OUT) &= ~BV(CC2520_RESET_PIN))

/* CC2520 voltage regulator enable pin. */
#define SET_VREG_ACTIVE()       (CC2520_VREG_PORT(OUT) |=  BV(CC2520_VREG_PIN))
#define SET_VREG_INACTIVE()     (CC2520_VREG_PORT(OUT) &= ~BV(CC2520_VREG_PIN))

/* CC2520 rising edge trigger for external interrupt 0 (FIFOP). */
#define CC2520_FIFOP_INT_INIT() do {                  \
    CC2520_FIFOP_PORT(IES) &= ~BV(CC2520_FIFOP_PIN);  \
    CC2520_CLEAR_FIFOP_INT();                         \
  } while(0)

/* FIFOP on external interrupt 0. */
#define CC2520_ENABLE_FIFOP_INT()  do {CC2520_FIFOP_PORT(IE) |= BV(CC2520_FIFOP_PIN);} while(0)
#define CC2520_DISABLE_FIFOP_INT() do {CC2520_FIFOP_PORT(IE) &= ~BV(CC2520_FIFOP_PIN);} while(0)
#define CC2520_CLEAR_FIFOP_INT()   do {CC2520_FIFOP_PORT(IFG) &= ~BV(CC2520_FIFOP_PIN);} while(0)

/*
 * Enables/disables CC2520 access to the SPI bus (not the bus).
 * (Chip Select)
 */
 /* ENABLE CSn (active low). */
#define CC2520_SPI_ENABLE()      CC2520_CSN_PORT(OUT) &= ~BV(CC2520_CSN_PIN)
#define CC2520_SPI_DISABLE()     CC2520_CSN_PORT(OUT) |= BV(CC2520_CSN_PIN)
#define CC2520_SPI_IS_ENABLED()  ((CC2520_CSN_PORT(OUT) & BV(CC2520_CSN_PIN)) != BV(CC2520_CSN_PIN))

#define NETSTACK_CONF_RADIO   cc2520_driver
#define NETSTACK_RADIO_MAX_PAYLOAD_LEN 125

#define NULLRDC_CONF_ACK_WAIT_TIME                RTIMER_SECOND / 500
#define NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME RTIMER_SECOND / 250

#define CONTIKIMAC_CONF_INTER_PACKET_INTERVAL     RTIMER_SECOND / 500
#define CONTIKIMAC_CONF_AFTER_ACK_DETECTECT_WAIT_TIME RTIMER_SECOND / 250

#define DROWSIE_CONF_ON_TIME                   (CLOCK_SECOND / 128)
#define DROWSIE_CONF_INTER_PACKET_INTERVAL     RTIMER_SECOND / 500
#define DROWSIE_CONF_AFTER_ACK_DETECTECT_WAIT_TIME RTIMER_SECOND / 200
#define DROWSIE_CONF_LISTEN_TIME_AFTER_PACKET_DETECTED RTIMER_SECOND / 40
#define DROWSIE_CONF_CCA_COUNT_MAX_TX          18


#include "mist-conf-const.h"
#ifndef MIST_CONF_NETSTACK
#define MIST_CONF_NETSTACK (MIST_CONF_AES | MIST_CONF_DROWSIE)
#endif /* MIST_CONF_NETSTACK */

#define MULTICHAN_CONF_SET_CHANNEL(x) cc2520_set_channel(x)
#define MULTICHAN_CONF_READ_RSSI(x)  cc2520_rssi()

#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM        4
#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_CONF_MAX_NEIGHBORS     4
#undef UIP_CONF_DS6_ROUTE_NBU
#define UIP_CONF_DS6_ROUTE_NBU   4

#include "ip64-conf.h"

#endif /* __PLATFORM_CONF_H__ */
