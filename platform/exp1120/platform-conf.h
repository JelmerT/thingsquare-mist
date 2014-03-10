
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

/*
 * SPI bus configuration for the TMote Sky.
 */

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

/*
 * SPI bus - M25P80 external flash configuration.
 */
//#define FLASH_PWR     3       /* P4.3 Output */ ALWAYS POWERED ON Z1
#define FLASH_CS        4       /* P4.4 Output */
#define FLASH_HOLD      7       /* P5.7 Output */

/* Enable/disable flash access to the SPI bus (active low). */

#define SPI_FLASH_ENABLE()  ( P4OUT &= ~BV(FLASH_CS) )
#define SPI_FLASH_DISABLE() ( P4OUT |=  BV(FLASH_CS) )

#define SPI_FLASH_HOLD()                ( P5OUT &= ~BV(FLASH_HOLD) )
#define SPI_FLASH_UNHOLD()              ( P5OUT |=  BV(FLASH_HOLD) )

#define CC11xx_ARCH_SPI_ENABLE  cc1120_arch_spi_enable
#define CC11xx_ARCH_SPI_DISABLE cc1120_arch_spi_disable
#define CC11xx_ARCH_SPI_RW_BYTE cc1120_arch_spi_rw_byte
#define CC11xx_ARCH_SPI_RW      cc1120_arch_spi_rw

#define cc11xx_arch_spi_enable  cc1120_arch_spi_enable
#define cc11xx_arch_spi_disable cc1120_arch_spi_disable
#define cc11xx_arch_spi_rw_byte cc1120_arch_spi_rw_byte
#define cc11xx_arch_spi_rw      cc1120_arch_spi_rw
#define cc11xx_arch_interrupt_enable cc1120_arch_interrupt_enable

#define cc11xx_arch_init        cc1120_arch_init

#define NULLRDC_CONF_ACK_WAIT_TIME                RTIMER_SECOND / 500
#define NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME RTIMER_SECOND / 250

#define NETSTACK_CONF_RADIO   cc11xx_driver
#define NETSTACK_RADIO_MAX_PAYLOAD_LEN 125

signed char cc11xx_read_rssi(void);
void cc11xx_channel_set(uint8_t c);
#define MULTICHAN_CONF_SET_CHANNEL(x) cc11xx_channel_set(x)
#define MULTICHAN_CONF_READ_RSSI(x) cc11xx_read_rssi()

#endif /* __PLATFORM_CONF_H__ */
