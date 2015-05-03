/* 
 * File:   project_macros.h
 * Author: karrister
 *
 * Created on 3. toukokuuta 2015, 21:51
 */

#ifndef PROJECT_MACROS_H
#define	PROJECT_MACROS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "project_defs.h"

#define GET_SOCK_HW_ADDR_FROM_OFFSET(base_addr, offset)     (base_addr + offset)


/*Macros for turning LEDs on*/
#define LED1_ON() \
  do { \
    g_portc |= 0x01; \
    PORTC = g_portc;\
  } while (0)

#define LED2_ON() \
  do { \
    g_portc |= 0x02; \
    PORTC = g_portc;\
  } while (0)

#define LED3_ON() \
  do { \
    g_portc |= 0x04; \
    PORTC = g_portc;\
  } while (0)

#define LED4_ON() \
  do { \
    g_portc |= 0x08; \
    PORTC = g_portc;\
  } while (0)

/*Macros for turning LEDs off*/
#define LED1_OFF() \
  do { \
    g_portc &= 0xFE; \
    PORTC = g_portc;\
  } while (0)

#define LED2_OFF() \
  do { \
    g_portc &= 0xFD; \
    PORTC = g_portc;\
  } while (0)

#define LED3_OFF() \
  do { \
    g_portc &= 0xFB; \
    PORTC = g_portc;\
  } while (0)

#define LED4_OFF() \
  do { \
    g_portc &= 0xF7; \
    PORTC = g_portc;\
  } while (0)

#define LED1_TOGGLE() \
  do { \
    g_portc ^= 0x01; \
    PORTC = g_portc;\
  } while (0)

#define LED2_TOGGLE() \
  do { \
    g_portc ^= 0x02; \
    PORTC = g_portc;\
  } while (0)

#define LED3_TOGGLE() \
  do { \
    g_portc ^= 0x04; \
    PORTC = g_portc;\
  } while (0)

#define LED4_TOGGLE() \
  do { \
    g_portc ^= 0x08; \
    PORTC = g_portc;\
  } while (0)

/* Read the status of the LEDs */
#define LED1_STATUS() (PORTC & 0x01)
#define LED2_STATUS() (PORTC & 0x02)
#define LED3_STATUS() (PORTC & 0x04)
#define LED4_STATUS() (PORTC & 0x08)



#define RESET_LOW() \
  do { \
    PORTCbits.RC4 = 0; \
  } while (0)

#define RESET_HIGH() \
  do { \
    g_portc = PORTC;\
    g_portc |= BIT4;\
    PORTC = g_portc;\
  } while (0)


/*SPI macros*/
#define CS_ASSERT() \
  do { \
    PORTCbits.RC6 = 0; \
  } while (0)

#define CS_DEASSERT() \
  do { \
    PORTCbits.RC6 = 1; \
  } while (0)


#define DELAY_BETWEEN_COMMANDS() delay(5000)

#define SPI_SYNC_TX_BYTE(tx_byte) \
    do { \
        PIR1bits.SSPIF = 0; \
        SSPBUF = tx_byte; \
        while(!PIR1bits.SSPIF); \
        PIR1bits.SSPIF = 0; \
    } while(0)


#define WIZNET_WRITE(addr, byte) \
    do { \
        CS_ASSERT();\
        SPI_SYNC_TX_BYTE(0xF0);\
        SPI_SYNC_TX_BYTE((addr & 0xFF00) >> 8);\
        SPI_SYNC_TX_BYTE(addr & 0x00FF);\
        SPI_SYNC_TX_BYTE(byte);\
        CS_DEASSERT();\
    } while(0)



#ifdef	__cplusplus
}
#endif

#endif	/* PROJECT_MACROS_H */

