/* 
 * File:   project_defs.h
 * Author: karrister
 *
 * Created on 3. toukokuuta 2015, 21:49
 */

#ifndef PROJECT_DEFS_H
#define	PROJECT_DEFS_H

#ifdef	__cplusplus
extern "C" {
#endif


#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

//Typedefs
typedef unsigned char DEBUG_MSG;

/*Debug related defines*/
//#define KARRI_DEBUG //basic debug
//#define UART_DEBUG //send debug to serial
//#define SPI_DEBUG //for debugging SPI, connection test etc
//#define KARRI_DEBUG_SM //debugging the main state machine
//#define KARRI_DEBUG_RX_PATH
/**/


#define KARRI_OK                    1
#define KARRI_NOK                   0

#define LED_STATE_ON                1
#define LED_STATE_OFF               0

#define LED_NUMBER_1                1
#define LED_NUMBER_2                2
#define LED_NUMBER_3                3
#define LED_NUMBER_4                4

#define READ_OPCODE                 0x0F
#define WRITE_OPCODE                0xF0

#define TEST_BYTE                   0xCA

#define MAX_UART_PRINT_LENGTH       64

#define TX_GETSIZE_TIMEOUT_THRESHOLD    0xFF //max for ushort

#define SOCK0                       0
#define SOCK1                       1
#define SOCK2                       2
#define SOCK3                       3

#define BIT0                        0x01
#define BIT1                        0x02
#define BIT2                        0x04
#define BIT3                        0x08
#define BIT4                        0x10
#define BIT5                        0x20
#define BIT6                        0x40
#define BIT7                        0x80

//Mode register values
#define SOCK_MODE_ENABLE_MCAST      (1 << 7)
#define SOCK_MODE_ENABLE_NO_ACK     (1 << 5)
#define SOCK_MODE_TCP               (0x01)
#define SOCK_MODE_UDP               (0x02)
#define SOCK_MODE_IPRAW             (0x03)
#define SOCK_MODE_MACRAW            (0x04)
#define SOCK_MODE_PPPOE             (0x05)
#define SOCK_MODE_CLOSED            (0x00)

//socket states
#define OPEN                        0x01
#define LISTEN                      0x02
#define CONNECT                     0x04
#define DISCON                      0x08
#define CLOSE                       0x10
#define SEND                        0x20
#define SEND_MAC                    0x21
#define SEND_KEEP                   0x22
#define RECV                        0x40

//socket statuses
#define SOCK_CLOSED                 0x00
#define SOCK_INIT                   0x13
#define SOCK_LISTEN                 0x14
#define SOCK_ESTABLISHED            0x17
#define SOCK_CLOSE_WAIT             0x1C
#define SOCK_UDP                    0x22
#define SOCK_IPRAW                  0x32
#define SOCK_MACRAW                 0x42
#define SOCK_PPPOE                  0x5F
#define SOCK_SYNSENT                0x15
#define SOCK_SYNRECV                0x16
#define SOCK_FIN_WAIT               0x18
#define SOCK_CLOSING                0x1A
#define SOCK_TIME_WAIT              0x1B
#define SOCK_LAST_ACK               0x1D
#define SOCK_ARP1                   0x11
#define SOCK_ARP2                   0x21
#define SOCK_ARP3                   0x31

#define CHIP_BASE_ADDRESS           0x0000
#define RX_MEMORY_BASE_ADDRESS      0x6000
#define TX_MEMORY_BASE_ADDRESS      0x4000

#define ETHERNET_BUFF_SIZE          64
#define MAX_RX_SIZE                 ETHERNET_BUFF_SIZE

#define MRREG_SOFT_RESET            0x80


/*The register addresses*/
//Common registers
#define REG_MODE                    0x0000
#define REG_GATEWAY_IP0             0x0001
#define REG_GATEWAY_IP1             0x0002
#define REG_GATEWAY_IP2             0x0003
#define REG_GATEWAY_IP3             0x0004
#define REG_SUBNET_MASK_IP0         0x0005
#define REG_SUBNET_MASK_IP1         0x0006
#define REG_SUBNET_MASK_IP2         0x0007
#define REG_SUBNET_MASK_IP3         0x0008
#define REG_SOURCE_ADDR_MAC0        0x0009
#define REG_SOURCE_ADDR_MAC1        0x000A
#define REG_SOURCE_ADDR_MAC2        0x000B
#define REG_SOURCE_ADDR_MAC3        0x000C
#define REG_SOURCE_ADDR_MAC4        0x000D
#define REG_SOURCE_ADDR_MAC5        0x000E
#define REG_SOURCE_ADDR_IP0         0x000F
#define REG_SOURCE_ADDR_IP1         0x0010
#define REG_SOURCE_ADDR_IP2         0x0011
#define REG_SOURCE_ADDR_IP3         0x0012
#define REG_INTERRUPT_SOURCE        0x0015
#define REG_INTERRUPT_MASK          0x0016
#define REG_RETRY_TIME_VALUE0       0x0017
#define REG_RETRY_TIME_VALUE1       0x0018
#define REG_RETRY_COUNT             0x0019
#define REG_RX_MEM_SIZE             0x001A
#define REG_TX_MEM_SIZE             0x001B
#define REG_PPPOE_AUTH_TYPE0        0x001C
#define REG_PPPOE_AUTH_TYPE1        0x001D
#define REG_PPP_TIMER               0x0028
#define REG_PPP_MAGIC               0x0029
#define REG_DEST_UNREACHABLE_IP0    0x002A
#define REG_DEST_UNREACHABLE_IP1    0x002B
#define REG_DEST_UNREACHABLE_IP2    0x002C
#define REG_DEST_UNREACHABLE_IP3    0x002D
#define REG_UNREACHABLE_PORT0       0x002E
#define REG_UNREACHABLE_PORT1       0x002F

//Base register addresses for sockets
#define SOCK0_REGS_BASE_ADDR        0x0400
#define SOCK1_REGS_BASE_ADDR        0x0500
#define SOCK2_REGS_BASE_ADDR        0x0600
#define SOCK3_REGS_BASE_ADDR        0x0700

//Register offsets for socket registers
#define SOCK_REG_MODE                0x0000
#define SOCK_REG_COMMAND             0x0001
#define SOCK_REG_INTERRUPT           0x0002
#define SOCK_REG_STATUS              0x0003
#define SOCK_REG_SOURCE_PORT0        0x0004
#define SOCK_REG_SOURCE_PORT1        0x0005
#define SOCK_REG_DEST_ADDR_MAC0      0x0006
#define SOCK_REG_DEST_ADDR_MAC1      0x0007
#define SOCK_REG_DEST_ADDR_MAC2      0x0008
#define SOCK_REG_DEST_ADDR_MAC3      0x0009
#define SOCK_REG_DEST_ADDR_MAC4      0x000A
#define SOCK_REG_DEST_ADDR_MAC5      0x000B
#define SOCK_REG_DEST_ADDR_IP0       0x000C
#define SOCK_REG_DEST_ADDR_IP1       0x000D
#define SOCK_REG_DEST_ADDR_IP2       0x000E
#define SOCK_REG_DEST_ADDR_IP3       0x000F
#define SOCK_REG_DEST_PORT0          0x0010
#define SOCK_REG_DEST_PORT1          0x0011
#define SOCK_REG_MAX_SEG_SIZE0       0x0012
#define SOCK_REG_MAX_SEG_SIZE1       0x0013
#define SOCK_REG_IP_PROTOCOL         0x0014
#define SOCK_REG_IP_TOS              0x0015
#define SOCK_REG_IP_TTL              0x0016
#define SOCK_REG_TX_FREE_SIZE0       0x0020
#define SOCK_REG_TX_FREE_SIZE1       0x0021
#define SOCK_REG_TX_READ_POINTER0    0x0022
#define SOCK_REG_TX_READ_POINTER1    0x0023
#define SOCK_REG_TX_WRITE_POINTER0   0x0024
#define SOCK_REG_TX_WRITE_POINTER1   0x0025
#define SOCK_REG_RX_RECEIVED_SIZE0   0x0026
#define SOCK_REG_RX_RECEIVED_SIZE1   0x0027
#define SOCK_REG_RX_READ_POINTER0    0x0028
#define SOCK_REG_RX_READ_POINTER1    0x0029

//Per-socket addresses
#define SOCKET1_REG_COMMAND          0x0401
#define SOCKET1_REG_STATUS           0x0403
#define SOCKET1_REG_TX_FREE_SIZE_MSB 0x0420
#define SOCKET1_REG_TX_FREE_SIZE_LSB 0x0421
#define SOCKET1_REG_TX_WRITE_PTR_MSB 0x0424
#define SOCKET1_REG_TX_WRITE_PTR_LSB 0x0425
#define SOCKET1_REG_RX_SIZE_MSB      0x0426
#define SOCKET1_REG_RX_SIZE_LSB      0x0427
#define SOCKET1_REG_RX_READ_PTR_MSB  0x0428
#define SOCKET1_REG_RX_READ_PTR_LSB  0x0428


/*reg addr end*/

#define L4_PORT_NO_MSB              0x00
#define L4_PORT_80_LSB              0x50



typedef unsigned char BYTE;
typedef BYTE KARRI_BOOL;

typedef enum {
            IDLE,
            LISTENING,
            PEER_CONNECTED,
            REQUEST_RECEIVED,
            PARSE_REQUEST,
            SEND_RESPONSE,
            WAIT_BEFORE_IDLE,
            CLOSE_SOCKET,
            DISCONNECT_SOCKET,
            RESET_MCU,
} mainserverSM;

typedef enum {
            PARSE_NO_MATCH,
            PARSE_MATCH,
            TOGGLE_LED1,
            TOGGLE_LED2,
            TOGGLE_LED3,
            TOGGLE_LED4,
} PARSE_STATUS;

/*Static */

/*static const saves it in the program memory, not RAM*/

static const BYTE get_cmd[] = "GET /";

/* This is the template for the website published on our PIC-server */
static const BYTE website_body[] =
" \
<HEAD> \
<TITLE>PIC16-webserver</TITLE> \
</HEAD> \
<BODY> \
<H2>Welcome to pic16server. Toggle LEDs: \
<H1> \
<a href=\"toggle_led1\">LED1</a> \
<a href=\"toggle_led2\">LED2</a> \
<a href=\"toggle_led3\">LED3</a> \
<a href=\"toggle_led4\">LED4</a> \
<br><br> \
<H2>Status:<P> \
<H1> \
";

/*
<HEAD>
<TITLE>PIC18-webserver</TITLE>
</HEAD>
<BODY>
<H2>Welcome to picserver. Toggle LEDs:
<H1>
<a href="toggle_led1">LED1</a>
<a href="toggle_led2">LED2</a>
<a href="toggle_led3">LED3</a>
<a href="toggle_led4">LED4</a>
<br><br>
<H2>Status:<P>
<H1>
LED1 <font color='green'>ON</font>
LED2 <font color='green'>ON</font>
LED3 <font color='red'>OFF</font>
LED4 <font color='green'>ON</font>
</body>
 */
#define DYNAMIC_HTML_LINE_MAX_SIZE 35 //length of the text: LED1 <font color='green'>ON</font>

static const BYTE website_body_end[] = "</BODY>";


/*Globals*/
short g_portc = 0;


int g_rx_sock0_mask;
int g_rx_sock0_base;

int g_tx_sock0_mask;
int g_tx_sock0_base;

int g_mainSM_state = IDLE;




/*Configuration bits*/
#pragma config CPD      = OFF
#pragma config BOREN    = ON
#pragma config IESO     = OFF
#pragma config FOSC     = HS //No need to consider power consumption
#pragma config FCMEN    = ON
#pragma config MCLRE    = ON //need to check
#pragma config WDTE     = OFF
#pragma config CP       = OFF
#pragma config PWRTE    = ON
/**/


#ifdef	__cplusplus
}
#endif

#endif	/* PROJECT_DEFS_H */

