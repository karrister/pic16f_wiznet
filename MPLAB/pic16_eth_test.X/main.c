/*********************************************
 *  WIZNET W5100 PROJECT
 *********************************************
 * AUTHOR: Karri Kivelä
 *
 * CREATION: Spring 2014
 *
 * BRIEF: Testing the capabilities of the
 * W5100 chip, using an Arduino(!!) ethernet
 * shield with PIC16F690. Intention to make
 * a web server that controls the LEDs on
 * the PICkit2 dev kit board (the CPU)
 */

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>


//Typedefs
typedef unsigned char DEBUG_MSG;


#define EXTENDED_FUNCTIONALITY

/*Debug related defines*/
//#define KARRI_DEBUG //LED debug
#define UART_DEBUG //send debug to serial
#define SPI_DEBUG //for debugging SPI, connection test etc

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


#define BIT0                        0x01
#define BIT1                        0x02
#define BIT2                        0x04
#define BIT3                        0x08
#define BIT4                        0x10
#define BIT5                        0x20
#define BIT6                        0x40
#define BIT7                        0x80

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

#define MAX_RX_SIZE                 128
#define ETHERNET_BUFF_SIZE          64

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

#define SOCK0                       0
#define SOCK1                       1
#define SOCK2                       2
#define SOCK3                       3


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

//Macros for getting the end address by socket #
inline unsigned int
get_sock_base_addr_by_sock_num(unsigned int sock)
{
    switch(sock)
    {
        case SOCK0:
            return SOCK0_REGS_BASE_ADDR;
        case SOCK1:
            return SOCK1_REGS_BASE_ADDR;
        case SOCK2:
            return SOCK2_REGS_BASE_ADDR;
        case SOCK3:
            return SOCK3_REGS_BASE_ADDR;
        default:
            return 0xFFFF;
    }
}

#define GET_SOCK_HW_ADDR_FROM_OFFSET(base_addr, offset)     (base_addr + offset)

/*reg addr end*/

typedef unsigned char BYTE;
typedef BYTE KARRI_BOOL;

typedef enum {
            IDLE,
            LISTENING,
            PEER_CONNECTED,
            REQUEST_RECEIVED,
            PARSE_REQUEST,
            SEND_RESPONSE
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

int g_rx_sock1_mask;
int g_rx_sock1_base;

int g_tx_sock0_mask;
int g_tx_sock0_base;

int g_tx_sock1_mask;
int g_tx_sock1_base;

int g_mainSM_state = IDLE;

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
#define LED1_STATUS() (PORTC && 0x01)
#define LED2_STATUS() (PORTC && 0x02)
#define LED3_STATUS() (PORTC && 0x04)
#define LED4_STATUS() (PORTC && 0x08)



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


#ifdef UART_DEBUG
/**
* This inline function outputs debug messages out of the chip UART bus.
* @param msg The debug message.
* @return void
*/
inline void debug_print(DEBUG_MSG *msg)
{
    int x;
    for(x=0;x<MAX_UART_PRINT_LENGTH;x++)
    {
        if(msg[x] == '\n') //break character
        {
            break;
        }

        while(!TXSTAbits.TRMT);

        TXREG = msg[x];
    }

    while(!TXSTAbits.TRMT);
    TXREG = 0x0A;
    while(!TXSTAbits.TRMT);
    TXREG = 0x0D;

}
#else
inline void debug_print(DEBUG_MSG *msg)
{
    _nop();
}
#endif //UART_DEBUG

/**
* This inline function reads a single byte from the chip. This function manages
* the data transfer over the SPI bus from the chip towards the master.
* @param addr The address inside the chip to read from.
* @return The byte read from the chip.
*/
inline unsigned char
wiznet_read(unsigned int addr)
{
    unsigned char rxByte = 0;
    CS_ASSERT();
    SPI_SYNC_TX_BYTE(READ_OPCODE);
    SPI_SYNC_TX_BYTE((addr & 0xFF00) >> 8);
    SPI_SYNC_TX_BYTE(addr & 0x00FF);
    SPI_SYNC_TX_BYTE(0x00); //dummy data, just for clock output
    rxByte = SSPBUF;
    CS_DEASSERT();
    return rxByte;
}


/**
* This function compares the input buffer to a command buffer and tries to find
* the specified command inside the input buffer (the offset doesn't matter).
* @param eth_buff pointer to the input buffer.
* @param cmd pointer to the command buffer.
* @param input_size the size of the data in the eth_buff input buffer
* @param cmd_size the size of the command we are looking for
* @param buffer_offset pointer to uint where the function will save the offset in the input buffer where the match was found
* @see main()
* @return The boolean result if the specified command was found inside the input buffer.
*/
inline KARRI_BOOL compare_buffers(BYTE *eth_buff, BYTE *cmd, unsigned int input_size, unsigned int cmd_size, unsigned int *buffer_offset)
{
    int get_cmd_lookup_loop, get_cmd_matching_loop;
    for(get_cmd_lookup_loop = 0; get_cmd_lookup_loop < input_size; get_cmd_lookup_loop++)
    {
        *buffer_offset = get_cmd_lookup_loop;
        if(eth_buff[get_cmd_lookup_loop] == cmd[0])
        {
            for(get_cmd_matching_loop = 0; get_cmd_matching_loop < cmd_size; get_cmd_matching_loop++)
            {
                if(eth_buff[get_cmd_lookup_loop + get_cmd_matching_loop] != cmd[get_cmd_matching_loop])
                {
                    /* No CMD found afterall */
                    return KARRI_NOK;
                }
            }
            /* If we get here, it means we got successfully out of the for-loop and found our CMD */
            return KARRI_OK;
        }
    }

    /* If we get here, it means we couldn't find our CMD */
    return KARRI_NOK;
}

inline void zeromem(BYTE *memory, unsigned int size)
{ /* clear out the buffer */
    unsigned int x;
    for(x = size; x > 0; x--)
    {
        memory[x] = 0;
    }
}


/**
* This function compares the input buffer to a command buffer and tries to find
* the specified command inside the input buffer (the offset doesn't matter).
* If there was a match, next task is to find out, starting from the offset of
* after the basic command, looking through the incoming ethernet buffer and trying
* to find some specific commands, and returning the result as a PARSE_STATUS enum.
* @param eth_buff pointer to the input buffer.
* @param cmd pointer to the command buffer.
* @param input_size the size of the data in the eth_buff input buffer
* @param cmd_size the size of the command we are looking for
* @see compare_buffers()
* @see main()
* @return The result of the parsing.
*/
inline PARSE_STATUS parse_command(BYTE *eth_buff, BYTE *cmd, unsigned int input_size, unsigned int cmd_size)
{
    PARSE_STATUS ret = PARSE_NO_MATCH;
    BYTE        *command_buffer;
    unsigned int buffer_offset;
    if(!compare_buffers(eth_buff, cmd, input_size, cmd_size, &buffer_offset))
    {
        /* No match was found, return immediately */
        return PARSE_NO_MATCH;
    }

    /* Let's make the buffer to point to local pointer from the wanted offset, after the initial command */
    command_buffer = eth_buff + buffer_offset + cmd_size;

    /* Get ready... Here is the huge parser! */
    if(command_buffer[0] == 't' &&
       command_buffer[1] == 'o' &&
       command_buffer[2] == 'g' &&
       command_buffer[3] == 'g' &&
       command_buffer[4] == 'l' &&
       command_buffer[5] == 'e' &&
       command_buffer[6] == '_' &&
       command_buffer[7] == 'l' &&
       command_buffer[8] == 'e' &&
       command_buffer[9] == 'd')
    {
        switch(command_buffer[10])
        {
            case '1':
                ret = TOGGLE_LED1;
                break;
            case '2':
                ret = TOGGLE_LED2;
                break;
            case '3':
                ret = TOGGLE_LED3;
                break;
            case '4':
                ret = TOGGLE_LED4;
                break;
            default:
                ret = PARSE_NO_MATCH;
                break;
        }
    }
    
    return ret;
}


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

/********************************************/
/*Implementations*/


void delay(unsigned int count)
{
    unsigned int x;
    for(x=0;x<count;x++)
    {
        _nop();
    }
}

void wiznet_read_bytes(unsigned int chip_addr, BYTE *rx_buff, unsigned int len)
{
    int x;
    for(x=0;x<len;x++)
    {
        rx_buff[x] = wiznet_read(chip_addr);
        chip_addr++;
    }
}


void wiznet_write_bytes(unsigned int chip_addr, BYTE *tx_buff, unsigned int len)
{
    int x;
    for(x=0;x<len;x++)
    {
        WIZNET_WRITE(chip_addr, tx_buff[x]);
        chip_addr++;
    }
}


// SDI - RB4
// SDO - RC7
// SCL - RB6
// SS  - RC6
/*Write & read test byte*/
short test_wiznet_connection(void)
{
    short conn_status = KARRI_NOK;
    unsigned char rxByte = 0;


    //We use here register REG_GATEWAY_IP3 for testing write & read of the CHIP
    WIZNET_WRITE(0x0004, TEST_BYTE);

    rxByte = wiznet_read(0x0004);

    WIZNET_WRITE(0x0004, 0x00);

    if(rxByte == TEST_BYTE)
    {
        conn_status = KARRI_OK;
    }
    else
    {
        conn_status = KARRI_NOK;
    }

    return conn_status;
}


void init_wiznet(void)
{
    /**GENERAL NETWORK STACK CONFIGURATION**/
    //Set IMR -  all interrupts as masked 
    WIZNET_WRITE(REG_INTERRUPT_MASK,0x00);
    DELAY_BETWEEN_COMMANDS();

    //RTR default (200ms) no need to change currently
    //RCR default (8) no need to change currently

    /**IP LEVEL CONFIGURATION**/
    //No space/time for DHCP server code, setting options currently as fixed!
    //GWR register - gateway IP address 10.0.0.138
    WIZNET_WRITE(REG_GATEWAY_IP0,0x0A); //10
    DELAY_BETWEEN_COMMANDS();
    WIZNET_WRITE(REG_GATEWAY_IP1,0x00); //.0
    DELAY_BETWEEN_COMMANDS();
    WIZNET_WRITE(REG_GATEWAY_IP2,0x00); //.0
    DELAY_BETWEEN_COMMANDS();
    WIZNET_WRITE(REG_GATEWAY_IP3,0x8A); //.138
    DELAY_BETWEEN_COMMANDS();

    //SUBR register - subnet mask 255.255.255.0
    WIZNET_WRITE(REG_SUBNET_MASK_IP0,0xFF); //255
    DELAY_BETWEEN_COMMANDS();
    WIZNET_WRITE(REG_SUBNET_MASK_IP1,0xFF); //.255
    DELAY_BETWEEN_COMMANDS();
    WIZNET_WRITE(REG_SUBNET_MASK_IP2,0xFF); //.255
    DELAY_BETWEEN_COMMANDS();
    WIZNET_WRITE(REG_SUBNET_MASK_IP3,0x00); //.0
    DELAY_BETWEEN_COMMANDS();
    //SAHR register - MAC address CA:CA:CA:FE:BA:BE
    WIZNET_WRITE(REG_SOURCE_ADDR_MAC0,0xCA); //CA
    DELAY_BETWEEN_COMMANDS();
    WIZNET_WRITE(REG_SOURCE_ADDR_MAC1,0xCA); //CA
    DELAY_BETWEEN_COMMANDS();
    WIZNET_WRITE(REG_SOURCE_ADDR_MAC2,0xCA); //CA
    DELAY_BETWEEN_COMMANDS();
    WIZNET_WRITE(REG_SOURCE_ADDR_MAC3,0xFE); //FE
    DELAY_BETWEEN_COMMANDS();
    WIZNET_WRITE(REG_SOURCE_ADDR_MAC4,0xBA); //BA
    DELAY_BETWEEN_COMMANDS();
    WIZNET_WRITE(REG_SOURCE_ADDR_MAC5,0xBE); //BE
    DELAY_BETWEEN_COMMANDS();
    //SIPR register - device IP address 10.0.0.77
    WIZNET_WRITE(REG_SOURCE_ADDR_IP0,0x0A); //10
    DELAY_BETWEEN_COMMANDS();
    WIZNET_WRITE(REG_SOURCE_ADDR_IP1,0x00); //.0
    DELAY_BETWEEN_COMMANDS();
    WIZNET_WRITE(REG_SOURCE_ADDR_IP2,0x00); //.0
    DELAY_BETWEEN_COMMANDS();
    WIZNET_WRITE(REG_SOURCE_ADDR_IP3,0x4D); //.77
    DELAY_BETWEEN_COMMANDS();

    /**SOCKET BUFFER CONFIGURATION**/
    //Setting up the 8K memory for 2 sockets, 4k per socket. Only using 1 socket but why not set up two if by some chance we'd need a second one :)
    //RMSR register - setting 4K to sockets 0 & 1
    WIZNET_WRITE(REG_RX_MEM_SIZE,0x0A);
    DELAY_BETWEEN_COMMANDS();
    //TMSR register - setting 4K to sockets 0 & 1
    WIZNET_WRITE(REG_TX_MEM_SIZE,0x0A);
    DELAY_BETWEEN_COMMANDS();

    g_rx_sock0_base = CHIP_BASE_ADDRESS + RX_MEMORY_BASE_ADDRESS;
    g_rx_sock0_mask = 0x0FFF;

    g_rx_sock1_base = g_rx_sock0_base + (g_rx_sock0_mask + 1);
    g_rx_sock1_mask = 0x0FFF;

    g_tx_sock0_base = CHIP_BASE_ADDRESS + TX_MEMORY_BASE_ADDRESS;
    g_tx_sock0_mask = 0x0FFF;

    g_tx_sock1_base = g_tx_sock0_base + (g_tx_sock0_mask + 1);
    g_tx_sock1_mask = 0x0FFF;

    
}


void dev_init(void)
{
    /*Clock*/
    OSCCON = 0x08; //clock from fOSC

    //Use all those MOTHER FUCKERS as DIGITAL-FUCKING-INPUT!!!
    ANSEL = 0;
    ANSELH = 0;

    /*LEDS I/O*/
    //Set RC0 (LED1) to output
    TRISCbits.TRISC0 = 0;
    //Set RC1 (LED2) to output
    TRISCbits.TRISC1 = 0;
    //Set RC2 (LED3) to output
    TRISCbits.TRISC2 = 0;
    //Set RC3 (LED4) to output
    TRISCbits.TRISC3 = 0;
    //All output to zero
    PORTC = 0;

#ifdef KARRI_DEBUG
    TRISCbits.TRISC4 = 0; //debug pin output
#endif
    
    /*SPI init*/

    //Set RC6 (SS) to output
    TRISCbits.TRISC6 = 0;

    //Set RC7 (SDO) to output
    TRISCbits.TRISC7 = 0;
    //Set RB6 (SCL) to output
    TRISBbits.TRISB6 = 0;
    //Set RB4 (SDI) to input
    TRISBbits.TRISB4 = 1;

    //Disable the pull up resistors for all the PORTB pins 4-7
    WPUB = 0;
    //Disable the itnerrupt on change for all the PORTB pins 4-7
    IOCB = 0;
    
    //SPI idle low (CKP = 0), Data transmitted on rising edge (CKE = 1)
    SSPSTAT = /*0xC0;*/ 0x40;
    SSPCON = 0x21; //SCL fOSC/16 == 250kHz

#ifdef UART_DEBUG
    /*Debug UART init*/
    TXSTA = 0x24; //enable TX, high baud bit
    RCSTA = 0x80; //enable serial
    BAUDCTL = 0; //8-bit baud rate generator

    SPBRG = 0x19; //9600baud
#endif

    /*Wiznet CHIP HW reset*/
    RESET_LOW();
    RESET_HIGH();

    /*This is for some reason important!*/
    CS_ASSERT();
    CS_DEASSERT();
}


int main(void)
{
    short spi_status = KARRI_NOK;
    dev_init();
    //Sleep after init (needed?)
    delay(60000);
    delay(60000);
    delay(60000);
    delay(60000);
    delay(60000);


#ifdef KARRI_DEBUG
    //Indicate init finished, firmware started
    LED1_ON();
    debug_print("Init finished, LED1 on, FW started!\n");

#endif //KARRI_DEBUG

#ifdef SPI_DEBUG //When suspect problems with SPI bus
    //debug_print("Testing connection to CHIP!\n");

    delay(60000);
    delay(60000);
    delay(60000);
    delay(60000);
    delay(60000);

    spi_status = test_wiznet_connection();
#ifndef KARRI_DEBUG
    if(spi_status != KARRI_OK)
    {
        //Indicate connection to wiznet was NOK
        //debug_print("Connection to CHIP failed!\n");
        //LED3_ON();
        while(1);
    }
#endif //KARRI_DEBUG
    //Indicate connection to wiznet was OK
#ifdef KARRI_DEBUG
    debug_print("Connection to CHIP OK\n");
    LED1_ON();
#endif

#else //SPI_DEBUG
    _nop();
#endif //SPI_DEBUG
    //Init ETH
    init_wiznet();

#ifdef KARRI_DEBUG
    LED1_OFF();
    debug_print("Went through CHIP init, entering main state machine!\n");
#endif

    BYTE spi_rx_byte                          = 0;

    unsigned int rx_data_size                 = 0;

    unsigned int rx_data_offset               = 0;
    unsigned int rx_data_start_addr           = 0;
    unsigned int rx_sock1_read_pointer        = 0;
    unsigned int rx_upper_size                = 0;

    unsigned int tx_data_size                 = 0;
    unsigned int tx_data_offset               = 0;
    unsigned int tx_data_start_addr           = 0;
    unsigned int tx_sock1_write_pointer       = 0;
    unsigned int tx_upper_size                = 0;

    unsigned short tx_getsize_timeout_counter = 0;

    PARSE_STATUS parse_status = PARSE_NO_MATCH;

    unsigned char print[] = "rx size: xxxx\n";

    BYTE int_string[] = "0123";

    BYTE eth_buff[ETHERNET_BUFF_SIZE];

    unsigned int sock0_base_addr = 0;

    //Get sock0 base address for all the subsequent calls
    sock0_base_addr = get_sock_base_addr_by_sock_num(SOCK0);

    while(1)
    {
        switch(g_mainSM_state)
        {
            case IDLE:
#ifdef KARRI_DEBUG
                debug_print("mainSM: case IDLE\n");
#endif
                /**SET SERVER SOCKET**/
                //Set TCP mode
                WIZNET_WRITE(GET_SOCK_HW_ADDR_FROM_OFFSET(sock0_base_addr, SOCK_REG_MODE),0x01);
                //Set port to 80 (www);
                WIZNET_WRITE(GET_SOCK_HW_ADDR_FROM_OFFSET(sock0_base_addr, SOCK_REG_SOURCE_PORT0),0x00);
                WIZNET_WRITE(GET_SOCK_HW_ADDR_FROM_OFFSET(sock0_base_addr, SOCK_REG_SOURCE_PORT1),0x50);
                
                //Set socket to open
                WIZNET_WRITE(GET_SOCK_HW_ADDR_FROM_OFFSET(sock0_base_addr, SOCK_REG_COMMAND),OPEN);

                //Read status from chip
                spi_rx_byte = wiznet_read(0x0403);

                if(spi_rx_byte != SOCK_INIT)
                {
                    WIZNET_WRITE(0x0403,CLOSE);
                    g_mainSM_state = 0xFF; //Reset PIC
                }

                //Set socket to listen
                WIZNET_WRITE(0x0401,LISTEN);

                //Read status from chip
                spi_rx_byte = wiznet_read(0x0403);

                if(spi_rx_byte != SOCK_LISTEN)
                {
                    WIZNET_WRITE(0x0403,CLOSE);
                    g_mainSM_state = 0xFF; //Reset PIC
                }

                g_mainSM_state = LISTENING;
                break;

            case LISTENING:
#ifdef KARRI_DEBUG
                debug_print("mainSM: case LISTENING\n");
#endif
                DELAY_BETWEEN_COMMANDS();
                //Read status from chip
                spi_rx_byte = wiznet_read(0x0403);

                if(spi_rx_byte == SOCK_ESTABLISHED)
                {
                    g_mainSM_state = PEER_CONNECTED;
                }
                break;

            case PEER_CONNECTED:
#ifdef KARRI_DEBUG
                debug_print("mainSM: case PEER_CONNECTED\n");
#endif
                //Get the size of received data size of socket 0
                spi_rx_byte = wiznet_read(0x0426);
                rx_data_size = (unsigned int)(spi_rx_byte << 8);
                spi_rx_byte = wiznet_read(0x0427);
                rx_data_size |= spi_rx_byte;


                if(rx_data_size > 0)
                {
                    g_mainSM_state = REQUEST_RECEIVED;
                }
                /*
                 * When working as a server and receiving data from
                 * a client, and the data size is too much for us
                 * (afterall, we only have 256 bytes of RAM), currently, on this MCU
                 * and this application, we will just RESET the CHIP and PIC,
                 * and we DULY NOTE THIS IS A SEVERE SECURITY HOLE IN A NORMAL
                 * NETWORK ENVIRONMENT. (but dont give a heck at the moment)
                 */
                if(rx_data_size > MAX_RX_SIZE)
                {
#ifdef KARRI_DEBUG
                    debug_print("RX too long, truncating!\n");
#endif
                    /* If the received data is longer than our receive buffer,
                     * we'll just truncate the data. */
                    rx_data_size = MAX_RX_SIZE - 1;
                }

                DELAY_BETWEEN_COMMANDS();
                //Check if socket has been closed
                spi_rx_byte = wiznet_read(0x0403);
                if(spi_rx_byte == SOCK_CLOSED ||
                   spi_rx_byte == SOCK_CLOSE_WAIT)
                {
#ifdef KARRI_DEBUG
                    debug_print("How rude! The socket was closed by the peer!\n");
#endif
                    WIZNET_WRITE(0x0403,CLOSE);
                    //g_mainSM_state = 0xFF; //reset PIC
                    g_mainSM_state = IDLE;
                }

                break;

            case REQUEST_RECEIVED:
#ifdef KARRI_DEBUG
                debug_print("mainSM: case REQUEST_RECEIVED\n");
#endif

#define KARRI_DEBUG_RX_PATH

#ifdef KARRI_DEBUG_RX_PATH
                sprintf( int_string, "RX size: %d\n", rx_data_size );
                debug_print(int_string);
#endif

#ifdef EXTENDED_FUNCTIONALITY

                //Get the read pointer to received data of socket 0
                spi_rx_byte            = wiznet_read(0x0428);
                rx_sock1_read_pointer  = (spi_rx_byte << 8);

                spi_rx_byte            = wiznet_read(0x0429);
                rx_sock1_read_pointer |= spi_rx_byte;

                

                rx_data_offset         = rx_sock1_read_pointer & g_rx_sock0_mask;
                rx_data_start_addr     = g_rx_sock0_base + rx_data_offset;

                if((rx_data_size + rx_data_offset) > (g_rx_sock0_mask + 1))
                {//buffer wrap-around occured
                    unsigned int left_size = 0;
                    rx_upper_size = (g_rx_sock0_mask + 1) - rx_data_offset;

                    wiznet_read_bytes(rx_data_start_addr, eth_buff, rx_upper_size);

                    left_size = rx_data_size - rx_upper_size;

                    wiznet_read_bytes(g_rx_sock0_base, eth_buff+rx_upper_size, left_size);
                }
                else
                {
                    wiznet_read_bytes(rx_data_start_addr, eth_buff, rx_data_size);
                }

                //Update RX buff pointer & send command for RECV done
                rx_sock1_read_pointer += rx_data_size;

                WIZNET_WRITE(0x0428,((rx_sock1_read_pointer & 0xFF00) >> 8));
                WIZNET_WRITE(0x0429,( rx_sock1_read_pointer & 0x00FF)      );

                DELAY_BETWEEN_COMMANDS();

                WIZNET_WRITE(0x0403,RECV);
#endif
                g_mainSM_state = PARSE_REQUEST;
                break;

            case PARSE_REQUEST:
#ifdef KARRI_DEBUG
                debug_print("mainSM: case PARSE_REQUEST\n");
#endif

#ifdef KARRI_DEBUG
                debug_print(eth_buff);
#endif
                DELAY_BETWEEN_COMMANDS();

                parse_status = parse_command(eth_buff, get_cmd, rx_data_size, (sizeof(get_cmd) - 1) ); /* We do minus 1 for the size because of the null character */

                if(parse_status >= PARSE_MATCH)
                {
                    switch(parse_status)
                    {
                        case TOGGLE_LED1:
                            LED1_TOGGLE();
                            break;
                        case TOGGLE_LED2:
                            LED2_TOGGLE();
                            break;
                        case TOGGLE_LED3:
                            LED3_TOGGLE();
                            break;
                        case TOGGLE_LED4:
                            LED4_TOGGLE();
                            break;
                    } 
                }

                zeromem(eth_buff, ETHERNET_BUFF_SIZE);
                
                g_mainSM_state = SEND_RESPONSE;
                break;
                
            case SEND_RESPONSE://send the web site as a response
#ifdef KARRI_DEBUG
                debug_print("mainSM: case SEND_RESPONSE\n");
#endif
                //wait for TX buffer size availability
                do {
                    tx_getsize_timeout_counter++;
                    delay(600);
                    //Get the size of TX socket 0
                    spi_rx_byte = wiznet_read(0x0420);
                    tx_data_size = (unsigned int)(spi_rx_byte << 8);
                    spi_rx_byte = wiznet_read(0x0421);
                    tx_data_size |= spi_rx_byte;

                    if(TX_GETSIZE_TIMEOUT_THRESHOLD < tx_getsize_timeout_counter)
                    {
                        WIZNET_WRITE(0x0403,CLOSE);
                        g_mainSM_state = IDLE;
                        break;
                    }
                } while(tx_data_size < (sizeof(website_body) + sizeof(website_body_end) + DYNAMIC_HTML_LINE_MAX_SIZE) );



                //Get the write pointer
                spi_rx_byte = wiznet_read(0x0424);
                tx_sock1_write_pointer = (spi_rx_byte << 8);
                spi_rx_byte = wiznet_read(0x0425);
                tx_sock1_write_pointer |= spi_rx_byte;

                tx_data_offset = tx_sock1_write_pointer & g_tx_sock0_mask;
                tx_data_start_addr = g_rx_sock0_base + tx_data_offset;

                { /* Send the website */
                    BYTE temp_dynamic_line_buff[DYNAMIC_HTML_LINE_MAX_SIZE];
                    zeromem(temp_dynamic_line_buff, sizeof(temp_dynamic_line_buff));

                    /*Test characters*/
                    temp_dynamic_line_buff[5] = 'K';
                    temp_dynamic_line_buff[6] = 'O';
                    temp_dynamic_line_buff[7] = 'R';
                    temp_dynamic_line_buff[8] = 'S';
                    temp_dynamic_line_buff[9] = 'O';
                    /**/

                    wiznet_write_bytes(tx_data_start_addr,
                                       website_body,
                                       sizeof(website_body));
                    tx_data_start_addr += sizeof(website_body);
                    wiznet_write_bytes(tx_data_start_addr,
                                       temp_dynamic_line_buff,
                                       sizeof(temp_dynamic_line_buff));
                    tx_data_start_addr += sizeof(temp_dynamic_line_buff);
                    wiznet_write_bytes(tx_data_start_addr + sizeof(website_body) + sizeof(temp_dynamic_line_buff),
                                       website_body_end,
                                       sizeof(website_body_end));
                    tx_data_start_addr += sizeof(website_body_end);
                    
                }
                tx_sock1_write_pointer = tx_data_start_addr;

                WIZNET_WRITE(0x0424,((tx_sock1_write_pointer & 0xFF00) >> 8));
                WIZNET_WRITE(0x0425,( tx_sock1_write_pointer & 0x00FF)      );

                DELAY_BETWEEN_COMMANDS();

                WIZNET_WRITE(0x0403,SEND);

                DELAY_BETWEEN_COMMANDS();
                DELAY_BETWEEN_COMMANDS();
                DELAY_BETWEEN_COMMANDS();
                DELAY_BETWEEN_COMMANDS();
                DELAY_BETWEEN_COMMANDS();
                DELAY_BETWEEN_COMMANDS();
                DELAY_BETWEEN_COMMANDS();

                debug_print("Sent! \n");

                WIZNET_WRITE(0x0403,CLOSE);
                g_mainSM_state = IDLE;
                break;
                
            default:/**ENDING UP HERE CAUSES A SOFTWARE RESET!**/
#ifdef KARRI_DEBUG
                debug_print("WARNING!! Doing a soft *RESET* to PIC soon!\n");
#endif

#ifdef KARRI_DEBUG
                WIZNET_WRITE(REG_MODE,MRREG_SOFT_RESET);
#endif //KARRI_DEBUG

                WDTCONbits.SWDTEN = 1; //Enable watchdog timer! Let it reset the damn thing!
#ifdef KARRI_DEBUG
                debug_print("Watchdog starts kicking, we shall RESET immediately!\n");
#endif
                //RESET
                break;
        }

        DELAY_BETWEEN_COMMANDS();
        DELAY_BETWEEN_COMMANDS();
    }

#ifdef KARRI_DEBUG
    //Indicate firmware finished
    LED4_ON();

    debug_print("Firmware end, looping forever\n");
    while(1);
#endif //KARRI_DEBUG

    return 0;
}

