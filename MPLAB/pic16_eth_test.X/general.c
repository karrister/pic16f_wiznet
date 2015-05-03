/*
 * File:   general.c
 * Author: karrister
 *
 * Created on 3. toukokuuta 2015, 21:54
 */

#include "general.h"

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
    //WIZNET_WRITE(REG_RX_MEM_SIZE,0x0A);
    WIZNET_WRITE(REG_RX_MEM_SIZE,0x55); //2k sockets
    DELAY_BETWEEN_COMMANDS();
    //TMSR register - setting 4K to sockets 0 & 1
    //WIZNET_WRITE(REG_TX_MEM_SIZE,0x0A);
    WIZNET_WRITE(REG_TX_MEM_SIZE,0x55); //2k sockets
    DELAY_BETWEEN_COMMANDS();

    g_rx_sock0_base = CHIP_BASE_ADDRESS + RX_MEMORY_BASE_ADDRESS;
    g_rx_sock0_mask = 0x07FF; //0x0FFF; //2k sockets


    g_tx_sock0_base = CHIP_BASE_ADDRESS + TX_MEMORY_BASE_ADDRESS;
    g_tx_sock0_mask = 0x07FF; //0x0FFF; //2k sockets

#ifdef DEBUG_INIT
    LED1_OFF();
    debug_print("Went through CHIP init, entering main state machine!\n");
#endif

}


void dev_init(void)
{
    /*Clock*/
    OSCCON = 0x08; //clock from fOSC

    //Use all those as DIGITAL input
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

    delay(60000);
    delay(60000);

    /*This is for some reason important!*/
    CS_ASSERT();
    CS_DEASSERT();
}

