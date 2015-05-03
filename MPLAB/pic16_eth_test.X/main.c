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


#include "general.h"


int main(void)
{
    short spi_status = KARRI_NOK;

    //Init device and in chip peripherals
    dev_init();
    
    //Sleep after init (needed?)
    delay(60000);
    delay(60000);
    delay(60000);
    delay(60000);
    delay(60000);


#ifdef INIT_DEBUG
    //Indicate init finished, firmware started
    LED1_ON();
    debug_print("Init finished, LED1 on, FW started!\n");

#endif //INIT_DEBUG

#ifdef SPI_DEBUG //When suspect problems with SPI bus
    //debug_print("Testing connection to CHIP!\n");

    delay(60000);
    delay(60000);
    delay(60000);
    delay(60000);
    delay(60000);

    spi_status = test_wiznet_connection();

    if(spi_status != KARRI_OK)
    {
#ifdef KARRI_DEBUG
        //Indicate connection to wiznet was NOK
        //debug_print("Connection to CHIP failed!\n");
        //LED3_ON();
#endif //KARRI_DEBUG
        while(1);
    }
    
#ifdef KARRI_DEBUG
    //Indicate connection to wiznet was OK
    debug_print("Connection to CHIP OK\n");
    LED1_ON();
#endif //KARRI_DEBUG

#endif //SPI_DEBUG
    
    //Init ETH
    init_wiznet();

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

    BYTE eth_buff[ETHERNET_BUFF_SIZE];

    unsigned int sock0_base_addr = 0;

    //Get sock0 base address for all the subsequent calls
    sock0_base_addr = get_sock_base_addr_by_sock_num(SOCK0);

    while(1)
    {
        switch(g_mainSM_state)
        {
            case IDLE:
#ifdef KARRI_DEBUG_SM
                debug_print("mainSM: case IDLE\n");
#endif
                //Make sure socket is closed!
                spi_rx_byte = wiznet_read(SOCKET1_REG_STATUS);

                if(spi_rx_byte != SOCK_CLOSED)
                {
                    g_mainSM_state = CLOSE_SOCKET;
                    break;
                }

                /**SET SERVER SOCKET**/
                //Set TCP mode
                WIZNET_WRITE(GET_SOCK_HW_ADDR_FROM_OFFSET(sock0_base_addr, SOCK_REG_MODE), SOCK_MODE_TCP);
                //Set port to 80 (www);
                WIZNET_WRITE(GET_SOCK_HW_ADDR_FROM_OFFSET(sock0_base_addr, SOCK_REG_SOURCE_PORT0), L4_PORT_NO_MSB);
                WIZNET_WRITE(GET_SOCK_HW_ADDR_FROM_OFFSET(sock0_base_addr, SOCK_REG_SOURCE_PORT1), L4_PORT_80_LSB);
                //Set socket to open
                WIZNET_WRITE(GET_SOCK_HW_ADDR_FROM_OFFSET(sock0_base_addr, SOCK_REG_COMMAND), OPEN);

                /* Block while chip is processing command */
                do {
                    spi_rx_byte = wiznet_read(SOCKET1_REG_COMMAND);
                } while(spi_rx_byte);

                //Read status from chip and make sure it is correct
                spi_rx_byte = wiznet_read(SOCKET1_REG_STATUS);
                if(spi_rx_byte != SOCK_INIT)
                {
                    g_mainSM_state = CLOSE_SOCKET;
                    break;
                }

                //Set socket to listen
                WIZNET_WRITE(SOCKET1_REG_COMMAND,LISTEN);

                /* Block while chip is processing command */
                do {
                    spi_rx_byte = wiznet_read(SOCKET1_REG_COMMAND);
                } while(spi_rx_byte);

                //Set the next state here - however if the socket state is not
                //correct we might still change this
                g_mainSM_state = LISTENING;

                //Read status from chip and make sure it is correct
                spi_rx_byte = wiznet_read(SOCKET1_REG_STATUS);
                if(spi_rx_byte != SOCK_LISTEN)
                {
                    g_mainSM_state = CLOSE_SOCKET;
                }

                break;

            case LISTENING:
#ifdef KARRI_DEBUG_SM
                debug_print("mainSM: case LISTENING\n");
#endif
                DELAY_BETWEEN_COMMANDS();
                //Read status from chip
                spi_rx_byte = wiznet_read(SOCKET1_REG_STATUS);

                //If the socket was established while listening, move to next state
                if(spi_rx_byte == SOCK_ESTABLISHED)
                {
                    g_mainSM_state = PEER_CONNECTED;
                }
                //Or if the socket is not at listening state, we need to start over
                else if(spi_rx_byte != SOCK_LISTEN)
                {
                    g_mainSM_state = CLOSE_SOCKET;
                }

                //Else, continue at listening state

                break;

            case PEER_CONNECTED:
#ifdef KARRI_DEBUG_SM
                debug_print("mainSM: case PEER_CONNECTED\n");
#endif
                //TODO: move these right before the break!
                //Get the size of received data size of socket 0
                spi_rx_byte = wiznet_read(SOCKET1_REG_RX_SIZE_MSB);
                rx_data_size = (unsigned int)(spi_rx_byte << 8);
                spi_rx_byte = wiznet_read(SOCKET1_REG_RX_SIZE_LSB);
                rx_data_size |= spi_rx_byte;


                //If any data was received, we should move to the next state
                if(rx_data_size > 0)
                {
                    g_mainSM_state = REQUEST_RECEIVED;
                }

                /*
                 * Our cute little MCU doesn't have a lot of RAM. Thus if the
                 * received data size is too much, we will just truncate the
                 * message.
                 */
                if(rx_data_size > MAX_RX_SIZE)
                {
                    rx_data_size = MAX_RX_SIZE - 1;
                }

                DELAY_BETWEEN_COMMANDS();
                //Check if socket has been closed
                spi_rx_byte = wiznet_read(SOCKET1_REG_STATUS);
                if(spi_rx_byte == SOCK_CLOSED ||
                   spi_rx_byte == SOCK_CLOSE_WAIT)
                {
                    g_mainSM_state = CLOSE_SOCKET;
                    break;
                }

                break;

            case REQUEST_RECEIVED:
#ifdef KARRI_DEBUG_SM
                debug_print("mainSM: case REQUEST_RECEIVED\n");
#endif

#ifdef KARRI_DEBUG_RX_PATH
                sprintf( int_string, "RX size: %d\n", rx_data_size );
                debug_print(int_string);
#endif

                //Get the read pointer to received data of socket 0
                spi_rx_byte            = wiznet_read(SOCKET1_REG_RX_READ_PTR_MSB);
                rx_sock1_read_pointer  = (spi_rx_byte << 8);

                spi_rx_byte            = wiznet_read(SOCKET1_REG_RX_READ_PTR_LSB);
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

                WIZNET_WRITE(SOCKET1_REG_RX_READ_PTR_MSB,((rx_sock1_read_pointer & 0xFF00) >> 8));
                WIZNET_WRITE(SOCKET1_REG_RX_READ_PTR_LSB,( rx_sock1_read_pointer & 0x00FF)      );

                DELAY_BETWEEN_COMMANDS();

                WIZNET_WRITE(SOCKET1_REG_COMMAND,RECV);

                /* Block while chip is receiving */
                do {
                    spi_rx_byte = wiznet_read(SOCKET1_REG_COMMAND);
                } while(spi_rx_byte);

                g_mainSM_state = PARSE_REQUEST;
                break;

            case PARSE_REQUEST:
#ifdef KARRI_DEBUG_SM
                debug_print("mainSM: case PARSE_REQUEST\n");
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
#ifdef KARRI_DEBUG_SM
                debug_print("mainSM: case SEND_RESPONSE\n");
#endif
                //Check if socket has been closed
                spi_rx_byte = wiznet_read(SOCKET1_REG_STATUS);
                if(spi_rx_byte != SOCK_ESTABLISHED)
                {
                    g_mainSM_state = DISCONNECT_SOCKET;
                    break;
                }
                //wait for TX buffer size availability
                do {
                    tx_getsize_timeout_counter++;
                    delay(600);
                    //Get the size of TX socket 0
                    spi_rx_byte = wiznet_read(SOCKET1_REG_TX_FREE_SIZE_MSB);
                    tx_data_size = (unsigned int)(spi_rx_byte << 8);
                    spi_rx_byte = wiznet_read(SOCKET1_REG_TX_FREE_SIZE_LSB);
                    tx_data_size |= spi_rx_byte;

                    if(TX_GETSIZE_TIMEOUT_THRESHOLD < tx_getsize_timeout_counter)
                    {
                        g_mainSM_state = DISCONNECT_SOCKET;
                        break;
                    }
                } while(tx_data_size < (sizeof(website_body) + sizeof(website_body_end) + DYNAMIC_HTML_LINE_MAX_SIZE) );

                //Get the write pointer
                spi_rx_byte = wiznet_read(SOCKET1_REG_TX_WRITE_PTR_MSB);
                tx_sock1_write_pointer = ((spi_rx_byte & 0x00FF) << 8);
                spi_rx_byte = wiznet_read(SOCKET1_REG_TX_WRITE_PTR_LSB);
                tx_sock1_write_pointer |= (spi_rx_byte & 0x00FF);

                tx_data_offset = tx_sock1_write_pointer & g_tx_sock0_mask;
                tx_data_start_addr = g_tx_sock0_base + tx_data_offset;
#if 1//karri test
                { /* Send the website */

                    BYTE temp_dynamic_line_buff[DYNAMIC_HTML_LINE_MAX_SIZE];
                    zeromem(temp_dynamic_line_buff, sizeof(temp_dynamic_line_buff));
#if 1
                    /*Test characters*/
                    temp_dynamic_line_buff[5] = 'K';
                    temp_dynamic_line_buff[6] = 'O';
                    temp_dynamic_line_buff[7] = 'R';
                    temp_dynamic_line_buff[8] = 'S';
                    temp_dynamic_line_buff[9] = 'O';
                    /**/
#endif

                    wiznet_write_bytes(tx_data_start_addr,
                                       website_body,
                                       sizeof(website_body));
                    tx_data_start_addr += sizeof(website_body);
#if 1
                    wiznet_write_bytes(tx_data_start_addr,
                                       temp_dynamic_line_buff,
                                       sizeof(temp_dynamic_line_buff));
                    tx_data_start_addr += sizeof(temp_dynamic_line_buff);
#endif

                    wiznet_write_bytes(tx_data_start_addr,
                                       website_body_end,
                                       sizeof(website_body_end));
                    tx_data_start_addr += sizeof(website_body_end);
                
                    tx_sock1_write_pointer = tx_sock1_write_pointer + (sizeof(website_body) + DYNAMIC_HTML_LINE_MAX_SIZE + sizeof(website_body_end));

                }
                

                WIZNET_WRITE(SOCKET1_REG_TX_WRITE_PTR_MSB,((tx_sock1_write_pointer & 0xFF00) >> 8));
                WIZNET_WRITE(SOCKET1_REG_TX_WRITE_PTR_LSB,( tx_sock1_write_pointer & 0x00FF)      );

                DELAY_BETWEEN_COMMANDS();

                WIZNET_WRITE(SOCKET1_REG_COMMAND,SEND);

                /* Block while chip is sending */
                do {
                    spi_rx_byte = wiznet_read(SOCKET1_REG_COMMAND);
                } while(spi_rx_byte);
#endif //karri test

                g_mainSM_state = DISCONNECT_SOCKET;
                break;

            case DISCONNECT_SOCKET:

                WIZNET_WRITE(SOCKET1_REG_COMMAND,DISCON);

                /* Block while chip is disconnecting the socket */
                do {
                    spi_rx_byte = wiznet_read(SOCKET1_REG_COMMAND);
                } while(spi_rx_byte);
                
                g_mainSM_state = CLOSE_SOCKET;
                /* fall-through to close socket */
            case CLOSE_SOCKET:

                WIZNET_WRITE(SOCKET1_REG_COMMAND,CLOSE);

                /* Block while chip is closing the socket */
                do {
                    spi_rx_byte = wiznet_read(SOCKET1_REG_COMMAND);
                } while(spi_rx_byte);

                g_mainSM_state = IDLE;
                break;
/***************************************************************/
            case RESET_MCU:
                /* fall-through */
            default:/**ENDING UP HERE CAUSES A SOFTWARE RESET!**/
#ifdef KARRI_DEBUG
                debug_print("WARNING!! Doing a soft *RESET* to PIC soon!\n");
#endif
                WDTCONbits.SWDTEN = 1; //Enable watchdog timer! Let it reset the damn thing!
#ifdef KARRI_DEBUG
                debug_print("Watchdog starts kicking, we shall RESET immediately!\n");
#endif
                //RESET
                break;
        }
/***************************************************************/

        DELAY_BETWEEN_COMMANDS();
        DELAY_BETWEEN_COMMANDS();
    } //Main loop

#ifdef KARRI_DEBUG
    //Indicate firmware finished
    LED4_ON();

    debug_print("Firmware end, looping forever\n");
    while(1);
#endif //KARRI_DEBUG

    return 0;
}

