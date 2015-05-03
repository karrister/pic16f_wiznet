/* 
 * File:   general.h
 * Author: karrister
 *
 * Created on 3. toukokuuta 2015, 21:55
 */

#ifndef GENERAL_H
#define	GENERAL_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "project_defs.h"
#include "project_macros.h"


/*********************************
 * Inline functions
 *********************************/

static inline unsigned int get_sock_base_addr_by_sock_num(unsigned int sock)
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


/**
* This inline function outputs debug messages out of the chip UART bus.
* @param msg The debug message. IMPORTANT: The last char needs to be newline,
*            otherwise the function will block forever.
* @return void
*/
static inline void debug_print(DEBUG_MSG *msg)
{
#ifdef UART_DEBUG
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
#else
    ;
#endif //UART_DEBUG
}


/**
* This inline function reads a single byte from the chip. This function manages
* the data transfer over the SPI bus from the chip towards the master.
* @param addr The address inside the chip to read from.
* @return The byte read from the chip.
*/
static inline unsigned char wiznet_read(unsigned int addr)
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
static inline KARRI_BOOL compare_buffers(BYTE *eth_buff, BYTE *cmd, unsigned int input_size, unsigned int cmd_size, unsigned int *buffer_offset)
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

/**
* This function zeroes out a specified amount of memory, starting from the
* address given.
* @param memory Pointer to the buffer to zero out.
* @param size The size of the buffer to zero out.
* @see main()
* @return void
*/
static inline void zeromem(BYTE *memory, unsigned int size)
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
static inline PARSE_STATUS parse_command(BYTE *eth_buff, BYTE *cmd, unsigned int input_size, unsigned int cmd_size)
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


/*********************************
 * Functions prototypes
 *********************************/


/**
* This function runs a delay loop the specified amount of times.
* @param count The amount of times to run the delay loop.
* @see main()
* @return void
*/
void delay(unsigned int count);

/**
* Reads the specified amount of bytes from the Wiznet W5100 chip.
* @param chip_addr The address in the chip address range.
* @param rx_buff Pointer to the PIC memory to read to.
* @param len The length of the data to read.
* @see main()
* @return void
*/
void wiznet_read_bytes(unsigned int chip_addr, BYTE *rx_buff, unsigned int len);

/**
* Writes the specified amount of bytes from the Wiznet W5100 chip.
* @param chip_addr The address in the chip address range.
* @param tx_buff Pointer to the PIC memory to write from.
* @param len The length of the data to write.
* @see main()
* @return void
*/
void wiznet_write_bytes(unsigned int chip_addr, BYTE *tx_buff, unsigned int len);

//TODO doxygen
short test_wiznet_connection(void);

//TODO doxygen
void init_wiznet(void);

//TODO doxygen
void dev_init(void);

#ifdef	__cplusplus
}
#endif

#endif	/* GENERAL_H */

