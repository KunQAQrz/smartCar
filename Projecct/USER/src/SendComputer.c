#include "KEA128_uart.h"

void Send_Ware(void *wareaddr, uint32 waresize)
{
  #define CMD_WARE  3
  uint8 cmdf[2]={CMD_WARE, ~CMD_WARE};
  uint8 cmdr[2]={~CMD_WARE, CMD_WARE};
  
  uart_putbuff (DEBUG_PORT1, cmdf, sizeof(cmdf));
  uart_putbuff (DEBUG_PORT1,(uint8 *)wareaddr, waresize);
  uart_putbuff (DEBUG_PORT1, cmdr, sizeof(cmdr));
}