#ifndef __SER_BRIDGE_H__
#define __SER_BRIDGE_H__

#include <ip_addr.h>
#include <c_types.h>
#include <espconn.h>

#define MAX_CONN 1
#define SER_BRIDGE_TIMEOUT 300 // 300 seconds = 5 minutes

// Send buffer size
#define MAX_TXBUFFER (2*1460)
#define MAX_RXBUFFER (8*1460)

typedef struct serbridgeConnData {
  uint16         rxbufferlen;   // length of data in rxbuffer
  char           *rxbuffer;     // buffer for received data
  struct espconn *conn;
  uint8_t        telnet_state;
  uint16         txbufferlen;   // length of data in txbuffer
  char           *txbuffer;     // buffer for the data to send
  char           *sentbuffer;   // buffer sent, awaiting callback to get freed
  uint32_t       txoverflow_at; // when the transmitter started to overflow
  bool           readytosend;   // true, if txbuffer can be sent by espconn_sent
} serbridgeConnData;

// port1 is transparent
void ICACHE_FLASH_ATTR serbridgeInit(int port1);
void ICACHE_FLASH_ATTR serbridgeUartCb(char *buf, short len);
void ICACHE_FLASH_ATTR serbridgeReset();

#endif /* __SER_BRIDGE_H__ */
