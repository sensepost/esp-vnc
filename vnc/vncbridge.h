#ifndef __VNC_BRIDGE_H__
#define __VNC_BRIDGE_H__

#include <ip_addr.h>
#include <c_types.h>
#include <espconn.h>

#define VNC_MAX_CONN 1
#define VNC_BRIDGE_TIMEOUT 300 // 300 seconds = 5 minutes

typedef enum {
  CLIENT_HELLO, // client has connected, we expect a hello
  CLIENT_AUTH,  // challenge sent, await auth
  CLIENT_INIT,  // expect client initialisation messages
  RFB_MESSAGE,  // expect RFB messages
  CUT_TEXT      // Received CUT_TEXT message, now read the data
} VncState;

typedef enum {
  SetPixelFormat = 0,
  FixColourMapEntries = 1,
  SetEncodings = 2,
  FrameBufferUpdateRequest = 3,
  KeyEvent = 4,
  PointerEvent = 5,
  ClientCutText = 6
} rfb;

typedef struct vncbridgeConnData {
  struct espconn *conn;
  uint16         rxbufferlen;   // length of data in rxbuffer
  char           *rxbuffer;     // buffer for received data
  bool		 recv_hold;     // is the connection on hold
  VncState       state;         // the next message to be processed
  uint32         cut_text;      // how much data to be read in cut_text state
  uint16         txbufferlen;   // length of data in txbuffer
  char           *txbuffer;     // buffer for the data to send
  char           *sentbuffer;   // buffer sent, awaiting callback to get freed
  uint32_t       txoverflow_at; // when the transmitter started to overflow
  bool           readytosend;   // true, if txbuffer can be sent by espconn_sent
} vncbridgeConnData;

void ICACHE_FLASH_ATTR vncbridgeInit(int port);
void ICACHE_FLASH_ATTR vncbridgeInitPins(void);
void ICACHE_FLASH_ATTR vncbridgeUartCb(char *buf, short len);
void ICACHE_FLASH_ATTR vncbridgeReset();
// int8 ICACHE_FLASH_ATTR vnc_proto_handler(vncbridgeConnData *conn, char *data, unsigned short len);

#endif /* __VNC_BRIDGE_H__ */
