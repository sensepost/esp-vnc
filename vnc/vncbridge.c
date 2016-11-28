// Copyright 2015 by Thorsten von Eicken, see LICENSE.txt

#include "esp8266.h"

#include "uart.h"
#include "vncbridge.h"
#include "config.h"
#ifdef SYSLOG
#include "syslog.h"
#else
#define syslog(a,...)
#endif
#include "tlv.h"
#include "task.h"

#define SKIP_AT_RESET

// Send buffer size
#define MAX_TXBUFFER (1460)
#define MAX_RXBUFFER (6*1460)

#define VNCBR_DBG
#ifdef VNCBR_DBG
#define DBG(format, ...) do { os_printf(format, ## __VA_ARGS__); } while(0)
#define DBG_RATE(timeout, format, ...) do { static uint32_t t = 0; if (system_get_time() - t > (timeout)) { DBG(format, ## __VA_ARGS__); t = system_get_time(); } } while (0);
#else
#define DBG(format, ...) do { } while(0)
#define DBG_RATE(format, ...)
#endif

LOCAL uint8_t deferredTaskNum;

static struct espconn vncbridgeConn; // plain bridging port
static esp_tcp vncbridgeTcp;

static const char RFB_HELLO[] = { 'R','F','B',' ','0','0','3','.','0','0','3','\n' };

static const char AUTH_CHALLENGE[] = { 0x00, 0x00, 0x00, 0x02 /* vncAuth */,
  0x48, 0x45, 0x2e, 0x4c, 0x7f, 0x2c, 0x50, 0x3c, 0x61, 0x1c, 0x7d, 0x7c, 0x3b, 0x6d, 0x42, 0x25 /* random bytes */};

// "password"
static const char AUTH_RESPONSE[] = { 0x55, 0xB3, 0x8b, 0x6, 0xAE, 0xE1, 0xD2, 0x1A, 0x6, 0x54, 0x9C, 0xEB, 0xE9,
  0x0, 0xF, 0x6E };
static const char AUTH_OK[] = { 0x00, 0x00, 0x00, 0x00 };
static const char AUTH_FAILED[] = { 0x00, 0x00, 0x00, 0x01 /* auth failed */, 0x00, 0x00, 0x00, 0x00 /* connfailed */};

static const char INIT_MESSAGE[] = { 0x0B, 0x40 /* 2880 */, 0x07, 0x08 /* 1800 */, /* X/Y resolution. Make it huge to accommodate all screens */
  0x08, 0x08, 0x00, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x03, 0x00, 0x03, 0x06 /* pixelformat */, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x06 /* length */, 'V', 'N', 'C', '_', 'K', 'M' };
  
// Connection pool
vncbridgeConnData vncConnData[VNC_MAX_CONN];

static void ICACHE_FLASH_ATTR
vncProcessRX(vncbridgeConnData *conn);

static bool ICACHE_FLASH_ATTR
emitKeyEvent(bool pressed, uint32 key);

//===== uC -> TCP

// Send all data in conn->txbuffer
// returns result from espconn_sent if data in buffer or ESPCONN_OK (0)
// Use only internally from espbuffsend and vncbridgeSentCb
static sint8 ICACHE_FLASH_ATTR
sendtxbuffer(vncbridgeConnData *conn)
{
  sint8 result = ESPCONN_OK;
  if (conn->txbufferlen != 0) {
    // DBG("TX %p %d\n", conn, conn->txbufferlen);
    conn->readytosend = false;
    result = espconn_sent(conn->conn, (uint8_t*)conn->txbuffer, conn->txbufferlen);
    conn->txbufferlen = 0;
    if (result != ESPCONN_OK) {
      os_printf("sendtxbuffer: espconn_sent error %d on conn %p\n", result, conn);
      conn->txbufferlen = 0;
      if (!conn->txoverflow_at) conn->txoverflow_at = system_get_time();
    } else {
      conn->sentbuffer = conn->txbuffer;
      conn->txbuffer = NULL;
      conn->txbufferlen = 0;
    }
  }
  return result;
}

// espbuffsend adds data to the send buffer. If the previous send was completed it calls
// sendtxbuffer and espconn_sent.
// Returns ESPCONN_OK (0) for success, -128 if buffer is full or error from  espconn_sent
// Use espbuffsend instead of espconn_sent as it solves the problem that espconn_sent must
// only be called *after* receiving an espconn_sent_callback for the previous packet.
static sint8 ICACHE_FLASH_ATTR
espbuffsend(vncbridgeConnData *conn, const char *data, uint16 len)
{
  if (conn->txbufferlen >= MAX_TXBUFFER) goto overflow;

  // make sure we indeed have a buffer
  if (conn->txbuffer == NULL) conn->txbuffer = os_zalloc(MAX_TXBUFFER);
  if (conn->txbuffer == NULL) {
    os_printf("vnc_espbuffsend: cannot alloc tx buffer\n");
    return -128;
  }

  // add to send buffer
  uint16_t avail = conn->txbufferlen+len > MAX_TXBUFFER ? MAX_TXBUFFER-conn->txbufferlen : len;
  os_memcpy(conn->txbuffer + conn->txbufferlen, data, avail);
  conn->txbufferlen += avail;

  // try to send
  sint8 result = ESPCONN_OK;
  if (conn->readytosend) {
    result = sendtxbuffer(conn);
  } else {
    // syslog(SYSLOG_FAC_USER, SYSLOG_PRIO_NOTICE, "esp-link", "espbuffsend: Not ready to send\n");
  }

  if (avail < len && result == ESPCONN_OK) {
    // syslog(SYSLOG_FAC_USER, SYSLOG_PRIO_NOTICE, "esp-link", "espbuffsend: some data didn't fit in the buffer\n");
    // some data didn't fit into the buffer
    if (conn->txbufferlen == 0) {
      // we sent the prior buffer, so try again
      return espbuffsend(conn, data+avail, len-avail);
    }
    goto overflow;
  }
  // syslog(SYSLOG_FAC_USER, SYSLOG_PRIO_NOTICE, "esp-link", "espbuffsend: result is %d\n", result);
  return result;

overflow:
  if (conn->txoverflow_at) {
    // we've already been overflowing
    if (system_get_time() - conn->txoverflow_at > 10*1000*1000) {
      // no progress in 10 seconds, kill the connection
      os_printf("vncbridge: killing overflowing stuck conn %p\n", conn);
      espconn_disconnect(conn->conn);
    }
    // else be silent, we already printed an error
  } else {
    // print 1-time message and take timestamp
    os_printf("vncbridge: txbuffer full, conn %p\n", conn);
    conn->txoverflow_at = system_get_time();
  }
  return -128;
}

static sint8 ICACHE_FLASH_ATTR
espbuffsend_static(vncbridgeConnData *conn, const char *data, uint16 len) {
  char *buff = os_zalloc(len);
  if (buff == NULL) {
    os_printf("espbuffsend_static: cannot alloc tx buffer\n");
    return -128;
  }
  
  os_memcpy(buff, data, len);
  sint8 ret = espbuffsend(conn, buff, len);
  os_free(buff);
  return ret;
}

//callback after the data are sent
static void ICACHE_FLASH_ATTR
vncbridgeSentCb(void *arg)
{
  vncbridgeConnData *conn = ((struct espconn*)arg)->reverse;
  //DBG("Sent CB %p\n", conn);
  if (conn == NULL) return;
  if (conn->sentbuffer != NULL) os_free(conn->sentbuffer);
  conn->sentbuffer = NULL;
  conn->readytosend = true;
  conn->txoverflow_at = 0;
  sendtxbuffer(conn); // send possible new data in txbuffer
}

//===== Connect / disconnect

// Disconnection callback
static void ICACHE_FLASH_ATTR
vncbridgeDisconCb(void *arg)
{
  vncbridgeConnData *conn = ((struct espconn*)arg)->reverse;
  if (conn == NULL) return;
  DBG("Closing connection\n");
  // Free buffers
  if (conn->sentbuffer != NULL) os_free(conn->sentbuffer);
  conn->sentbuffer = NULL;
  if (conn->txbuffer != NULL) os_free(conn->txbuffer);
  conn->txbuffer = NULL;
  conn->txbufferlen = 0;
  if (conn->rxbuffer != NULL && conn->rxbufferlen == 0) {
    DBG("VNC Freed RX buffer\n");
    os_free(conn->rxbuffer);
    DBG("VncDisc: RX at %p\n", conn->rxbuffer);
    conn->rxbuffer = NULL;
  } else {
    DBG("VNC RX buffer still has data, leaving it\n");
  }
  conn->conn = NULL;
}

// Connection reset callback (note that there will be no DisconCb)
static void ICACHE_FLASH_ATTR
vncbridgeResetCb(void *arg, sint8 err)
{
  os_printf("vncbridge: connection reset err=%d\n", err);
  vncbridgeDisconCb(arg);
}


// Receive callback
static void ICACHE_FLASH_ATTR
vncbridgeRecvCb(void *arg, char *data, unsigned short len)
{
  vncbridgeConnData *conn = ((struct espconn*)arg)->reverse;
  if (conn == NULL) return;
  if (conn->rxbufferlen + len > MAX_RXBUFFER) {
    os_printf("RX buffer overrun!\n");
    espconn_disconnect(conn->conn);
    return;
  }
  sint8_t res = espconn_recv_hold(conn->conn);
  if (res != 0) os_printf("Hold: %d\n", res);
  os_memcpy(conn->rxbuffer+conn->rxbufferlen, data, len);
  conn->rxbufferlen += len;
  // DBG("RX += %d, now %d\n", len, conn->rxbufferlen);
  post_usr_task(deferredTaskNum, 0);
}


// New connection callback, use one of the connection descriptors, if we have one left.
static void ICACHE_FLASH_ATTR
vncbridgeConnectCb(void *arg)
{
  struct espconn *conn = arg;
  // Find empty conndata in pool
  int i;
  for (i=0; i<VNC_MAX_CONN; i++) if (vncConnData[i].conn==NULL) break;
  DBG("Accept port %d, conn=%p, pool slot %d\n", conn->proto.tcp->local_port, conn, i);
  if (i==VNC_MAX_CONN) {
    os_printf("Aiee, conn pool overflow!\n");
    espconn_disconnect(conn);
    return;
  }

  os_memset(vncConnData+i, 0, sizeof(struct vncbridgeConnData));
  vncConnData[i].conn = conn;
  conn->reverse = vncConnData+i;
  vncConnData[i].readytosend = true;

  // allocate the rx buffer
  vncConnData[i].rxbuffer = os_zalloc(MAX_RXBUFFER);
  vncConnData[i].rxbufferlen = 0;
  if (vncConnData[i].rxbuffer == NULL) {
    os_printf("Out of memory for RX buffer\n");
    espconn_disconnect(conn);
    return;
  }

  espconn_regist_recvcb(conn, vncbridgeRecvCb);
  espconn_regist_disconcb(conn, vncbridgeDisconCb);
  espconn_regist_reconcb(conn, vncbridgeResetCb);
  espconn_regist_sentcb(conn, vncbridgeSentCb);

  espconn_set_opt(conn, ESPCONN_REUSEADDR|ESPCONN_NODELAY);
  espbuffsend_static(&vncConnData[i], RFB_HELLO, sizeof(RFB_HELLO));
  vncConnData[i].state = CLIENT_HELLO;
}

// Internal functions

static uint8_t ICACHE_FLASH_ATTR
getModifier(uint32 key) {
  switch (key) {
  case 0xFFE3u: return 1 << 0; // left ctrl
  case 0xFFE1u: return 1 << 1; // left shift
  case 0xFFE9u: return 1 << 2; // left alt
  case 0xFFE7u: return 1 << 3; // left gui
  case 0xFFE4u: return 1 << 4; // right ctrl
  case 0xFFE2u: return 1 << 5; // right shift;
  case 0xFFEAu: return 1 << 6; // right alt
  case 0xFFE8u: return 1 << 7; // right gui
  case 0x21: // Keyboard 1 and !
  case 0x40: // Keyboard 2 and @
  case 0x23: // Keyboard 3 and #
  case 0x24: // Keyboard 4 and $
  case 0x25: // Keyboard 5 and %
  case 0x5E: // Keyboard 6 and ^
  case 0x26: // Keyboard 7 and &
  case 0x2A: // Keyboard 8 and *
  case 0x28: // Keyboard 9 and (
  case 0x29: // Keyboard 0 and )
  case 0x5F: // Keyboard - and (underscore)
  case 0x2B: // Keyboard = and +
  case 0x7B: // Keyboard [ and {
  case 0x7D: // Keyboard ] and }
  case 0x7C: // Keyboard \ and |
  // case unknown: // Keyboard Non-US # and ~
  case 0x3A: // Keyboard ; and :
  case 0x22: // Keyboard ' and "
  case 0x7E: // Keyboard Grave Accent and Tilde
  case 0x3C: // Keyboard, and <
  case 0x3E: // Keyboard . and >
  case 0x3F: // Keyboard / and ?
    return 1 << 1; // left shift
  default: 
    if (key != tolower(key))
      return 1 << 1; // left shift
  }
  return 0;
}

static uint8_t ICACHE_FLASH_ATTR
mapKey(uint32 key) {
  if (key >= 0x41 && key <= 0x5A) // Map capital letters to lowercase
    key |= 0x20;
  
  switch (key) {
  case 0x61: return 0x4; // Keyboard a and A
  case 0x62: return 0x5; // Keyboard b and B
  case 0x63: return 0x6; // Keyboard c and C
  case 0x64: return 0x7; // Keyboard d and D
  case 0x65: return 0x8; // Keyboard e and E
  case 0x66: return 0x9; // Keyboard f and F
  case 0x67: return 0xA; // Keyboard g and G
  case 0x68: return 0xB; // Keyboard h and H
  case 0x69: return 0xC; // Keyboard i and I
  case 0x6A: return 0xD; // Keyboard j and J
  case 0x6B: return 0xE; // Keyboard k and K
  case 0x6C: return 0xF; // Keyboard l and L
  case 0x6D: return 0x10; // Keyboard m and M
  case 0x6E: return 0x11; // Keyboard n and N
  case 0x6F: return 0x12; // Keyboard o and O
  case 0x70: return 0x13; // Keyboard p and P
  case 0x71: return 0x14; // Keyboard q and Q
  case 0x72: return 0x15; // Keyboard r and R
  case 0x73: return 0x16; // Keyboard s and S
  case 0x74: return 0x17; // Keyboard t and T
  case 0x75: return 0x18; // Keyboard u and U
  case 0x76: return 0x19; // Keyboard v and V
  case 0x77: return 0x1A; // Keyboard w and W
  case 0x78: return 0x1B; // Keyboard x and X
  case 0x79: return 0x1C; // Keyboard y and Y
  case 0x7A: return 0x1D; // Keyboard z and Z
  case 0x31: case 0x21: return 0x1E; // Keyboard 1 and !
  case 0x32: case 0x40: return 0x1F; // Keyboard 2 and @
  case 0x33: case 0x23: return 0x20; // Keyboard 3 and #
  case 0x34: case 0x24: return 0x21; // Keyboard 4 and $
  case 0x35: case 0x25: return 0x22; // Keyboard 5 and %
  case 0x36: case 0x5E: return 0x23; // Keyboard 6 and ^
  case 0x37: case 0x26: return 0x24; // Keyboard 7 and &
  case 0x38: case 0x2A: return 0x25; // Keyboard 8 and *
  case 0x39: case 0x28: return 0x26; // Keyboard 9 and (
  case 0x30: case 0x29: return 0x27; // Keyboard 0 and )
  case 0xFF0Du: return 0x28; // Keyboard Return (ENTER)
  case 0xFF1Bu: return 0x29; // Keyboard ESCAPE
  case 0xFF08u: return 0x2A; // Keyboard DELETE (Backspace)
  case 0xFF09u: return 0x2B; // Keyboard Tab
  case 0x20: return 0x2C; // Keyboard Spacebar
  case 0x2D: case 0x5F: return 0x2D; // Keyboard - and (underscore)
  case 0x3D: case 0x2B: return 0x2E; // Keyboard = and +
  case 0x5B: case 0x7B: return 0x2F; // Keyboard [ and {
  case 0x5D: case 0x7D: return 0x30; // Keyboard ] and }
  case 0x5C: case 0x7C: return 0x31; // Keyboard \ and |
  // case unknown: return 0x32; // Keyboard Non-US # and ~
  case 0x3B: case 0x3A: return 0x33; // Keyboard ; and :
  case 0x27: case 0x22: return 0x34; // Keyboard ' and "
  case 0x60: case 0x7E: return 0x35; // Keyboard Grave Accent and Tilde
  case 0x2C: case 0x3C: return 0x36; // Keyboard, and <
  case 0x2E: case 0x3E: return 0x37; // Keyboard . and >
  case 0x2F: case 0x3F: return 0x38; // Keyboard / and ?
  // case unknown: return 0x39; // Keyboard Caps Lock
  case 0xFFBEu: return 0x3A; // Keyboard F1
  case 0xFFBFu: return 0x3B; // Keyboard F2
  case 0xFFC0u: return 0x3C; // Keyboard F3
  case 0xFFC1u: return 0x3D; // Keyboard F4
  case 0xFFC2u: return 0x3E; // Keyboard F5
  case 0xFFC3u: return 0x3F; // Keyboard F6
  case 0xFFC4u: return 0x40; // Keyboard F7
  case 0xFFC5u: return 0x41; // Keyboard F8
  case 0xFFC6u: return 0x42; // Keyboard F9
  case 0xFFC7u: return 0x43; // Keyboard F10
  case 0xFFC8u: return 0x44; // Keyboard F11
  case 0xFFC9u: return 0x45; // Keyboard F12
  case 0xFFCAu: return 0x46; // Keyboard PrintScreen
  // case unknown: return 0x47; // Keyboard Scroll Lock
  case 0xFF02u: return 0x48; // Keyboard Pause
  case 0xFF6Au: return 0x49; // Keyboard Insert
  case 0xFF50u: return 0x4A; // Keyboard Home
  case 0xFF55u: return 0x4B; // Keyboard PageUp
  case 0xFFFFu: return 0x4C; // Keyboard Delete Forward
  case 0xFF57u: return 0x4D; // Keyboard End
  case 0xFF56u: return 0x4E; // Keyboard PageDown
  case 0xFF53u: return 0x4F; // Keyboard RightArrow
  case 0xFF51u: return 0x50; // Keyboard LeftArrow
  case 0xFF54u: return 0x51; // Keyboard DownArrow
  case 0xFF52u: return 0x52; // Keyboard UpArrow
  // case unknown: return 0x53; // Keypad Num Lock and Clear
  case 0xFFAFu: return 0x54; // Keypad /
  case 0xFFAAu: return 0x55; // Keypad *
  case 0xFFADu: return 0x56; // Keypad -
  case 0xFFABu: return 0x57; // Keypad +
  case 0xFF8Du: return 0x58; // Keypad ENTER
  case 0xFFB1u: return 0x59; // Keypad 1 and End
  case 0xFFB2u: return 0x5A; // Keypad 2 and Down Arrow
  case 0xFFB3u: return 0x5B; // Keypad 3 and PageDn
  case 0xFFB4u: return 0x5C; // Keypad 4 and Left Arrow
  case 0xFFB5u: return 0x5D; // Keypad 5
  case 0xFFB6u: return 0x5E; // Keypad 6 and Right Arrow
  case 0xFFB7u: return 0x5F; // Keypad 7 and Home
  case 0xFFB8u: return 0x60; // Keypad 8 and Up Arrow
  case 0xFFB9u: return 0x61; // Keypad 9 and PageUp
  case 0xFFB0u: return 0x62; // Keypad 0 and Insert
  case 0xFFAEu: return 0x63; // Keypad . and Delete
  // case unknown: return 0x64; // Keyboard Non-US \ and |
  // case unknown: return 0x65; // Keyboard Application
  // case unknown: return 0x66; // Keyboard Power
  // case unknown: return 0x67; // Keypad =
  // case unknown: return 0x68; // Keyboard F13
  // case unknown: return 0x69; // Keyboard F14
  // case unknown: return 0x6A; // Keyboard F15
  // case unknown: return 0x6B; // Keyboard F16
  // case unknown: return 0x6C; // Keyboard F17
  // case unknown: return 0x6D; // Keyboard F18
  // case unknown: return 0x6E; // Keyboard F19
  // case unknown: return 0x6F; // Keyboard F20
  // case unknown: return 0x70; // Keyboard F21
  // case unknown: return 0x71; // Keyboard F22
  // case unknown: return 0x72; // Keyboard F23
  // case unknown: return 0x73; // Keyboard F24
  // case unknown: return 0x74; // Keyboard Execute
  // case unknown: return 0x75; // Keyboard Help
  // case unknown: return 0x76; // Keyboard Menu
  // case unknown: return 0x77; // Keyboard Select
  // case unknown: return 0x78; // Keyboard Stop
  // case unknown: return 0x79; // Keyboard Again
  // case unknown: return 0x7A; // Keyboard Undo
  // case unknown: return 0x7B; // Keyboard Cut
  // case unknown: return 0x7C; // Keyboard Copy
  // case unknown: return 0x7D; // Keyboard Paste
  // case unknown: return 0x7E; // Keyboard Find
  // case unknown: return 0x7F; // Keyboard Mute
  // case unknown: return 0x80; // Keyboard Volume Up
  // case unknown: return 0x81; // Keyboard Volume Down
  // case unknown: return 0x82; // Keyboard Locking Caps Lock
  // case unknown: return 0x83; // Keyboard Locking Num Lock
  // case unknown: return 0x84; // Keyboard Locking Scroll Lock
  // case unknown: return 0x85; // Keypad Comma
  // case unknown: return 0x86; // Keypad Equal Sign
  // case unknown: return 0x87; // Keyboard International1
  // case unknown: return 0x88; // Keyboard International2
  // case unknown: return 0x89; // Keyboard International3
  // case unknown: return 0x8A; // Keyboard International4
  // case unknown: return 0x8B; // Keyboard International5
  // case unknown: return 0x8C; // Keyboard International6
  // case unknown: return 0x8D; // Keyboard International7
  // case unknown: return 0x8E; // Keyboard International8
  // case unknown: return 0x8F; // Keyboard International9
  // case unknown: return 0x90; // Keyboard LANG1
  // case unknown: return 0x91; // Keyboard LANG2
  // case unknown: return 0x92; // Keyboard LANG3
  // case unknown: return 0x93; // Keyboard LANG4
  // case unknown: return 0x94; // Keyboard LANG5
  // case unknown: return 0x95; // Keyboard LANG6
  // case unknown: return 0x96; // Keyboard LANG7
  // case unknown: return 0x97; // Keyboard LANG8
  // case unknown: return 0x98; // Keyboard LANG9
  // case unknown: return 0x99; // Keyboard Alternate Erase
  // case unknown: return 0x9A; // Keyboard SysReq/Attention
  // case unknown: return 0x9B; // Keyboard Cancel
  // case unknown: return 0x9C; // Keyboard Clear
  // case unknown: return 0x9D; // Keyboard Prior
  // case unknown: return 0x9E; // Keyboard Return
  // case unknown: return 0x9F; // Keyboard Separator
  // case unknown: return 0xA0; // Keyboard Out
  // case unknown: return 0xA1; // Keyboard Oper
  // case unknown: return 0xA2; // Keyboard Clear/Again
  // case unknown: return 0xA3; // Keyboard CrSel/Props
  // case unknown: return 0xA4; // Keyboard ExSel
  // case 0xFFE3u: return 0xE0; // Keyboard LeftControl
  // case 0xFFE1u: return 0xE1; // Keyboard LeftShift
  // case 0xFE03u: return 0xE2; // Keyboard LeftAlt
  // case 0xFFE9u: return 0xE3; // Keyboard Left GUI
  // case 0xFFE4u: return 0xE4; // Keyboard RightControl
  // case 0xFFE2u: return 0xE5; // Keyboard RightShift
  // case 0xFF7Eu: return 0xE6; // Keyboard RightAlt
  // case 0xFFEBu: return 0xE7; // Keyboard Right GUI default:
  default: return 0;
  }
}

static bool ICACHE_FLASH_ATTR
emitKeyEvent(bool pressed, uint32 key) {
  static uint8_t keys[7];
  uint8_t newKeys[7];
  os_memcpy(newKeys, keys, sizeof(keys));

  uint8_t modifier = getModifier(key);
  if (modifier != 0) {
    if (pressed) {
      newKeys[0] |= modifier;
      if ((newKeys[0] & (1<<1 | 1<<5)) == (1<<1 | 1<<5)) // left-shift && right-shift
      // reset key state
        memset(newKeys, 0, sizeof(newKeys));
    } else
      newKeys[0] &= ~modifier;
  }

  uint8_t mapkey = mapKey(key);
  // DBG("KE: m: 0x%X, 0x%X => 0x%X, %s\n", modifier, key, mapkey, pressed ? "down" : "up");
  if (pressed) {
    bool found = false;
    for (int i = 1; i < 7; i++)
      if (newKeys[i] == mapkey)
        found = true;
    if (! found) {
      for (int i = 1; i < 7; i++)
        if (newKeys[i] == 0) {
          newKeys[i] = mapkey;
          break;
        }
    }
  } else {
    for (int i = 1; i < 7; i++)
      if (newKeys[i] == mapkey)
        newKeys[i] = 0;
  }

  if (memcmp(keys, newKeys, sizeof(keys)) != 0) {
    // FIXME: For performance, we only send 2 bytes, modifier and key
    // as a result, we cannot do n-key rollover
    if (tlv_send(TLV_HID, (char *) newKeys, 2) != 0)
      return false;
    os_memcpy(keys, newKeys, sizeof(keys));
  }
  return true;
}

static int8_t pointer_event[] = {0, -1, -1, 0};

static bool ICACHE_FLASH_ATTR
emitPointerEvent(int mask, int x, int y, int z) {
  static int old_x = -1, old_y = -1;
  
  if (old_x != -1) {
    
    pointer_event[0] = mask;
    pointer_event[1] = x - old_x;
    pointer_event[2] = y - old_y;
    pointer_event[3] = z;
    if (tlv_send(TLV_HID, (char *) pointer_event, 4) != 0)
      return false;
  }
  old_x = x;
  old_y = y;
  return true;
}

int8 ICACHE_FLASH_ATTR
vnc_proto_handler(vncbridgeConnData *conn) {
  while (conn->rxbufferlen > 0) {
    uint16 consume = 0;
    switch (conn->state) {
    case CLIENT_HELLO:
      if (conn->rxbufferlen < 12)
        return true; // need more input
      // DBG("Received client hello\n");

      espbuffsend_static(conn, AUTH_CHALLENGE, sizeof(AUTH_CHALLENGE));
      // DBG("Sent auth challenge\n");
      conn->state = CLIENT_AUTH;
      consume = 12;
      break;
    case CLIENT_AUTH:
      if (conn->rxbufferlen < 16)
        return true; // need more input
      // DBG("Received Client_auth\n");
      bool authSuccessful = true;
      for (int i = 0; i < 16; i++) {
        if (conn->rxbuffer[i] != AUTH_RESPONSE[i]) {
          authSuccessful = false;
        }
      }
      // DBG("Client auth %d\n", authSuccessful);
      
      consume = 16;
      if (authSuccessful) {
        // DBG("Client auth successful\n");
        espbuffsend_static(conn, AUTH_OK, sizeof(AUTH_OK)); // authOK
        conn->state = CLIENT_INIT;
        break;
      } else {
        espbuffsend_static(conn, AUTH_FAILED, sizeof(AUTH_FAILED));
        return false;
      }
    case CLIENT_INIT:
      // DBG("Client init, sending INIT_MESSAGE\n");
      consume = 1;
      espbuffsend_static(conn, INIT_MESSAGE, sizeof(INIT_MESSAGE));
      conn->state = RFB_MESSAGE;
      break;
    case RFB_MESSAGE:
      switch (conn->rxbuffer[0]) {
        case SetPixelFormat:
          // DBG("SetPixelFormat\r\n");
          if (conn->rxbufferlen < 20)
            return true;
          // discard the request
          consume = 20;
          break;
       case FixColourMapEntries:
          // DBG("FixColorMapEntries\n");
          if (conn->rxbufferlen < 6)
            return true;
          int entries = conn->rxbuffer[4] << 8 | conn->rxbuffer[5];
          if (conn->rxbufferlen < 6 + entries * 6)
            return true;
          consume = 6 + 6 * entries;
          break;
        case SetEncodings:
          // DBG("SetEncodings\n");
          if (conn->rxbufferlen < 4)
            return true;
          int nCodings = conn->rxbuffer[2] << 8 | conn->rxbuffer[3];
          if (conn->rxbufferlen < 4 + nCodings * 4)
            return true;
          consume = 4 + nCodings * 4;
          break;
        case FrameBufferUpdateRequest:
          // DBG("FramebufferUpdateRequest\n");
          if (conn->rxbufferlen < 10)
            return true;
          consume = 10;
          // we don't respond to these
          break;
        case KeyEvent:
          // DBG("KeyEvent\n");
          if (conn->rxbufferlen < 8)
            return true;
          consume = 8;
          bool pressed = conn->rxbuffer[1] == 1;
          uint32 key = ((uint32) (conn->rxbuffer[4] << 24)) | (conn->rxbuffer[5] << 16) | (conn->rxbuffer[6] << 8) | (conn->rxbuffer[7]);
          if (!emitKeyEvent(pressed, key))
            return true;
          break;
        case PointerEvent:
          // DBG("PointerEvent\n");
          if (conn->rxbufferlen < 6)
            return true;
          consume = 6;
          uint8 mask = conn->rxbuffer[1];
          int32 x = conn->rxbuffer[2] << 8 | conn->rxbuffer[3];
          int32 y = conn->rxbuffer[4] << 8 | conn->rxbuffer[5];
          if (!emitPointerEvent(mask, x, y, 0))
            return true;
          break;
        case ClientCutText:
          if (conn->rxbufferlen < 8)
            return true;
          consume = 8;
          conn->cut_text = (conn->rxbuffer[4] << 24) | (conn->rxbuffer[5] << 16) | (conn->rxbuffer[6] | 8) | (conn->rxbuffer[7]);
          conn->state = CUT_TEXT;
          break;
        }
        break;
      case CUT_TEXT:
        consume = conn->cut_text;
        if (consume > conn->rxbufferlen)
          consume = conn->rxbufferlen;
        conn->cut_text -= consume;
        if (conn->cut_text == 0)
          conn->state = RFB_MESSAGE;
        break;
    default:
      return false;
    }
    if (consume > 0) {
      os_memcpy(conn->rxbuffer, conn->rxbuffer + consume, conn->rxbufferlen - consume);
      conn->rxbufferlen -= consume;
      // DBG("consumed 0x%X, 0x%X remain\n", consume, conn->rxbufferlen);
    }
  }
  return true;
}

static void ICACHE_FLASH_ATTR
vncProcessRX(vncbridgeConnData *conn) {
  if (!vnc_proto_handler(conn)) {
    // discard any pending data
    conn->rxbufferlen = 0;
    //close connection
    DBG("proto says to close connection!\n");
    espconn_disconnect(conn->conn);
  }
}

//===== Initialization

void ICACHE_FLASH_ATTR
vncbridgeInitPins()
{

  PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, 0);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, 0);
  PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
  PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0RXD_U);
  system_uart_de_swap();

}

int8_t ICACHE_FLASH_ATTR
vncTlvCb(tlv_data_t *tlv_data) {

  if (tlv_data != NULL) {
    // do something with it
  }

  return 0;
}

static void ICACHE_FLASH_ATTR
deferredTask(os_event_t *events)
{
  bool more = false;
  // look for any residual data in the rx buffers
  uint8_t c;
  for (c=0; c<VNC_MAX_CONN; c++) {
    DBG_RATE(1000*1000, "Task %d : %d rx bytes\n", c, vncConnData[c].rxbufferlen);
    if (vncConnData[c].rxbufferlen > 0) {
      vncProcessRX(&vncConnData[c]);
      DBG_RATE(1000*1000, "Heap: %ld\n", (unsigned long) system_get_free_heap_size());
    }
    if (vncConnData[c].conn != NULL && vncConnData[c].rxbufferlen < 32) {
      espconn_recv_unhold(vncConnData[c].conn);
    } else if (vncConnData[c].conn == NULL && vncConnData[c].rxbuffer != NULL && vncConnData[c].rxbufferlen == 0) {
      DBG("Freed RX buffer\n");
      os_free(vncConnData[c].rxbuffer);
      DBG("VncDefr: RX at %p\n", vncConnData[c].rxbuffer);
      vncConnData[c].rxbuffer = NULL;
      vncConnData[c].rxbufferlen = 0;
    }
    if (vncConnData[c].rxbufferlen > 0)
      more = true;
  }
  if (more) // data awaits, schedule it ourselves
    post_usr_task(deferredTaskNum, 0);
}

// Start vnc bridge TCP server on specified port (typ. 5900)
void ICACHE_FLASH_ATTR
vncbridgeInit(int port)
{
  vncbridgeInitPins();

  os_memset(vncConnData, 0, sizeof(vncConnData));
  os_memset(&vncbridgeTcp, 0, sizeof(vncbridgeTcp));

  // set-up the primary port for plain bridging
  vncbridgeConn.type = ESPCONN_TCP;
  vncbridgeConn.state = ESPCONN_NONE;
  vncbridgeTcp.local_port = port;
  vncbridgeConn.proto.tcp = &vncbridgeTcp;

  espconn_regist_connectcb(&vncbridgeConn, vncbridgeConnectCb);
  espconn_accept(&vncbridgeConn);
  espconn_tcp_set_max_con_allow(&vncbridgeConn, VNC_MAX_CONN);
  espconn_regist_time(&vncbridgeConn, VNC_BRIDGE_TIMEOUT, 0);

  tlv_register_channel_handler(TLV_HID, vncTlvCb);

  deferredTaskNum = register_usr_task(deferredTask);
}
