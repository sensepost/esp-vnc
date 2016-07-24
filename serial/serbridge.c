// Copyright 2015 by Thorsten von Eicken, see LICENSE.txt

#include "esp8266.h"

#include "uart.h"
#include "crc16.h"
#include "serbridge.h"
#include "serled.h"
#include "config.h"
#include "console.h"
#include "slip.h"
#include "cmd.h"
#ifdef SYSLOG
#include "syslog.h"
#else
#define syslog(a,...)
#endif
#include "tlv.h"
#include "task.h"

// #define SERBR_DBG
#ifdef SERBR_DBG
#define DBG(format, ...) do { os_printf(format, ## __VA_ARGS__); } while(0)
#else
#define DBG(format, ...) do { } while(0)
#endif

#define SKIP_AT_RESET

LOCAL uint8_t deferredTaskNum;

static struct espconn serbridgeConn1; // plain bridging port
static esp_tcp serbridgeTcp1;

// Connection pool
serbridgeConnData connData[MAX_CONN];

static void ICACHE_FLASH_ATTR
serbridgeProcessRX(serbridgeConnData *conn)
{

  if (conn->rxbuffer != NULL && conn->rxbufferlen > 0) {
    uint8_t len = conn->rxbufferlen > TLV_MAX_PACKET ? TLV_MAX_PACKET : conn->rxbufferlen;
    if (tlv_send(TLV_PIPE, conn->rxbuffer, len) == 0) {
      os_memcpy(conn->rxbuffer, conn->rxbuffer+len, conn->rxbufferlen-len);
      conn->rxbufferlen -= len;
    }
  }

}

static void ICACHE_FLASH_ATTR
deferredTask(os_event_t *events)
{
  bool more = false;
  // look for any residual data in the rx buffers
  uint8_t c;
  for (c=0; c<MAX_CONN; c++) {
    // os_printf("Task %d : %d rx bytes\n", c, connData[c].rxbufferlen);
    if (connData[c].rxbufferlen > 0) {
      serbridgeProcessRX(&connData[c]);
    }
    if (connData[c].conn != NULL && connData[c].rxbufferlen < 32) {
      espconn_recv_unhold(connData[c].conn);
    } else if (connData[c].conn == NULL && connData[c].rxbuffer != NULL && connData[c].rxbufferlen == 0) {
      DBG("Freed RX buffer\n");
      os_free(connData[c].rxbuffer);
      DBG("Defr: RX at %p\n", connData[c].rxbuffer);
      connData[c].rxbuffer = NULL;
      connData[c].rxbufferlen = 0;
    }
    if (connData[c].rxbufferlen > 0)
      more = true;
  }
  if (more) // data awaits, schedule it ourselves
    post_usr_task(deferredTaskNum, 0);
}

// Receive callback
static void ICACHE_FLASH_ATTR
serbridgeRecvCb(void *arg, char *data, unsigned short len)
{
  serbridgeConnData *conn = ((struct espconn*)arg)->reverse;
  if (conn == NULL) return;
  if (conn->rxbufferlen + len > MAX_RXBUFFER) {
    os_printf("RX buffer overrun!\n");
    espconn_disconnect(conn->conn);
    return;
  }
  sint8_t res = espconn_recv_hold(conn->conn);
  if (res != 0) os_printf("Hold %d\n", res);
  if (conn->rxbuffer != NULL) {
    os_memcpy(conn->rxbuffer + conn->rxbufferlen, data, len);
  } else {
    DBG("Serial receive buffer is NULL!\n");
  }
  conn->rxbufferlen += len;
  os_printf("RX Buffer now %d\n", conn->rxbufferlen);
  post_usr_task(deferredTaskNum, 0);
}

//===== UART -> TCP

// Send all data in conn->txbuffer
// returns result from espconn_sent if data in buffer or ESPCONN_OK (0)
// Use only internally from espbuffsend and serbridgeSentCb
static sint8 ICACHE_FLASH_ATTR
sendtxbuffer(serbridgeConnData *conn)
{
  sint8 result = ESPCONN_OK;
  if (conn->txbufferlen != 0) {
    //os_printf("TX %p %d\n", conn, conn->txbufferlen);
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
espbuffsend(serbridgeConnData *conn, const char *data, uint16 len)
{
  if (conn->txbufferlen >= MAX_TXBUFFER) goto overflow;

  // make sure we indeed have a buffer
  if (conn->txbuffer == NULL) conn->txbuffer = os_zalloc(MAX_TXBUFFER);
  if (conn->txbuffer == NULL) {
    os_printf("espbuffsend: cannot alloc tx buffer\n");
    return -128;
  }

  // add to send buffer
  uint16_t avail = conn->txbufferlen+len > MAX_TXBUFFER ? MAX_TXBUFFER-conn->txbufferlen : len;
  os_memcpy(conn->txbuffer + conn->txbufferlen, data, avail);
  conn->txbufferlen += avail;

  // try to send
  sint8 result = ESPCONN_OK;
  if (conn->readytosend) result = sendtxbuffer(conn);

  if (avail < len) {
    // some data didn't fit into the buffer
    if (conn->txbufferlen == 0) {
      // we sent the prior buffer, so try again
      return espbuffsend(conn, data+avail, len-avail);
    }
    goto overflow;
  }
  return result;

overflow:
  if (conn->txoverflow_at) {
    // we've already been overflowing
    if (system_get_time() - conn->txoverflow_at > 10*1000*1000) {
      // no progress in 10 seconds, kill the connection
      os_printf("serbridge: killing overlowing stuck conn %p\n", conn);
      espconn_disconnect(conn->conn);
    }
    // else be silent, we already printed an error
  } else {
    // print 1-time message and take timestamp
    os_printf("serbridge: txbuffer full, conn %p\n", conn);
    conn->txoverflow_at = system_get_time();
  }
  return -128;
}

//callback after the data are sent
static void ICACHE_FLASH_ATTR
serbridgeSentCb(void *arg)
{
  serbridgeConnData *conn = ((struct espconn*)arg)->reverse;
  //os_printf("Sent CB %p\n", conn);
  if (conn == NULL) return;
  //os_printf("%d ST\n", system_get_time());
  if (conn->sentbuffer != NULL) os_free(conn->sentbuffer);
  conn->sentbuffer = NULL;
  conn->readytosend = true;
  conn->txoverflow_at = 0;
  sendtxbuffer(conn); // send possible new data in txbuffer
}

void ICACHE_FLASH_ATTR
console_process(char *buf, short len)
{
  // push buffer into web-console
  for (short i=0; i<len; i++)
    console_write_char(buf[i]);
  // push the buffer into each open connection
  for (short i=0; i<MAX_CONN; i++) {
    if (connData[i].conn) {
      espbuffsend(&connData[i], buf, len);
    }
  }
}

//===== Connect / disconnect

// Disconnection callback
static void ICACHE_FLASH_ATTR
serbridgeDisconCb(void *arg)
{
  serbridgeConnData *conn = ((struct espconn*)arg)->reverse;
  if (conn == NULL) return;

  // Free buffers
  if (conn->sentbuffer != NULL) os_free(conn->sentbuffer);
  conn->sentbuffer = NULL;

  if (conn->txbuffer != NULL) os_free(conn->txbuffer);
  conn->txbuffer = NULL;
  conn->txbufferlen = 0;

  if (conn->rxbuffer != NULL) os_free(conn->rxbuffer);
  DBG("SerDisc: RX at %p\n", conn->rxbuffer);
  // conn->rxbuffer = NULL;
  // conn->rxbufferlen = 0;

  conn->conn = NULL;
  DBG("SER connection closed, all buffers freed\n");
}

// Connection reset callback (note that there will be no DisconCb)
static void ICACHE_FLASH_ATTR
serbridgeResetCb(void *arg, sint8 err)
{
  os_printf("serbridge: connection reset err=%d\n", err);
  serbridgeDisconCb(arg);
}

// New connection callback, use one of the connection descriptors, if we have one left.
static void ICACHE_FLASH_ATTR
serbridgeConnectCb(void *arg)
{
  struct espconn *conn = arg;
  // Find empty conndata in pool
  int i;
  for (i=0; i<MAX_CONN; i++) if (connData[i].conn==NULL) break;
#ifdef SERBR_DBG
  os_printf("Accept port %d, conn=%p, pool slot %d\n", conn->proto.tcp->local_port, conn, i);
#endif
  syslog(SYSLOG_FAC_USER, SYSLOG_PRIO_NOTICE, "esp-link", "Accept port %d, conn=%p, pool slot %d\n", conn->proto.tcp->local_port, conn, i);
  if (i==MAX_CONN) {
#ifdef SERBR_DBG
    os_printf("Aiee, conn pool overflow!\n");
#endif
        syslog(SYSLOG_FAC_USER, SYSLOG_PRIO_WARNING, "esp-link", "Aiee, conn pool overflow!\n");
    espconn_disconnect(conn);
    return;
  }

  os_memset(connData+i, 0, sizeof(struct serbridgeConnData));
  connData[i].conn = conn;
  conn->reverse = connData+i;
  connData[i].readytosend = true;

  // allocate the rx buffer
  connData[i].rxbuffer = os_zalloc(MAX_RXBUFFER);
  connData[i].rxbufferlen = 0;
  if (connData[i].rxbuffer == NULL) {
    os_printf("Out of memory for RX buffer\n");
    espconn_disconnect(conn);
    return;
  } else {
    DBG("SerConn: RX at %p\n", connData[i].rxbuffer);
  }

  espconn_regist_recvcb(conn, serbridgeRecvCb);
  espconn_regist_disconcb(conn, serbridgeDisconCb);
  espconn_regist_reconcb(conn, serbridgeResetCb);
  espconn_regist_sentcb(conn, serbridgeSentCb);

  espconn_set_opt(conn, ESPCONN_REUSEADDR|ESPCONN_NODELAY);
}

int8_t ICACHE_FLASH_ATTR
serTlvCb(tlv_data_t *tlv_data) {
  if (tlv_data != NULL) {
    switch(tlv_data->channel) {
    case TLV_PIPE:
      // log them to the console
      for (short i=0; i<tlv_data->length; i++)
        console_write_char(tlv_data->data[i]);

      for (short i=0; i<MAX_CONN; i++) {
        if (connData[i].conn != NULL) {
          DBG("Connection %d gets them!\n", i);
          espbuffsend(&connData[i], (char *) tlv_data->data, tlv_data->length);
        }
      }
      break;
    case TLV_DEBUG:
      for (short i=0; i<tlv_data->length; i++)
        DBG("%c", tlv_data->data[i]);
      break;
    default:
      break;
    }
  }

  return 0;
}

// Start transparent serial bridge TCP server on specified port (typ. 23)
void ICACHE_FLASH_ATTR
serbridgeInit(int port)
{
  os_memset(connData, 0, sizeof(connData));
  os_memset(&serbridgeTcp1, 0, sizeof(serbridgeTcp1));

  // set-up the primary port for plain bridging
  serbridgeConn1.type = ESPCONN_TCP;
  serbridgeConn1.state = ESPCONN_NONE;
  serbridgeTcp1.local_port = port;
  serbridgeConn1.proto.tcp = &serbridgeTcp1;

  espconn_regist_connectcb(&serbridgeConn1, serbridgeConnectCb);
  espconn_accept(&serbridgeConn1);
  espconn_tcp_set_max_con_allow(&serbridgeConn1, MAX_CONN);
  espconn_regist_time(&serbridgeConn1, SER_BRIDGE_TIMEOUT, 0);

  tlv_register_channel_handler(TLV_PIPE, serTlvCb);
  tlv_register_channel_handler(TLV_DEBUG, serTlvCb);
  tlv_register_channel_handler(TLV_CONTROL, serTlvCb);

  deferredTaskNum = register_usr_task(deferredTask);
}
