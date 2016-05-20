#include "tlv.h"
#include "config.h"
#ifdef SYSLOG
#include "syslog.h"
#else
#define syslog(a, ...) do {} while (0);
#endif
#include <uart.h>

#define TLV_DBG
#ifdef TLV_DBG
#define DBG(format, ...) do { os_printf(format, ## __VA_ARGS__); } while(0)
#define DBG_RATE(timeout, format, ...) do { static uint32_t t = 0; if (system_get_time() - t > (timeout)) { DBG(format, ## __VA_ARGS__); t = system_get_time(); } } while (0);
#else
#define DBG(format, ...) do { } while(0)
#define DBG_RATE(format, ...)
#endif

volatile static bool tlv_send_flow_paused = false;

void tlv_poll_uart(void);

static uint32_t lastUart = 0;

bool ICACHE_FLASH_ATTR
tlv_is_send_paused() {
  return tlv_send_flow_paused;
}

void ICACHE_FLASH_ATTR tlv_send_fc(bool enabled) {
  static char buf[] = { 0, 2, 0, 0 };

  buf[3] = enabled ? 1 : 0;
  uart0_tx_buffer(buf, 4);
}

int8_t ICACHE_FLASH_ATTR tlv_send(uint8_t channel, char *buf, uint8_t len)
{
  if (tlv_send_flow_paused) {
    DBG_RATE(10*1000*1000, "Flow control active while sending\n");
    if (system_get_time() - lastUart > 50*1000) { // 0.05s
      tlv_poll_uart();
    }
    return -1;
  }
  DBG_RATE(10*1000*1000, "Sending packet, channel %d, length %d\n", channel, len);

  uart0_write_char((char) channel);
  uart0_write_char((char) len);
  uart0_tx_buffer(buf, len);
  tlv_send_flow_paused = true;
  return 0;
}

static tlv_data_t tlv_data;
static uint8_t tlv_data_read = 0;

static enum {
	CHANNEL = 0, LENGTH = 1, DATA = 2
} tlv_read_state = CHANNEL;


static tlv_receive_cb tlv_cb[TLV_MAX_HANDLERS];

// callback with a buffer of characters that have arrived on the uart
void ICACHE_FLASH_ATTR
tlvUartCb(char *buf, short length) {
  lastUart = system_get_time();
  short pos = 0;
  uint8_t read;

  if (length < 4) { DBG("tlvUartCb: %d bytes\n", length); }
  while (pos < length) {
    switch (tlv_read_state) {
    case CHANNEL:
      tlv_data.channel = (uint8_t) buf[pos++];
      tlv_read_state = LENGTH;
      // DBG("New packet, channel %d\n", tlv_data.channel);
      break;
    case LENGTH:
      tlv_data.length = (uint8_t) buf[pos++];
      // DBG("New packet, channel %d, length %d\n", tlv_data.channel, tlv_data.length);
      tlv_data_read = 0;
      tlv_read_state = DATA;
      break;
    case DATA:
      #define MIN(X, Y) (((X)<(Y))?(X):(Y))
      read = MIN(tlv_data.length - tlv_data_read, length - pos);
      #undef MIN

      memcpy(tlv_data.data + tlv_data_read, buf + pos, read);
      pos += read;
      tlv_data_read += read;

      // DBG("Packet for channel %d, read %d of %d\n", tlv_data.channel, tlv_data_read, tlv_data.length);

      if (tlv_data_read == tlv_data.length) {
        // DBG("Complete packet read, channel %d, length %d at %d of %d\n", tlv_data.channel, tlv_data.length, pos, length);

        if (tlv_data.channel == 0) {
          if (tlv_data.length == 2 && tlv_data.data[0] == 0) {
            // DBG("TLV Flow control message: %d\n", tlv_data.data[1]);
            tlv_send_flow_paused = (tlv_data.data[1] != 0);
          }
        } else {
          tlv_receive_cb cb;
          if (tlv_data.channel < TLV_MAX_HANDLERS) {
            cb = tlv_cb[tlv_data.channel];
          } else {
            cb = tlv_cb[0];
          }
          if (cb != NULL) {
            cb(&tlv_data);
          } else {
            // DBG("Received message for unregistered channel %d\n", tlv_data.channel);
          }
        }
        tlv_read_state = CHANNEL;
      }
      break;
    }
  }
}

void ICACHE_FLASH_ATTR tlv_register_channel_handler(uint8_t channel, tlv_receive_cb cb) {
  if (channel < TLV_MAX_HANDLERS) {
    tlv_cb[channel] = cb;
  }
}

void ICACHE_FLASH_ATTR tlv_poll_uart() {
  char recv[1];

  uint16_t got;
  while ((got = uart0_rx_poll(recv, 1, 100)) == 1) {
    // feed the watchdog
    system_soft_wdt_feed();
    DBG("poll\n");
    tlvUartCb(recv, got);
  }
}
