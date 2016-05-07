#include "tlv.h"
#include "config.h"
#include "syslog.h"
#include <uart.h>

uint8_t ICACHE_FLASH_ATTR tlv_send(uint8_t channel, char *buf, uint8_t len)
{
  static char copy_buf[TLV_MAX_PACKET + 2];

  copy_buf[0] = len + 1;
  copy_buf[1] = channel;
  memcpy(copy_buf+2, buf, len);

  syslog(SYSLOG_FAC_USER, SYSLOG_PRIO_NOTICE, "esp-link", "Sending packet, channel %d, length %d\n", channel, len);
  if (len == 4) {
    syslog(SYSLOG_FAC_USER, SYSLOG_PRIO_NOTICE, "esp-link", "Mouse %x %d %d %d\n", (uint8_t) buf[0], (uint8_t) buf[1], (uint8_t) buf[2], (uint8_t) buf[3]);
  }
    
  uart0_tx_buffer(copy_buf, len+2);
  return 0;
}

static char tlv_buf[TLV_MAX_PACKET];
static uint8_t tlv_read = 0;
static uint8_t tlv_outstanding = 0;

static tlv_receive_cb tlv_cb[TLV_MAX_HANDLERS];

// callback with a buffer of characters that have arrived on the uart
void ICACHE_FLASH_ATTR
tlvUartCb(char *buf, short length) {
  while (length > 0) {
    if (tlv_outstanding == 0) {

      tlv_outstanding = (uint8_t) buf[0];
      tlv_read = 0;
      syslog(SYSLOG_FAC_USER, SYSLOG_PRIO_NOTICE, "esp-link", "New packet, length %d\n", tlv_outstanding);

    } else {

      #define min(X, Y) (((X)<(Y))?(X):(Y))
      uint8_t read = min(tlv_outstanding, length);
      #undef min

      memcpy(tlv_buf + tlv_read, buf + tlv_read, read);
      tlv_read += read;
      tlv_outstanding -= read;

      if (tlv_outstanding == 0) {
        uint8_t channel = (uint8_t) tlv_buf[0];
        syslog(SYSLOG_FAC_USER, SYSLOG_PRIO_NOTICE, "esp-link", "Complete packet read, length %d, channel %d\n", tlv_read, channel);
        tlv_receive_cb cb;
        if (channel < TLV_MAX_HANDLERS) {
          cb = tlv_cb[channel];
        } else {
          cb = tlv_cb[0];
        }
        if (cb != NULL) {
          cb(channel, tlv_buf + 1, tlv_read - 1);
        } else {
          syslog(SYSLOG_FAC_USER, SYSLOG_PRIO_NOTICE, "esp-link", "Received message for unregistered channel %d\n", channel);
        }
      }
    }
  }
}

void ICACHE_FLASH_ATTR tlv_register_channel_handler(uint8_t channel, tlv_receive_cb cb) {
  if (channel < TLV_MAX_HANDLERS) {
    tlv_cb[channel] = cb;
  }
}

