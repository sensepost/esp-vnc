#ifndef __TLV_H__
#define __TLV_H__

#include <esp8266.h>

#define TLV_MAX_HANDLERS 4
#define TLV_MAX_PACKET 64

void ICACHE_FLASH_ATTR
tlvUartCb(char *buf, short length);

/** Callback used when registering the handler for incoming data
 *  The channel number is a 1-based channel sequence
 *  Channel 0 is the default handler for any channels greater than TLV_MAX_CHANNELS
 *  It should return a true or false to indicate whether the message could be consumed or not
 *  this is intended to be used as a sort of flow control mechanism, but needs to be properly thought through!
 */
typedef bool (*tlv_receive_cb)(uint8_t channel, char *buf, uint8_t len);

uint8_t ICACHE_FLASH_ATTR tlv_send(uint8_t channel, char *buf, uint8_t len);
void ICACHE_FLASH_ATTR tlv_register_channel_handler(uint8_t channel, tlv_receive_cb cb);

#endif /* __TLV_H__ */
