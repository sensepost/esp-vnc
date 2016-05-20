#ifndef __TLV_H__
#define __TLV_H__

#include <esp8266.h>

#define TLV_MAX_HANDLERS 4
#define TLV_MAX_PACKET 64

#define TLV_CONTROL 0
#define TLV_HID 1
#define TLV_GENERIC 2

#define TLV_RESUME_FLOW 0

typedef struct {
	uint8_t channel;
	uint8_t length;
	uint8_t data[TLV_MAX_PACKET];
} tlv_data_t;

void ICACHE_FLASH_ATTR
tlvUartCb(char *buf, short length);

/** Callback used when registering the handler for incoming data
 *  The channel number is a 1-based channel sequence
 *  Channel 0 is the default handler for any channels greater than TLV_MAX_CHANNELS
 *  It should return a true or false to indicate whether the message could be consumed or not
 *  this is intended to be used as a sort of flow control mechanism, but needs to be properly thought through!
 */
typedef int8_t (*tlv_receive_cb)(tlv_data_t *tlv_data);

/**
 * Returns 0 if successful, non-zero if not
 * -1 indicates that the uart has been blocked
 * -2 indicates that the channel has been blocked
 *
 * In the event of a blocked transmission, action should be taken to 
 * send the tlv at a later time, typically by posting a task using post_user_task
 **/
int8_t ICACHE_FLASH_ATTR tlv_send(uint8_t channel, char *buf, uint8_t len);
void ICACHE_FLASH_ATTR tlv_register_channel_handler(uint8_t channel, tlv_receive_cb cb);

bool tlv_is_send_paused(void);

#endif /* __TLV_H__ */
