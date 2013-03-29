/* protocol_bt.h
 * thomas tsai	thomas@life100.cc

 */
#ifndef	_H_PROTOCOL_BT_H
#define	_H_PROTOCOL_BT_H
#ifdef	LED_PWM_SUPPORTED
#include "led_pwm.h"
#else
#include "led.h"
#endif

#define	BTP_DeviceInfo			0xC1
#define	BTP_DeviceManagement	0xC2
#define	BTP_DeviceAppCmd		0xC3
#define	BTP_AppInfo				0xD1
#define	BTP_AppCommand			0xD3

#define	BTP_READ				0x26
#define	BTP_WRITE				0x27
#define	MAX_BTP_LEN				(64-7)

#define	BTP_RESP_OK				(0x00)
#define	BTP_RESP_NG				(0x01)
#define	BTP_RESET_NO_SUPPORT	(0xFF)

#pragma pack(1)
typedef struct _btp_header{
rt_uint8_t	start;
rt_uint8_t	rw;
rt_uint16_t	cmd;
rt_uint16_t idx;
rt_uint8_t len;
} BTP_HEADER;
#define	BTP_HEAD_SIZE	(sizeof(BTP_HEADER))

typedef struct _btp_resp{
rt_uint8_t	start;
rt_uint8_t	rw;
rt_uint16_t	cmd;
rt_uint16_t idx;
rt_uint8_t len;
rt_uint8_t err;
rt_uint8_t checksum;
} BTP_RESP;

struct btp_led_mode{
	rt_uint8_t led;
	struct led_mode mode;	/* repeat times, -1 : infinite */
};
#pragma pack()

int process_led_command(struct btapp_dev *dev);
//rt_uint8_t btp_h_valid(rt_uint8_t ch);
//int checkbtp_header(BTP_HEADER *p);
//void parsing_led_command(rt_uint8_t *buf, rt_uint8_t len);

#endif
