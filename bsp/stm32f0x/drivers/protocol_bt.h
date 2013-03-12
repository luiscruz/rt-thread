/* protocol_bt.h
 * thomas tsai	thomas@life100.cc

 */
#ifndef	_H_PROTOCOL_BT_H
#define	_H_PROTOCOL_BT_H

#define	BTP_DeviceInfo			0xC1
#define	BTP_DeviceManagement	0xC2
#define	BTP_DeviceAppCmd		0xC3
#define	BTP_AppInfo				0xD1
#define	BTP_AppCommand			0xD3

#define	BTP_READ				0x26
#define	BTP_WRITE				0x27
#define	MAX_BTP_LEN				47

typedef struct _btp_header{
rt_uint8_t	start;
rt_uint8_t	rw;
rt_uint16_t	cmd;
rt_uint16_t idx;
rt_uint8_t len;
} BTP_HEADER;
#define	BTP_HEAD_SIZE	(sizeof(BTP_HEADER))

rt_uint8_t btp_h_valid(rt_uint8_t ch);
int checkbtp_header(BTP_HEADER *p);

#endif
