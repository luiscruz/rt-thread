#ifndef	_H_BT_PARSER_H
#define	_H_BT_PARSER_H

#define	HEAD_SIG1		(0x55)
#define	HEAD_SIG2		(0xAA)
#define	HEAD_RESP		(0xAB)
#define	HEAD_FINDME		(0xAC)

#define	MAX_CMD_LEN		(0x20)
#define	MAX_TOKENS		(5)

#pragma pack(1)
struct	_led_token{
	rt_int8_t	period; 	/* blinking period of the led */
					/* 1-127 : 1hz-127hz */
					/* -2 - -128 : 2s - 128s period */
	rt_int8_t	on;	/* percentage of on, so off is (100-on)% */
};

struct _led_blink{
	rt_uint8_t led;		/* led number */
	rt_uint8_t toks;	/* number of tokens in this command string */
	rt_int32_t rpts; 	/* repeating times of this command string */
	struct _led_token *seq;	/* token command sequence */
};

union _led_cmdq{
	rt_uint8_t bt_cmd[MAX_CMD_LEN];
	struct _led_blink blinks;
};

struct _resp_pkt{
	rt_uint16_t sig;
	rt_uint8_t len;
	rt_uint16_t rc;
	rt_uint8_t checksum;
};

#pragma pack()

rt_int8_t btcmd_paser(void);

#endif	//_H_BT_PARSER_H
