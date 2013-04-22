#include <rtthread.h>
#include <rthw.h>

#include "board.h"
#include "bt_app.h"
#include "bt_parser.h"
#include "led_pwm.h"

#define	printk	rt_kprintf

extern struct _led_dev *leds;

static rt_uint8_t is_head(void)
{
	rt_uint8_t ch=0;

	while(1){
		while( ch != HEAD_SIG1 ) read_bt(1, &ch);
		read_bt(1, &ch);
		if(ch == HEAD_SIG2) break;
	}

	return 0;
}

/*
 * 0xC1 0x27 len led# toks repeat (token0 token1 token2 ...) checksum
 */
rt_int8_t btcmd_paser(void)
{
	rt_uint8_t chksum=0, len, i, n;
	rt_uint8_t cmd[MAX_CMD_LEN];
	union _led_cmdq *pcmdq;

	if(is_head()){
		read_bt(1, &len); /* length of the command */
		read_bt(len<MAX_CMD_LEN?len:MAX_CMD_LEN, cmd);
		chksum += HEAD_SIG1 + HEAD_SIG2;
		chksum += len;
		for( i = 0 ; i < (len - 1) ; i ++) chksum += cmd[i];
		if(chksum==cmd[i]){ /* checksum is ok */
			n = cmd[0]; /* led number */
			pcmdq = &(leds[n].cmdq);
			rt_memcpy(pcmdq->bt_cmd, cmd, len-1);
			set_hw_led(n);
		}else{
			printk("checksum error 0x%x, it should be [0x%x]!\n", chksum, cmd[i]);
			return -1;
		}
	} else return -1;
	return 0;
}

