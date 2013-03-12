#include <rtthread.h>
#include <rthw.h>

#include "board.h"
#include "bt_app.h"
#include "protocol_bt.h"
#define	printk	rt_kprintf

rt_uint8_t btp_h_valid(rt_uint8_t ch)
{
	if( (ch == BTP_DeviceInfo) ||
		(ch == BTP_DeviceManagement) ||
		(ch == BTP_AppCommand) ||
		(ch == BTP_AppInfo) ||
		(ch == BTP_AppCommand) ) return 1;
	else return 0;
}

int checkbtp_header(BTP_HEADER *p)
{
	if( btp_h_valid(p->start)){ /* valid start code */

		if( (p->rw != BTP_READ) && (p->rw != BTP_WRITE) )
			return -2;
		//p->cmd !=
		//p->idx
		if( p->len > (MAX_BTP_LEN - BTP_HEAD_SIZE -1) ){
			printk("btp lenth %d over %d\n", p->len, (255 - BTP_HEAD_SIZE -1));
			return -3;
		}
		return 1;
	}
	return -1;

}
