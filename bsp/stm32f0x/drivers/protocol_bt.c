#include <rtthread.h>
#include <rthw.h>

#include "board.h"
#include "bt_app.h"
#include "protocol_bt.h"
#define	printk	rt_kprintf

static rt_uint8_t btp_h_valid(rt_uint8_t ch)
{
	if( (ch == BTP_DeviceInfo) ||
		(ch == BTP_DeviceManagement) ||
		(ch == BTP_AppCommand) ||
		(ch == BTP_AppInfo) ||
		(ch == BTP_AppCommand) ) return 1;
	else return 0;
}

static int checkbtp_header(BTP_HEADER *p)
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

static void parsing_led_command(rt_uint8_t *buf, rt_uint8_t len)
{
	rt_uint8_t i;
	struct btp_led_mode led;
	printk("parsing_led_command\n");
	for(i = 0; i < len; i++){
		printk("0x%x ", buf[i]);
	}
	printk("\n");

	rt_memcpy(&led, buf, len);
#ifdef	LED_PWM_SUPPORTED
	set_hw_led(led.led, led.mode.period, led.mode.duty, led.mode.cnt);
#endif
}

/* scan a byte from bt or a finsh command line */
static rt_size_t scan_buffer(struct btapp_dev *dev,
                         void     *buffer,
                         rt_size_t   size)
{
#ifdef RT_USING_FINSH
	static rt_uint8_t sh_buf[80];
/*	if(check finsh commnd line first){
		return size bytes from finsh buffer;
		{
	else*/
#endif
	{
		rt_uint8_t *buf;
		rt_size_t s=0,r=0;
		int i;
		buf = (rt_uint8_t *)buffer;
		printk(">>>read %d bytes\n",size);
		do{
			/* wait receive */
			if (rt_sem_take(&(dev->rx_sem), RT_WAITING_FOREVER) != RT_EOK) continue;
			r = rt_device_read(dev->device, 0, buf, size-s);
			if(r != (size-s))
				printk("read err=0x%x\n",rt_get_errno());
			s+=r;
			for(i = 0; i < r; i ++)
				printk("0x%02x %c ", buf[i], buf[i]);
			printk("\n(%d, %d,%d)<<<<\n",r, s, size);
			if(s < size){
				buf = (rt_uint8_t *)buffer + s;
			}else break;
		}while(s<size);
		printk("<<<%d bytes read\n",s);
		return s;
	}
}

/*
packet :
0xC1 0x27 0x0002 0x0001 01 0xer 0xck
*/
static int write_led_resp(struct btapp_dev *dev, rt_uint8_t err)
{
	rt_uint8_t cks=0;
	rt_uint8_t *p;
	BTP_RESP btpr;
	int i;
	printk("write_led_resp %d\n", err);
	rt_memset(&btpr, 0, sizeof(BTP_RESP));
	btpr.start=BTP_DeviceInfo;
	btpr.rw=BTP_WRITE;
	btpr.cmd=0x0002;
	btpr.idx=0x0001;
	btpr.len=1;
	btpr.err = err;
	p = (rt_uint8_t *)&btpr;
	for(i = 0; i < sizeof(BTP_RESP) - 1; i++)
		cks += p[i];
	btpr.checksum = cks;
	for(i = 0; i < sizeof(BTP_RESP); i++){
		printk("0x%x ",p[i]);
	}
	printk("\n");
	/*Response command: C1 27 02 00 01 00 01 err checksum */
	rt_device_write(dev->device, 0, p, sizeof(BTP_RESP));
	return 0;
}

/*
 command packet
 0xC1 0x26 0x0002 0x0001 0xll 0x0l 0xpp 0xdd 0xcccccccc 0xck
*/
int process_led_command(struct btapp_dev *dev)
{
	char ch;
	if(dev == RT_NULL){
		printk("process_led_command 0x%x\n", dev);
		return -1;
	}
	/* read header from device */
	if( (scan_buffer(dev, &ch, 1) == 1) &&
		(btp_h_valid(ch))) { /* valid commands */
		BTP_HEADER btp_h;
		rt_uint8_t *ptr = (rt_uint8_t *)&btp_h;
		*ptr++ = ch;
		if(scan_buffer(dev, ptr, BTP_HEAD_SIZE -1 )
			== (BTP_HEAD_SIZE-1 ) ) {/* rest of the header*/
			//printk("header 1\n");
			if(checkbtp_header(&btp_h))	{
				/* parsing header and read the content*/
				rt_uint8_t buf[MAX_BTP_LEN];
				//printk("header 2\n");
				if(scan_buffer(dev, buf, btp_h.len )
					!= btp_h.len ){
					printk("read content failure\n");
					write_led_resp(dev, BTP_RESP_NG);
					return -1;
					//continue;
				}else{
					rt_uint8_t i, cks;
					//checksum
					//printk("header 3\n");
					cks = 0;
					for(i = 0; i < BTP_HEAD_SIZE; i++) cks+= ptr[i];
					for(i = 0; i < btp_h.len-1; i ++) cks+= buf[i];
					printk("checksum(0x%x,0x%x)\n", cks, buf[i]);
					if(cks == buf[i] ){
						//parsing it
						parsing_led_command(buf, btp_h.len );
						write_led_resp(dev, BTP_RESP_OK);
						printk("OK\n");
					}else{
						printk("error checksum\n");
						write_led_resp(dev, BTP_RESP_NG);
						return -1;
						//continue;
					}
				}
			}else{
				printk("bad header\n");
				write_led_resp(dev, BTP_RESP_NG);
			}
		}else{
			printk("read header failure\n");
			write_led_resp(dev, BTP_RESP_NG);
		}
	}else{
		printk("read start code failure\n");
		write_led_resp(dev, BTP_RESP_NG);
	}/* end of device read */
	return 0;
}
