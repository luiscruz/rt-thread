/*
 * File      : bt_app.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 03-02-2013	  Thomas Tsai	cloned from finsh to use usart
 * 2006-04-30     Bernard      the first verion for FinSH
 * 2006-05-08     Bernard      change btapp thread stack to 2048
 * 2006-06-03     Bernard      add support for skyeye
 * 2006-09-24     Bernard      remove the code related with hardware
 * 2010-01-18     Bernard      fix down then up key bug.
 * 2010-03-19     Bernard      fix backspace issue and fix device read in btapp_sh.
 * 2010-04-01     Bernard      add prompt output when start and remove the empty history
 * 2011-02-23     Bernard      fix variable section end issue of btapp btapp_sh
 *                             initialization when use GNU GCC compiler.
 */

#include <rtthread.h>
#include <rthw.h>

#include "bt_app.h"

#define	printk	rt_kprintf

/* btapp thread */
static struct rt_thread btapp_thread;
ALIGN(RT_ALIGN_SIZE)
static char btapp_thread_stack[BTAPP_THREAD_STACK_SIZE];
struct btapp_shell* btapp_sh;

#if !defined (RT_USING_NEWLIB) && !defined (RT_USING_MINILIBC)
int strcmp (const char *s1, const char *s2)
{
	while (*s1 && *s1 == *s2) s1++, s2++;

	return (*s1 - *s2);
}

#if !defined(__CC_ARM) && !defined(__IAR_SYSTEMS_ICC__) && !defined(__ADSPBLACKFIN__) && !defined(_MSC_VER)
int isalpha( int ch )
{
	return (unsigned int)((ch | 0x20) - 'a') < 26u;
}

int atoi(const char* s)
{
	long int v=0;
	int sign=1;
	while ( *s == ' '  ||  (unsigned int)(*s - 9) < 5u) s++;

	switch (*s)
	{
	case '-': sign=-1;
	case '+': ++s;
	}

	while ((unsigned int) (*s - '0') < 10u)
	{
		v=v*10+*s-'0'; ++s;
	}

	return sign==-1?-v:v;
}

int isprint(unsigned char ch)
{
    return (unsigned int)(ch - ' ') < 127u - ' ';
}
#endif
#endif

static rt_err_t btapp_rx_ind(rt_device_t dev, rt_size_t size)
{
	RT_ASSERT(btapp_sh != RT_NULL);

	/* release semaphore to let btapp thread rx data */
	//rt_sem_release(&btapp_sh->rx_sem);

	return RT_EOK;
}

/**
 * @ingroup btapp
 *
 * This function sets the input device of btapp btapp_sh.
 *
 * @param device_name the name of new input device. "usart1", "usart2"...
 */
void btapp_set_device(const char* device_name)
{
	rt_device_t dev = RT_NULL;

	RT_ASSERT(btapp_sh != RT_NULL);
	dev = rt_device_find(device_name);
	if (dev != RT_NULL && rt_device_open( dev, RT_DEVICE_OFLAG_RDWR) == RT_EOK)
	{
		if (btapp_sh->device != RT_NULL)
		{
			/* close old btapp device */
			rt_device_close(btapp_sh->device);
		}

		btapp_sh->device = dev;
		rt_device_set_rx_indicate(dev, btapp_rx_ind);
	}
	else
	{
		printk("btapp: can not find device:%s\n", device_name);
	}
}

/**
 * @ingroup btapp
 *
 * This function returns current btapp btapp_sh input device.
 *
 * @return the btapp btapp_sh input device name is returned.
 */
const char* btapp_get_device()
{
	RT_ASSERT(btapp_sh != RT_NULL);
	return btapp_sh->device->parent.name;
}

/* running the command from bt channel */
void btapp_run_line(struct btapp_parser* parser, const char *line)
{
	const char* err_str;

	btapp_parser_run(parser, (unsigned char*)line);

	/* compile node root */
	if (btapp_errno() == 0)
	{
		btapp_compiler_run(parser->root);
	}
	else
	{
		err_str = btapp_error_string(btapp_errno());
		printk("%s\n", err_str);
	}

	/* run virtual machine */
	if (btapp_errno() == 0)
	{
		char ch;
		btapp_vm_run();

		ch = (unsigned char)btapp_stack_bottom();
		if (ch > 0x20 && ch < 0x7e)
		{
			printk("\t'%c', %d, 0x%08x\n",
				(unsigned char)btapp_stack_bottom(),
				(unsigned int)btapp_stack_bottom(),
				(unsigned int)btapp_stack_bottom());
		}
		else
		{
			printk("\t%d, 0x%08x\n",
				(unsigned int)btapp_stack_bottom(),
				(unsigned int)btapp_stack_bottom());
		}
	}

    btapp_flush(parser);
}

struct btapp_shell _btapp_sh;
void btapp_thread_entry(void* parameter)
{
    char ch;

    btapp_pinit(&btapp_sh->parser);/* init parser */

	while (1)
	{
		/* wait receive */
		//if (rt_sem_take(&btapp_sh->rx_sem, RT_WAITING_FOREVER) != RT_EOK) continue;

		/* read one character from device */
		while (rt_device_read(btapp_sh->device, 0, &ch, 1) == 1)
		{
			/* handle CR key */
			if (ch == '\r')
			{
				char next;

				if (rt_device_read(btapp_sh->device, 0, &next, 1) == 1)
					ch = next;
				else ch = '\r';
			}
			/* handle end of line, break */
			if (ch == '\r' || ch == '\n')
			{
				/* change to ';' and break */
				btapp_sh->line[btapp_sh->line_position] = ';';

				if (btapp_sh->line_position != 0) btapp_run_line(&btapp_sh->parser, btapp_sh->line);
				else printk("\n");
				printk("OK\n");
				memset(btapp_sh->line, 0, sizeof(btapp_sh->line));
				btapp_sh->line_position = 0;

				break;
			}

			/* it's a large line, discard it */
			if (btapp_sh->line_position >= BTAPP_CMD_SIZE) btapp_sh->line_position = 0;

			/* normal character */
			btapp_sh->line[btapp_sh->line_position] = ch; ch = 0;
			btapp_sh->line_position ++;
		} /* end of device read */
	}
}

/*
 * @ingroup btapp
 *
 * This function will initialize btapp btapp_sh
 */
void btapp_init(const char* device_name)
{
	rt_err_t result;

	/* create or set btapp_sh structure */
	btapp_sh = &_btapp_sh;

	memset(btapp_sh, 0, sizeof(struct btapp_shell));

	//rt_sem_init(&(btapp_sh->rx_sem), "btapprx", 0, 0);
	result = rt_thread_init(&btapp_thread,
		"btapp",
		btapp_thread_entry, RT_NULL,
		&btapp_thread_stack[0], sizeof(btapp_thread_stack),
		BTAPP_THREAD_PRIORITY, 10);

	if (result == RT_EOK)
		rt_thread_startup(&btapp_thread);

	btapp_set_device(device_name);
}
