/*
 * File      : btapp.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2011, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-03-02	  Thomas Tsai  a btapp shell downgraded from finsh
 * 2011-06-02     Bernard      Add btapp_get_prompt function declaration
 */

#ifndef __BTAPP_H__
#define __BTAPP_H__

#include <rtthread.h>

#ifndef BTAPP_THREAD_PRIORITY
#define BTAPP_THREAD_PRIORITY 20
#endif
#ifndef BTAPP_THREAD_STACK_SIZE
#define BTAPP_THREAD_STACK_SIZE 2048
#endif
#define BTAPP_CMD_SIZE		80

struct btapp_shell
{
	struct rt_semaphore rx_sem;

	enum input_stat stat;

	struct btapp_parser parser;

	char line[BTAPP_CMD_SIZE];
	rt_uint8_t line_position;

	rt_device_t device;
};

void btapp_init(const char* device_name);
void btapp_set_device(const char* device_name);
const char* btapp_get_device(void);

#endif
