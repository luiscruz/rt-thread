#ifndef __NRF24AP_H__
#define __NRF24AP_H__

#include <rtthread.h>

struct calibration_data
{
	rt_uint16_t min_x, max_x;
	rt_uint16_t min_y, max_y;
};

typedef void (*rt_touch_calibration_func_t)(rt_uint16_t x, rt_uint16_t y);

void rtgui_nRF24AP_hw_init(void);

#endif

