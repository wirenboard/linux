/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * i.MX GPT Input Capture support.
 *
 * Copyright (C) 2015 Mentor Graphics, Inc. All Rights Reserved.
 */
#ifndef __MXC_ICAP_H__
#define __MXC_ICAP_H__

typedef void (*mxc_icap_handler_t)(int, void *, ktime_t);

int mxc_request_input_capture(unsigned int chan, mxc_icap_handler_t handler,
			      unsigned long capflags, void *dev_id);
void mxc_free_input_capture(unsigned int chan, void *dev_id);

#endif
