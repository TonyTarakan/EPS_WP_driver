#ifndef __SLIP_H
#define __SLIP_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>


/* SLIP protocol characters. */
#define SLIP_END             0300	// 0xC0		// indicates end of frame
#define SLIP_ESC             0333	// 0xDB		// indicates byte stuffing
#define SLIP_ESC_END         0334	// 0xDC		// ESC ESC_END means END 'data'
#define SLIP_ESC_ESC         0335	// 0xDD		// ESC ESC_ESC means ESC 'data'

typedef struct
{
	int maxlen;
	int pos;
	bool ready_to_replace;
	unsigned char * output;
}slip_t;

int slip_unstuff(slip_t * context, unsigned char input, int (*data_route)(slip_t * context));
int slip_stuff(unsigned char * src, unsigned char * dst, int len);

#endif
