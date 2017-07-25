#ifndef __SLIP_H
#define __SLIP_H

#define BUF_SIZE 1500
 /* two bytes each for a (pathological) max packet of escaped chars +  *
  * terminating END char + initial END char                            */
#define ENC_BUF_SIZE (2 * BUF_SIZE + 2)

/* SLIP protocol characters. */
#define SLIP_END             0300	// 0xC0		// indicates end of frame
#define SLIP_ESC             0333	// 0xDB		// indicates byte stuffing
#define SLIP_ESC_END         0334	// 0xDC		// ESC ESC_END means END 'data'
#define SLIP_ESC_ESC         0335	// 0xDD		// ESC ESC_ESC means ESC 'data'

static inline int slip_unesc(unsigned char * buf, int length)
{
	int i = 1;
	if(buf[0] != SLIP_END)
	{
		printk(KERN_ALERT "slip_unesc begin ERROR\n");
		return -1;
	}

	while(i < length)
	{
		if(buf[i] == SLIP_ESC)
		{
			i++;
			if(buf[i] == SLIP_ESC_END)
			{
				buf[i-1] = SLIP_END;
				memcpy( &buf[i], &buf[i+1], length-(i+1) );
				length--;
			}
			else if(buf[i] == SLIP_ESC_ESC)
			{
				buf[i-1] = SLIP_ESC;
				memcpy( &buf[i], &buf[i+1], length-(i+1) );
				length--;
			}
			else
			{
				printk(KERN_ALERT "slip_unesc middle ERROR\n");
				return -1;
			}
			break;
		}
		i++;
	}

	if(buf[i] != SLIP_END)
	{
		printk(KERN_ALERT "slip_unesc end ERROR\n");
		return -1;
	}

	return length - 2; // BEGIN and END bytes
}

static inline int slip_esc(unsigned char * s, unsigned char * d, int len)
{
	unsigned char * ptr = d;
	unsigned char c;

	// Send an initial END character
	*ptr++ = SLIP_END;

	// For each byte in the packet, send the appropriate character sequence
	while (len-- > 0)
	{
		switch(c = *s++)
		{
		case SLIP_END:
			*ptr++ = SLIP_ESC;
			*ptr++ = SLIP_ESC_END;
			break;
		case SLIP_ESC:
			*ptr++ = SLIP_ESC;
			*ptr++ = SLIP_ESC_ESC;
			break;
		default:
			*ptr++ = c;
			break;
		}
	}
	*ptr++ = SLIP_END;
	return (ptr - d);
}

struct slip_proto 
{
	unsigned char ibuf[ENC_BUF_SIZE];
	unsigned char obuf[ENC_BUF_SIZE];
	int pos;
	int esc;
};

static inline void slip_proto_init(struct slip_proto * slip)
{
	memset(slip->ibuf, 0, sizeof(slip->ibuf));
	memset(slip->obuf, 0, sizeof(slip->obuf));
	slip->pos = 0;
	slip->esc = 0;
}

// extern int slip_proto_read(int fd, void *buf, int len, struct slip_proto *slip);
// extern int slip_proto_write(int fd, void *buf, int len, struct slip_proto *slip);

#endif
