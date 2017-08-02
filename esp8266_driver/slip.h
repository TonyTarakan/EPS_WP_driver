#ifndef __SLIP_H
#define __SLIP_H

/* SLIP protocol characters. */
#define SLIP_END             0300	// 0xC0		// indicates end of frame
#define SLIP_ESC             0333	// 0xDB		// indicates byte stuffing
#define SLIP_ESC_END         0334	// 0xDC		// ESC ESC_END means END 'data'
#define SLIP_ESC_ESC         0335	// 0xDD		// ESC ESC_ESC means ESC 'data'


static inline int slip_unstuff(unsigned char * input, int in_len)
{
	int i = 0;
	int out_len = 0;
	int pos = 0;
	bool in_progress = false;

	unsigned char * output = kmalloc(in_len, GFP_KERNEL);
	if(output == NULL)
	{
		printk(KERN_ALERT "slip_unesc ENOMEM\n");
		return -ENOMEM;
	}

	while(i < in_len)
	{
		if(input[i] == SLIP_END)
		{
			if(in_progress)
			{
				memcpy(input, output, pos);
				kfree(output);
				return pos;
			}
			else
				in_progress = true;
		}
		else if(in_progress)
		{
			if(input[i] == SLIP_ESC)
			{
				i++;
				if(input[i] == SLIP_ESC_END)
				{
					output[pos] = SLIP_END;
				}
				else if(input[i] == SLIP_ESC_ESC)
				{
					output[pos] = SLIP_ESC;
				}
				else
				{
					printk(KERN_ALERT "slip_unesc middle ERROR\n");
					kfree(output);
					return 0;
				}
				pos++;
			}
			else
			{
				output[pos] = input[i];
				pos++;
			}
		}
		i++;
	}

	int j;
	printk("FAILED UNSTUFFED data: \n");
	for(j = 0; j < in_len; j++)
	{
		if(j%32 == 0)
			printk("\n");
		printk("%02x ", input[j]);
	}
	printk("\n");
	printk(KERN_ALERT "slip_unesc end ERROR, length %d\n", in_len);
	kfree(output);

	return 0; // if we are here then it's error
}

static inline int slip_stuff(unsigned char * src, unsigned char * dst, int len)
{
    unsigned char * ptr = dst;
    unsigned char c;

    // Send an initial END character
    *ptr++ = SLIP_END;

    // For each byte in the packet, send the appropriate character sequence
    while (len-- > 0)
    {
        switch(c = *src++)
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
    return (ptr - dst);
}


#endif
