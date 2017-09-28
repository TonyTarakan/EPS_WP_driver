#include "slip.h"

int slip_unstuff(slip_t * context, unsigned char input, int (*data_route)(slip_t * context))
{
	int res = 0;
	if(context->pos >= context->maxlen)
	{
		context->pos = 0;
		context->ready_to_replace = false;
		printk("ESP RX buffer overflow\n");
		return -ENOMEM;
	}

	if(input == SLIP_END)
	{
		if(context->pos > 4)
		{
			res = data_route(context);
		}
		context->ready_to_replace = false;
		context->pos = 0;

		if(res) return res;
	}
	else if (input == SLIP_ESC)
	{
		context->ready_to_replace = true;
	}
	else if(context->ready_to_replace)
	{
		if(input == SLIP_ESC_END)
		{
			context->output[context->pos] = SLIP_END;
		}
		else if(input == SLIP_ESC_ESC)
		{
			context->output[context->pos] = SLIP_ESC;
		}
		else
		{
			printk(KERN_ALERT "slip_unesc middle ERROR: %02x\n", context->output[context->pos]);
			context->ready_to_replace = false;
			context->pos = 0;
			return -EINVAL;
		}
		context->ready_to_replace = false;
		context->pos++;
	}
	else
	{
		context->output[context->pos] = input;
		context->pos++;
	}
	return res;
}

int slip_stuff(unsigned char * src, unsigned char * dst, int len)
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
