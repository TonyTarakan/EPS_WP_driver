#include "esp8266_custom.h"

static esp_transfer_t esp_transfer;

static int esp_on(void)
{
	int ret = 0;


	// check if UART is OK (we must be ensure that UART_RX has high voltage level)
	struct file * f_esp_uart;
	mm_segment_t oldfs;
	oldfs = get_fs();
	set_fs(KERNEL_DS);

	f_esp_uart = filp_open(ESP_UART_DEV, O_RDWR, ESP_UART_MODE);
	if(IS_ERR_OR_NULL(f_esp_uart)) 
	{
		printk(KERN_ALERT "Error while trying to open %s: %p\n", ESP_UART_DEV, f_esp_uart);
		return -EBADFD;
	}
	filp_close(f_esp_uart, NULL);
	set_fs(oldfs);





	return ret;
}

static int esp_off(void)
{
	return 0;
}

static int esp_reset(void)
{
	return 0;
}

static int esp_to_prog(void)
{
	return 0;
}

static ssize_t esp_cmd_write(struct file *file, const char __user *buf, size_t len, loff_t *ppos)
{
	ssize_t ret;
	void * cmd_buf;

	if((len > ESP_MAX_CMD_LEN) || (len == 0))
	{
		pr_info("Wrong size of ESP command\n");
		return -EINVAL;
	}
	
	cmd_buf = kmalloc(len, GFP_KERNEL);
	if(cmd_buf == NULL)
		return -ENOMEM;

	ret = copy_from_user(cmd_buf, buf, len);
	if(ret != 0)
	{
		kfree(cmd_buf);
		return -ENOMEM;
	}

	if(memcmp(buf, ESP_CMD_ON, len) == 0)
	{
		ret = esp_on();
		if(ret != 0)
			pr_info("ERROR ON\n");
		else
    		pr_info("ESP ON\n");
	}
    else if(memcmp(buf, ESP_CMD_OFF, len) == 0)
    {
    	ret = esp_off();
    	if(ret != 0)
			pr_info("ERROR ON\n");
		else
		pr_info("ESP OFF\n");
    }
	else if(memcmp(buf, ESP_CMD_RESET, len) == 0)
	{
		ret = esp_reset();
		if(ret != 0)
			pr_info("ERROR ON\n");
		else
		pr_info("ESP RESET\n");
	}
		else if(memcmp(buf, ESP_CMD_PROG, len) == 0)
	{
		ret = esp_to_prog();
		if(ret != 0)
			pr_info("ERROR ON\n");
		else
		pr_info("ESP switched to programming mode\n");
	}
	else
	{
		pr_info("Unknown ESP command\n");
	}

	kfree(cmd_buf);
    return len;
}

static const struct file_operations esp_ctrl_fops = 
{
    .owner			= THIS_MODULE,
    .write			= esp_cmd_write,
    .llseek 		= no_llseek,
};

static struct miscdevice esp_ctrl = 
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "esp8266_control",
    .fops = &esp_ctrl_fops,
};

static int __init ktest_module_init( void )
{
	int res = 0;

	// take spi under control
	struct spi_master * master;

/*	struct spi_board_info * chip;
	spi_device_info->modalias = "my-device-driver-name",
    spi_device_info->max_speed_hz = 1, //speed your device (slave) can handle
    spi_device_info->bus_num = 0,
    spi_device_info->chip_select = 0,
    spi_device_info->mode = 3,

	esp_transfer.spi_dev = spi_new_device(master, chip);*/






	// Create new virtual device in /dev/ 
	esp_ctrl.minor = MISC_DYNAMIC_MINOR;
    esp_ctrl.name = "esp8266_ctrl";
    esp_ctrl.fops = &esp_ctrl_fops;

    res = misc_register(&esp_ctrl);
    if (res) 
    	return res;


    printk( "ESP8266 under control\n" ); 
	return res;
} 
static void __exit ktest_module_exit( void )
{
	// gpio_free(ESP_RST_GPIO);
	// gpio_free(ESP_PWR_GPIO);
	misc_deregister( &esp_ctrl ); 
	printk( "ESP8266 is free\n" ); 
}

MODULE_LICENSE("GPL");
 
module_init(ktest_module_init);
module_exit(ktest_module_exit);
