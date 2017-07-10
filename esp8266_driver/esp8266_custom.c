#include "esp8266_custom.h"

static esp_transfer_t esp_transfer;

static int esp_on(int prog)
{
	int ret = 0;

	struct file * f_esp_uart;
	mm_segment_t oldfs;

	struct spi_master * master;
	struct spi_board_info chip;

	// check if UART is OK (we must be ensure that UART_RX has high voltage level)
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


	// make SPI_CS high
    chip.max_speed_hz = 5000000,
    chip.bus_num = 0,
    chip.chip_select = 0,
    chip.mode = SPI_MODE_0,	// don't forget to make SPI_MODE_0 | SPI_CS_HIGH after start

    master = spi_busnum_to_master(chip.bus_num);
    if(!master)
    {
        printk("MASTER not found.\n");
        return -ENODEV;
    }

	esp_transfer.spi_dev = spi_new_device(master, &chip);
    if(!(esp_transfer.spi_dev)) 
    {
        printk("FAILED to create slave(1).\n");
        return -ENODEV;
    }
    spi_unregister_device(esp_transfer.spi_dev);


    // power OFF
    gpio_set_value_cansleep(ESP_PWR_GPIO, 0);
    gpio_set_value_cansleep(ESP_PROG_GPIO, prog);
	mdelay(2000);
	// powen ON
 	gpio_set_value_cansleep(ESP_PWR_GPIO, 1);

 	// SPI_CS_HIGH for esp8266
 	chip.mode = SPI_MODE_0 | SPI_CS_HIGH,
    master = spi_busnum_to_master(chip.bus_num);
    if(!master)
    {
        printk("MASTER not found.\n");
        return -ENODEV;
    }

	esp_transfer.spi_dev = spi_new_device(master, &chip);
    if(!(esp_transfer.spi_dev)) 
    {
        printk("FAILED to create slave(2).\n");
        return -ENODEV;
    }


	return ret;
}

static int esp_off(void)
{
	// power OFF
	spi_unregister_device(esp_transfer.spi_dev);
    gpio_set_value_cansleep(ESP_PWR_GPIO, 0);
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
		// char * eer = "dfsafasdfa";
		// spi_write(esp_transfer.spi_dev, eer, 10);




		ret = esp_on(0);
		if(ret != 0)
			pr_info("ERROR ON\n");
		else
    		pr_info("ESP ON\n");
	}
    else if(memcmp(buf, ESP_CMD_OFF, len) == 0)
    {
    	ret = esp_off();
    	if(ret != 0)
			pr_info("ERROR OFF\n");
		else
			pr_info("ESP OFF\n");
    }
	else if(memcmp(buf, ESP_CMD_RESET, len) == 0)
	{
		ret = esp_off();
		if(ret != 0)
			pr_info("ERROR OFF\n");
		else
		{
			ret = esp_on(0);
			if(ret != 0)
				pr_info("ERROR ON\n");
			else
				pr_info("ESP RESET\n");
		}
	}
	else if(memcmp(buf, ESP_CMD_PROG, len) == 0)
	{
		ret = esp_off();
		if(ret != 0)
			pr_info("ERROR OFF\n");
		else
		{
			ret = esp_on(1);
			if(ret != 0)
				pr_info("ERROR ON\n");
			else
				pr_info("ESP switched to programming mode\n");
		}
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

	if(gpio_is_valid(ESP_PROG_GPIO))
	{
		printk("esp_custom: ESP_PROG_GPIO %d\n", ESP_PROG_GPIO);

		res = gpio_request(ESP_PROG_GPIO, "ESP_RST");
		if(res < 0) return res;
		res = gpio_direction_output(ESP_PROG_GPIO, 0);
		if(res < 0) return res;
	}
	else
	{
		printk("esp_custom: invalid GPIO%d\n", ESP_PROG_GPIO);
		return -EINVAL;
	}

	if(gpio_is_valid(ESP_PWR_GPIO))
	{
		printk("esp_custom: ESP_PWR_GPIO %d\n", ESP_PWR_GPIO);

		res = gpio_request(ESP_PWR_GPIO, "ESP_PWR");
		if(res < 0) return res;
		res = gpio_direction_output(ESP_PWR_GPIO, 0);
		if(res < 0) return res;
	}
	else
	{
		printk("esp_custom: invalid GPIO%d\n", ESP_PWR_GPIO);
		return -EINVAL;
	}

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
	gpio_free(ESP_PROG_GPIO);
	gpio_free(ESP_PWR_GPIO);
	spi_unregister_device(esp_transfer.spi_dev);
	misc_deregister( &esp_ctrl ); 
	printk( "ESP8266 is free\n" ); 
}

MODULE_LICENSE("GPL");
 
module_init(ktest_module_init);
module_exit(ktest_module_exit);
