#include "esp8266_custom.h"

static esp_t esp;

static int esp_on(int prog)
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
	//////////////////////////////////////////////////////////


	// make SPI_CS high
    esp.spi_dev->mode = SPI_MODE_0,	// don't forget to make SPI_MODE_0 | SPI_CS_HIGH after start
    ret = spi_setup(esp.spi_dev);
    if(ret != 0)
    {
        printk("Error spi setup SPI_CS high.\n");
        return ret;
    }
    //////////////////////////////////////////////////////////

    // power OFF
    gpio_set_value_cansleep(ESP_PWR_GPIO, 0);
    gpio_set_value_cansleep(ESP_PROG_GPIO, prog);
	msleep(2000);
	// powen ON
 	gpio_set_value_cansleep(ESP_PWR_GPIO, 1);


 	// SPI_CS_HIGH for esp8266
 	esp.spi_dev->mode = SPI_MODE_0 | SPI_CS_HIGH,
    ret = spi_setup(esp.spi_dev);
    if(ret != 0)
    {
        printk("Error spi setup SPI_CS low.\n");
        return ret;
    }
    //////////////////////////////////////////////////////////

	return ret;
}

static int esp_off(void)
{
	// power OFF
    gpio_set_value_cansleep(ESP_PWR_GPIO, 0);
	return 0;
}

static ssize_t esp_cmd_write(struct file * file, const char __user * buf, size_t len, loff_t * ppos)
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
		// spi_write(esp.spi_dev, eer, 10);



		ret = esp_on(ESP_BOOT_FLASH);
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
			ret = esp_on(ESP_BOOT_FLASH);
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
			ret = esp_on(ESP_BOOT_UART);
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

static void wifi_setup(struct net_device * dev) 
{
	dev->netdev_ops = &(esp.wifi_ndops);
	dev->dev_addr[0] = 0xE5;
	dev->dev_addr[1] = 0x82;
	dev->dev_addr[2] = 0x66;
	dev->dev_addr[3] = 0x11;
	dev->dev_addr[4] = 0x11;
	dev->dev_addr[5] = 0xF1;
	ether_setup(dev);
}

static void mesh_setup(struct net_device * dev) 
{
	dev->netdev_ops = &(esp.mesh_ndops);
	dev->dev_addr[0] = 0xE5;
	dev->dev_addr[1] = 0x82;
	dev->dev_addr[2] = 0x66;
	dev->dev_addr[3] = 0x11;
	dev->dev_addr[4] = 0x1E;
	dev->dev_addr[5] = 0x5F;
	ether_setup(dev);
} 

static int esp_net_open( struct net_device * dev )
{ 
   netif_start_queue(dev); 
   return 0; 
}

static int esp_net_close( struct net_device * dev)
{ 
   netif_stop_queue(dev); 
   return 0; 
} 

static int esp_net_start_xmit( struct sk_buff * skb, struct net_device * dev)
{ 
   dev_kfree_skb(skb); 
   return 0; 
}


static int __init ktest_module_init( void )
{
	int res = 0;


	// Take GPIOs under control
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
    //////////////////////////////////////////////////////////


	// SPI init
    esp.chip.max_speed_hz 	= 5000000;
    esp.chip.bus_num 		= 0;
    esp.chip.chip_select 	= 0;
    esp.chip.mode 			= SPI_MODE_0;	// don't forget to make SPI_MODE_0 | SPI_CS_HIGH after start

    esp.master = spi_busnum_to_master(esp.chip.bus_num);
    if(!(esp.master))
    {
        printk("MASTER not found.\n");
        return -ENODEV;
    }

	esp.spi_dev = spi_new_device(esp.master, &(esp.chip));
    if(!(esp.spi_dev)) 
    {
        printk("FAILED to create slave(1).\n");
        return -ENODEV;
    }
    //////////////////////////////////////////////////////////


	// Create new virtual device in /dev/ for switch control
	esp.ctrl_fops.owner 	= THIS_MODULE;
	esp.ctrl_fops.write		= esp_cmd_write;
	esp.ctrl_fops.llseek 	= no_llseek;
	esp.ctrl_dev.minor 		= MISC_DYNAMIC_MINOR;
    esp.ctrl_dev.name 		= "esp8266_ctrl";
    esp.ctrl_dev.fops 		= &(esp.ctrl_fops);

    res = misc_register(&(esp.ctrl_dev));
	if(res != 0)
	{
		printk( KERN_INFO "Failed to register control interface\n" ); 
		return res; 
	}
    //////////////////////////////////////////////////////////


	// Create new network devices
	esp.wifi_ndops.ndo_open 		= esp_net_open;
	esp.wifi_ndops.ndo_stop 		= esp_net_close;
	esp.wifi_ndops.ndo_start_xmit	= esp_net_start_xmit;

	esp.mesh_ndops.ndo_open 		= esp_net_open;
	esp.mesh_ndops.ndo_stop 		= esp_net_close;
	esp.mesh_ndops.ndo_start_xmit	= esp_net_start_xmit;


	esp.wifi_dev = alloc_netdev( 0, "espwifi%d", wifi_setup);
	res = register_netdev(esp.wifi_dev);
	if(res != 0)
	{
		printk( KERN_INFO "Failed to register wifi interface\n" ); 
		free_netdev(esp.wifi_dev);
		return res; 
	}

	esp.mesh_dev = alloc_netdev( 0, "espmesh%d", mesh_setup);
	res = register_netdev(esp.mesh_dev);
	if(res != 0)
	{
		printk( KERN_INFO "Failed to register mesh interface\n" ); 
		free_netdev(esp.mesh_dev);
		return res; 
	}
    //////////////////////////////////////////////////////////


    printk( "ESP8266 under control\n" ); 
	return res;
}

static void __exit ktest_module_exit( void )
{
	gpio_free(ESP_PROG_GPIO);
	gpio_free(ESP_PWR_GPIO);
	spi_unregister_device(esp.spi_dev); // ??? is it enough?
	unregister_netdev(esp.wifi_dev);
	free_netdev(esp.wifi_dev);
	unregister_netdev(esp.mesh_dev);
	free_netdev(esp.mesh_dev);
	misc_deregister(&(esp.ctrl_dev)); 
	printk( "ESP8266 is free\n" ); 
}

MODULE_LICENSE("GPL");
 
module_init(ktest_module_init);
module_exit(ktest_module_exit);
