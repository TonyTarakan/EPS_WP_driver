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
        printk(KERN_ALERT "Error spi setup SPI_CS high.\n");
        return ret;
    }
    //////////////////////////////////////////////////////////

    // power OFF
    gpio_set_value_cansleep(ESP_PWR_GPIO, 0);
    gpio_set_value_cansleep(ESP_PROG_GPIO, prog);
	msleep(ESP_RST_WAIT_MS);
	// powen ON
 	gpio_set_value_cansleep(ESP_PWR_GPIO, 1);


 	// SPI_CS_HIGH for esp8266
 	esp.spi_dev->mode = SPI_MODE_0 | SPI_CS_HIGH,
    ret = spi_setup(esp.spi_dev);
    if(ret != 0)
    {
        printk(KERN_ALERT "Error spi setup SPI_CS low.\n");
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

static ssize_t on_esp_cmd_received(struct file * file, const char __user * buf, size_t len, loff_t * ppos)
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

static void wifi_setup(struct net_device * netdev) 
{
	netdev->netdev_ops = &(esp.wifi_ndops);
	netdev->dev_addr[0] = 0xE5;
	netdev->dev_addr[1] = 0x82;
	netdev->dev_addr[2] = 0x66;
	netdev->dev_addr[3] = 0x11;
	netdev->dev_addr[4] = 0x11;
	netdev->dev_addr[5] = 0xF1;
	ether_setup(netdev);
}

static void mesh_setup(struct net_device * netdev) 
{
	netdev->netdev_ops = &(esp.mesh_ndops);
	netdev->dev_addr[0] = 0xE5;
	netdev->dev_addr[1] = 0x82;
	netdev->dev_addr[2] = 0x66;
	netdev->dev_addr[3] = 0x11;
	netdev->dev_addr[4] = 0x1E;
	netdev->dev_addr[5] = 0x5F;
	ether_setup(netdev);
} 

int spi_write_and_read(struct spi_device * spidev, unsigned char * rx_data, unsigned char * tx_data, int buf_length)
{
	int ret = 0;
	struct spi_message mess;
	struct spi_transfer transfer;

	printk(KERN_INFO "spi_write_and_read\n");

	printk(KERN_INFO "spidev: %p\n", spidev);
    printk(KERN_INFO "rx_data: %p\n", rx_data);

	transfer.tx_buf	= tx_data;
	transfer.rx_buf = rx_data;
	transfer.len	= buf_length;

	spi_message_init(&mess);
	spi_message_add_tail(&transfer, &mess);

	ret = spi_sync(spidev, &mess);
	if (ret)
	{
		printk(KERN_ALERT "spi_sync: %d\n", ret);
		return ret;
	}

    return ret;
}

static int esp_spi_read_data(esp_net_priv_t * esp_priv)
{
    int ret = 0;
    int total_read_len = 0;

    printk(KERN_INFO "esp_spi_read_data\n");

    printk(KERN_INFO "esp_priv->spi: %p\n", esp_priv->spi);
    printk(KERN_INFO "esp_priv: %p\n", esp_priv);
    
    while(gpio_get_value(ESP_HAS_DATA_GPIO))
    {
        if(!gpio_get_value(ESP_BUSY_GPIO))
        {
            continue;
        }

        esp_priv->spi_blk_tx_buf[0] = MASTER_READ_DATA_FROM_SLAVE_CMD;
        esp_priv->spi_blk_tx_buf[1] = 0;

        ret = spi_write_and_read(esp_priv->spi, esp_priv->spi_blk_rx_buf, esp_priv->spi_blk_tx_buf, ESP_SPI_BUF_SIZE);
        if(ret >= 0)
        {
        	//printk(KERN_INFO "SPI data: %s\n", esp_priv->spi_blk_rx_buf);
            //memcpy(esp_priv->spi_rx_buf + total_read_len, esp_priv->spi_blk_rx_buf, 32);
            total_read_len += 32;
            printk(KERN_INFO "SPI data: %d\n", total_read_len);
        }
    }
    
    // if(total_read_len >= read_packet_len)
    // {
    //     gateway_route((char *)read_buffer, read_packet_len);
    // }
    
    return total_read_len;
}

// Transmit pack to SPI
static void esp_rx_work_handler(struct work_struct * rx_work)
{
	int res = 0;
	esp_net_priv_t * esp_priv = container_of(rx_work, esp_net_priv_t, rx_work);


	printk(KERN_INFO "esp_rx_work_handler\n");

	printk(KERN_INFO "11 rx_work: %p\n", rx_work);
	printk(KERN_INFO "11 esp_priv: %p\n", esp_priv);

	res = esp_spi_read_data(esp_priv);
	if (res < 0)
	{
		printk(KERN_ALERT "esp_spi_read_data: %d\n", res);
		return;\
	}
	printk(KERN_INFO "esp_spi_read_data: %d bytes\n", res);

	// mutex_unlock(&esp_priv->spi_lock);
	// mutex_lock(&esp_priv->spi_lock);

	//blabla
}



// Handling of ESP_HAS_DATA event(interrupt)
static irqreturn_t esp_rx_gpio_handler(int irq, void * dev_id)
{
	//int ret = 0;
	esp_net_priv_t * esp_priv = dev_id;

	printk(KERN_INFO "!!! esp_rx_gpio_handler\n");

	// mutex_lock(&esp_priv->spi_lock);


	queue_work(esp_priv->wq, &esp_priv->rx_work);




	return IRQ_HANDLED;
}

// Transmit pack to SPI
static void esp_tx_work_handler(struct work_struct * tx_work)
{
	//esp_net_priv_t * esp_priv = container_of(tx_work, esp_net_priv_t, tx_work); //WTF??


	printk(KERN_INFO "esp_tx_work_handler\n");

	// mutex_lock(&esp_priv->spi_lock);

	//blabla

	// mutex_unlock(&esp_priv->spi_lock);
}

// Start to transmit to SPI
static int esp_net_start_tx(struct sk_buff * skb, struct net_device * netdev)
{ 
	esp_net_priv_t * esp_priv = netdev_priv(netdev);
	//struct spi_device * spi = esp_priv->spi;

	printk(KERN_INFO "esp_net_start_tx\n");




	// Add buffer processing to irq handling queue
	// netif_stop_queue(netdev);	// MUST BE WAKED AFTER !!! (maybe after spi answer)
	esp_priv->tx_skb = skb;
	queue_work(esp_priv->wq, &esp_priv->tx_work);

	return NETDEV_TX_OK;
}

static int esp_net_open(struct net_device * netdev)
{
	int res;
	esp_net_priv_t * esp_priv = netdev_priv(netdev);
	printk(KERN_INFO "esp_net_open\n");

	//struct spi_device * spi = priv->spi;

	// mutex_lock(&esp_priv->spi_lock);

	// GPIO irq registration
	res = request_irq(esp.rx_gpio_irq, esp_rx_gpio_handler, IRQF_TRIGGER_RISING, "esp_rx_gpio", esp_priv);
	if(res < 0)
	{
		printk(KERN_ALERT "request_irq: %d\n", res);
		return res;
	}

	esp_priv->wq = alloc_workqueue("esp_net_wq", WQ_FREEZABLE | WQ_MEM_RECLAIM, 0);
	if (!esp_priv->wq)
	{
		return -ENOMEM;
	}
	INIT_WORK(&(esp_priv->tx_work), esp_tx_work_handler);
	INIT_WORK(&(esp_priv->rx_work), esp_rx_work_handler);
	netif_wake_queue(netdev); 

	// mutex_unlock(&esp_priv->spi_lock);
	return 0; 
}

static int esp_net_close(struct net_device * netdev)
{ 
	esp_net_priv_t * esp_priv = netdev_priv(netdev);
	free_irq(esp.rx_gpio_irq, esp_priv);
	destroy_workqueue(esp_priv->wq);
	printk(KERN_INFO "esp_net_close\n");
	return 0; 
}


static int esp_init_spi_gpio(void)
{
	int res = 0;
	// SPI data GPIOs
    if(gpio_is_valid(ESP_BUSY_GPIO))
	{
		printk(KERN_INFO "esp_custom: ESP_BUSY_GPIO %d\n", ESP_BUSY_GPIO);

		res = gpio_request(ESP_BUSY_GPIO, "ESP_BUSY");
		if(res < 0) return res;
		res = gpio_direction_input(ESP_BUSY_GPIO);
		if(res < 0) return res;
	}
	else
	{
		printk(KERN_ALERT "esp_custom: invalid GPIO%d\n", ESP_BUSY_GPIO);
		return -EINVAL;
	}
	
	if(gpio_is_valid(ESP_HAS_DATA_GPIO))
	{
		printk(KERN_INFO "esp_custom: ESP_HAS_DATA_GPIO %d\n", ESP_HAS_DATA_GPIO);

		res = gpio_request(ESP_HAS_DATA_GPIO, "ESP_HAS_DATA_GPIO");
		if(res < 0) return res;
		res = gpio_direction_input(ESP_HAS_DATA_GPIO);
		if(res < 0) return res;
	}
	else
	{
		printk(KERN_ALERT "esp_custom: invalid GPIO%d\n", ESP_HAS_DATA_GPIO);
		return -EINVAL;
	}


	// Interrupt setup for RX data available
	esp.rx_gpio_irq = gpio_to_irq(ESP_HAS_DATA_GPIO);
    if(esp.rx_gpio_irq < 0)
    {
		printk(KERN_ALERT "gpio_to_irq: %d\n", esp.rx_gpio_irq);
		return esp.rx_gpio_irq;
	}



	return res;
}

static int esp_init_switch_gpio(void)
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
	return res;
}

static int __init ktest_module_init(void)
{
	int res = 0;

	esp_net_priv_t * esp_priv;

	// Take GPIOs under control
	res = esp_init_switch_gpio();
	if(res < 0) return res;
	//////////////////////////////////////////////////////////


	// SPI init
    esp.chip.max_speed_hz 	= ESP_SPI_MAX_SPEED;
    esp.chip.bus_num 		= ESP_SPI_BUS_NUM;
    esp.chip.chip_select 	= ESP_SPI_DEV_NUM;
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

	// Create new virtual device in /dev/ for switch control
	esp.ctrl_fops.owner 	= THIS_MODULE;
	esp.ctrl_fops.write		= on_esp_cmd_received;
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
	esp.wifi_ndops.ndo_start_xmit	= esp_net_start_tx;	// transmit socket buffer from interface to physical net

	esp.mesh_ndops.ndo_open 		= esp_net_open;
	esp.mesh_ndops.ndo_stop 		= esp_net_close;
	esp.mesh_ndops.ndo_start_xmit	= esp_net_start_tx;

/*
	esp.wifi_dev = alloc_netdev(sizeof(esp_net_priv_t), "espwifi%d", wifi_setup);
	res = register_netdev(esp.wifi_dev);
	if(res != 0)
	{
		printk(KERN_INFO "Failed to register wifi interface\n" ); 
		free_netdev(esp.wifi_dev);
		return res; 
	}*/

	esp.mesh_dev = alloc_netdev(sizeof(esp_net_priv_t), "espmesh%d", mesh_setup);

	esp_priv = netdev_priv(esp.mesh_dev);
	esp_priv->chip.max_speed_hz = ESP_SPI_MAX_SPEED;
    esp_priv->chip.bus_num 		= ESP_SPI_BUS_NUM;
    esp_priv->chip.chip_select 	= ESP_SPI_DEV_NUM;
    esp_priv->chip.mode 		= SPI_MODE_0;	// don't forget to make SPI_MODE_0 | SPI_CS_HIGH after start

    esp_priv->master = spi_busnum_to_master(esp.chip.bus_num);
    if(!(esp_priv->master))
    {
        printk("MASTER not found.\n");
        return -ENODEV;
    }

	esp_priv->spi = spi_new_device(esp_priv->master, &(esp_priv->chip));
    if(!(esp_priv->spi)) 
    {
        printk("FAILED to create slave(1).\n");
        return -ENODEV;
    }

    res = esp_init_spi_gpio();
    if(res < 0) return res;


	res = register_netdev(esp.mesh_dev);
	if(res != 0)
	{
		printk(KERN_INFO "Failed to register mesh interface\n" ); 
		free_netdev(esp.mesh_dev);
		return res; 
	}
    //////////////////////////////////////////////////////////


    printk( "ESP8266 under control\n" ); 
	return res;
}

static void __exit ktest_module_exit(void)
{
	gpio_free(ESP_PROG_GPIO);
	gpio_free(ESP_PWR_GPIO);
	gpio_free(ESP_BUSY_GPIO);
	//free_irq(esp.rx_gpio_irq, NULL);
	gpio_free(ESP_HAS_DATA_GPIO);

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
