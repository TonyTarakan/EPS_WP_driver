#include "esp8266.h"

static esp_t esp;
struct net_device * wifi_dev;
struct net_device * mesh_dev;


// Just catch interrupt and then process with work_handler
static irqreturn_t esp_irq_handler(int irq, void * dev_id)
{
	// printk(KERN_INFO "esp_irq_handler\n");
	queue_work(esp.wq, &(esp.work));
	return IRQ_HANDLED;
}


// maybe one function can be used fo it?
static void esp_net_mesh_setup(struct net_device * netdev) 
{
	netdev->netdev_ops = &(esp.ndops);
	/* Point-to-Point TUN Device */
	netdev->hard_header_len = 0;
	netdev->addr_len = 0;
	netdev->mtu = 200;
	/* Zero header length */
	netdev->type = ARPHRD_NONE;
	netdev->flags = IFF_NOARP | IFF_MULTICAST;
	netdev->tx_queue_len = 500;  /* We prefer our own queue length */
}
static void esp_net_setup(struct net_device * netdev) 
{
	netdev->netdev_ops = &(esp.ndops);
	/* Point-to-Point TUN Device */
	netdev->hard_header_len = 0;
	netdev->addr_len = 0;
	netdev->mtu = 1500;
	/* Zero header length */
	netdev->type = ARPHRD_NONE;
	netdev->flags = IFF_NOARP | IFF_MULTICAST;
	netdev->tx_queue_len = 500;  /* We prefer our own queue length */
}
static void esp_net_mac_setup(struct net_device * netdev, unsigned char * devaddr, int length) 
{
	memcpy(netdev->dev_addr, devaddr, length);
}


/*
 * Netdev operations
 */
static int esp_net_start_tx(struct sk_buff * skb, struct net_device * netdev)
{ 
	skb_queue_tail(&esp.q_to_spi, skb);
	queue_work(esp.wq, &esp.work);
	return NETDEV_TX_OK;
}

static int esp_net_open(struct net_device * netdev)
{
	int res;
	printk(KERN_INFO "esp_net_open\n");

	// GPIO irq registration
	if(!esp.data_irq_captured) 
	{
		res = request_threaded_irq(esp.data_gpio_irq, esp_irq_handler, NULL, (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING), "esp_data_gpio", NULL);
		if(res < 0)
		{
			printk(KERN_ALERT "request_irq data_gpio_irq: %d\n", res);
			return res;
		}
		esp.data_irq_captured = true;
	}
	if(!esp.ready_irq_captured)
	{
		res = request_threaded_irq(esp.ready_gpio_irq, esp_irq_handler, NULL, (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING), "esp_ready_gpio", NULL);
		if(res < 0)
		{
			printk(KERN_ALERT "request_irq ready_gpio_irq: %d\n", res);
			return res;
		}
		esp.ready_irq_captured = true;
	}

	netif_wake_queue(netdev);  // for what ???
	return 0; 
}

static int esp_net_close(struct net_device * netdev)
{
	if(esp.data_irq_captured) 
	{
		free_irq(esp.data_gpio_irq, NULL);
		esp.data_irq_captured = false;
	}
	if(esp.ready_irq_captured)
	{
		free_irq(esp.ready_gpio_irq, NULL);
		esp.ready_irq_captured = false;
	}
	printk(KERN_INFO "esp_net_close\n");
	return 0; 
}




static int esp_spi_transfer(struct spi_device * spidev, unsigned char * tx_data, unsigned char * rx_data, int buf_length)
{
	int ret = 0;
	struct spi_message mess;
	struct spi_transfer transfer;

	memset(&transfer, 0, sizeof(struct spi_transfer));

	// printk(KERN_INFO "esp_spi_transfer\n");

	transfer.tx_buf	= tx_data;
	transfer.rx_buf = rx_data;
	transfer.len	= buf_length;

	spi_message_init(&mess);
	spi_message_add_tail(&transfer, &mess);

	ret = spi_sync(spidev, &mess);
	if (ret)
	{
		printk(KERN_ALERT "spi_sync ERR: %d\n", ret);
		return ret; 
	}

	return ret;
}

static int data_route(slip_t * context)
{
	struct net_device * dev;
	struct sk_buff * skb;

	unsigned char * data = context->output;
	int length = context->pos;

	int j;

	if((data[0] & 0xF0) == 0x40)
	{
		// IP routing
		if((data[0] == 0x45) && (data[12] == 10))
		{
			dev = mesh_dev;
		}
		else
		{
			dev = wifi_dev;
		}
	}
	else if (data[0] == 0x54)
	{
		// Routing table handling
		printk(KERN_NOTICE "Routing table received\n");
		for(j = 0; j < length; j++)
			printk("%02x ", data[j]);
		printk("\n");
		return 0;
	}
	else
	{
		printk(KERN_NOTICE "Routing unknown packet type: Packet dropped\n");
		for(j = 0; j < length; j++)
			printk("%02x ", data[j]);
		printk("\n");
		return 0;
	}
		

	skb = dev_alloc_skb(length);
	if (!skb)
	{
		printk(KERN_NOTICE "ESP received packet dropped (dev_alloc_skb)\n");
		return -ENOMEM;
	}

	skb->dev = dev;
	skb->pkt_type = PACKET_HOST;
	skb->protocol = htons(ETH_P_IP);
	skb->ip_summed = CHECKSUM_UNNECESSARY;

	memcpy(skb_put(skb, length), data, length);
	netif_rx(skb);
	return 0;
}


static int esp_spi_read_data(void)
{
	int ret = 0;
	int i;
	int (*route_callback)(slip_t * context) = &data_route;
	
	while(gpio_get_value_cansleep(ESP_HAS_DATA_GPIO) != 0)
	{
		if(gpio_get_value_cansleep(ESP_READY_GPIO) == 0) continue; // don't do anything if ESP is busy 

		esp.spi_blk_tx_buf[0] = MASTER_READ_DATA_FROM_SLAVE_CMD;
		esp.spi_blk_tx_buf[1] = 0;

		ret = esp_spi_transfer(esp.spi_dev, esp.spi_blk_tx_buf, esp.spi_blk_rx_buf, ESP_SPI_BUF_SIZE);
		if(ret == 0)
		{
			for(i = ESP_SPI_BUF_HEAD_SIZE; i < ESP_SPI_BUF_SIZE; i++)
			{
				//printk("dataroute before %p\n", data_route);
				ret = slip_unstuff(&esp.slip_context, esp.spi_blk_rx_buf[i], route_callback);
				if(ret)
					return ret;
			}
			// printk(KERN_INFO "SPI data: %d\n", total_read_len);
		}
	}

	return ret;
}

void esp_spi_write_data(unsigned char * spi_tx_buf, int len)
{
	// TODO: send skb->data with skb->len to SPI
	int ret = 0;
	int total_write_len = 0;

	// printk(KERN_INFO "esp_spi_write_data\n");

	while(total_write_len < len)
	{
		// printk(KERN_INFO "esp_spi HAS DATA\n");
		if(gpio_get_value_cansleep(ESP_READY_GPIO) == 0)
		{
			// printk(KERN_INFO "esp_spi_write_data IS BUSY\n");
			continue;
		}
		// else
		// 	printk(KERN_INFO "esp_spi_write_data IS READY\n");

		esp.spi_blk_tx_buf[0] = MASTER_WRITE_DATA_TO_SLAVE_CMD;
		esp.spi_blk_tx_buf[1] = 0;

		memcpy(esp.spi_blk_tx_buf + ESP_SPI_BUF_HEAD_SIZE, spi_tx_buf + total_write_len, ESP_SPI_BUF_DATA_SIZE);

		ret = esp_spi_transfer(esp.spi_dev, esp.spi_blk_tx_buf, esp.spi_blk_rx_buf, ESP_SPI_BUF_SIZE);
		if(ret >= 0)
		{
			// printk("-> SPI data: ");
			// for(j = 0; j < ESP_SPI_BUF_SIZE; j++)
			// 	printk("%02x ", esp.spi_blk_tx_buf[j]);
			// printk("\n");
			total_write_len += ESP_SPI_BUF_DATA_SIZE;
		}
	}
}



int esp_net_init(void)
{
	int res;
	unsigned char devaddr_mesh[6] = {0xE5, 0x82, 0x66, 0x11, 0x1E, 0x5F};
	unsigned char devaddr_wifi[6] = {0xE5, 0x82, 0x66, 0x11, 0x11, 0xF1};


	esp.ndops.ndo_open 			= esp_net_open;
	esp.ndops.ndo_stop 			= esp_net_close;
	esp.ndops.ndo_start_xmit 	= esp_net_start_tx;


	wifi_dev = alloc_netdev(sizeof(esp_net_priv_t), "espwifi%d", esp_net_setup);

	// !!! hardcode
	esp_net_mac_setup(wifi_dev, devaddr_wifi, 6);

	res = register_netdev(wifi_dev);
	if(res != 0)
	{
		printk(KERN_INFO "Failed to register wifi interface\n" ); 
		free_netdev(wifi_dev);
		return res; 
	}

	mesh_dev = alloc_netdev(sizeof(esp_net_priv_t), "espmesh%d", esp_net_mesh_setup);

	// !!! hardcode
	esp_net_mac_setup(mesh_dev, devaddr_mesh, 6);

	res = register_netdev(mesh_dev);
	if(res != 0)
	{
		printk(KERN_INFO "Failed to register mesh interface\n" ); 
		free_netdev(mesh_dev);
		return res; 
	}

	return 0;
}

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
	esp.spi_dev->mode = SPI_MODE_3 | SPI_CS_HIGH;
	// printk("esp FREQ = %d\n", esp.spi_dev->max_speed_hz);
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
 	msleep(ESP_RST_WAIT_MS);


	// SPI_CS_HIGH for esp8266
	esp.spi_dev->mode = SPI_MODE_3/* | SPI_CS_HIGH*/;
	ret = spi_setup(esp.spi_dev);
	// printk("esp FREQ 2 = %d\n", esp.spi_dev->max_speed_hz);
	if(ret != 0)
	{
		printk(KERN_ALERT "Error spi setup SPI_CS low.\n");
		return ret;
	}
	//////////////////////////////////////////////////////////

	if(prog)
		esp.state = IS_IN_PROG;
	else
		esp.state = IS_ON;


	return ret;
}

static int esp_off(void)
{
	gpio_set_value_cansleep(ESP_PWR_GPIO, 0);
	esp.state = IS_OFF;
	return 0;
}

ssize_t on_esp_cmd_received(struct file * file, const char __user * buf, size_t len, loff_t * ppos)
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


static void esp_big_worker(struct work_struct * work)
{
	int esp_ready;
	int esp_has_data;

	int spi_esc_tx_len;

	struct sk_buff * skb;

	// printk(KERN_INFO "esp_big_worker\n");

	esp_ready = gpio_get_value_cansleep(ESP_READY_GPIO);
	if(esp_ready == 0)
	{
		// printk(KERN_INFO "ESP_READY_GPIO = %d\n", esp_ready);
		return;
	}

	esp_has_data = gpio_get_value_cansleep(ESP_HAS_DATA_GPIO);
	if(esp_has_data)
	{
		int res = esp_spi_read_data();
		if(res)
		{
			printk(KERN_ALERT "esp_spi_read_data ERROR: %d\n", res);
			return;
		}
	}

	if(!skb_queue_empty(&esp.q_to_spi))
	{
		skb = skb_dequeue(&esp.q_to_spi);
		spi_esc_tx_len = slip_stuff(skb->data, esp.spi_tx_buf, skb->len);
		esp_spi_write_data(esp.spi_tx_buf, spi_esc_tx_len);
	}
}



static int esp_gpio_init(void)
{
	int res = 0;
	// SPI data GPIOs
	if(gpio_is_valid(ESP_READY_GPIO))
	{
		printk(KERN_INFO "esp_custom: ESP_READY_GPIO %d\n", ESP_READY_GPIO);

		res = gpio_request(ESP_READY_GPIO, "ESP_READY");
		if(res < 0) return res;
		res = gpio_direction_input(ESP_READY_GPIO);
		if(res < 0) return res;
	}
	else
	{
		printk(KERN_ALERT "esp_custom: invalid GPIO%d\n", ESP_READY_GPIO);
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


	// Interrupt setup for RX data available
	esp.data_gpio_irq = gpio_to_irq(ESP_HAS_DATA_GPIO);
	if(esp.data_gpio_irq < 0)
	{
		printk(KERN_ALERT "data_gpio_irq: %d\n", esp.data_gpio_irq);
		return esp.data_gpio_irq;
	}
	printk(KERN_INFO "data_gpio_irq: %d\n", esp.data_gpio_irq);
	// Interrupt setup for change ready status
	esp.ready_gpio_irq = gpio_to_irq(ESP_READY_GPIO);
	if(esp.ready_gpio_irq < 0)
	{
		printk(KERN_ALERT "ready_gpio_irq: %d\n", esp.ready_gpio_irq);
		return esp.ready_gpio_irq;
	}
	printk(KERN_INFO "data_gpio_irq: %d\n", esp.ready_gpio_irq);

	esp.data_irq_captured = false;
	esp.ready_irq_captured = false;

	return res;
}

static int esp_control_init(void)
{
	int res;
	esp.ctrl_fops.owner 	= THIS_MODULE;
	esp.ctrl_fops.write		= on_esp_cmd_received;
	esp.ctrl_fops.llseek 	= no_llseek;
	esp.ctrl_dev.minor 		= MISC_DYNAMIC_MINOR;
	esp.ctrl_dev.name 		= "esp8266_ctrl";
	esp.ctrl_dev.fops 		= &(esp.ctrl_fops);

	res = misc_register(&(esp.ctrl_dev));
	if(res)
	{
		printk(KERN_ALERT "misc_register: %d\n", res); 
		return res; 
	}
	return 0;
}

static int esp_spi_init(void)
{
	esp.chip.max_speed_hz	= ESP_SPI_MAX_SPEED;
	esp.chip.bus_num 		= ESP_SPI_BUS_NUM;
	esp.chip.chip_select 	= ESP_SPI_DEV_NUM;
	esp.chip.mode 			= SPI_MODE_3;	// don't forget to make ... | SPI_CS_HIGH after start

	esp.master = spi_busnum_to_master(esp.chip.bus_num);
	if(!(esp.master))
	{
		printk(KERN_ALERT "MASTER not found.\n");
		return -ENODEV;
	}

	esp.spi_dev = spi_new_device(esp.master, &(esp.chip));
	if(!(esp.spi_dev)) 
	{
		printk(KERN_ALERT "FAILED to create slave(1).\n");
		return -ENODEV;
	}

	esp.spi_dev->bits_per_word = 8;
	spi_setup(esp.spi_dev);

	return 0;
}

static int esp_wq_init(void)
{
	// skb queue
	skb_queue_head_init(&esp.q_to_spi);

	// workqueue
	esp.wq = alloc_workqueue("esp_wq", WQ_FREEZABLE | WQ_MEM_RECLAIM, 0);
	if (!(esp.wq))
	{
		return -ENOMEM;
	}
	INIT_WORK(&(esp.work), esp_big_worker);

	return 0;
}


static int esp_init(void)
{
	int res = 0;

	esp.state = IS_OFF;
	esp.data_gpio_irq = 0;
	esp.ready_gpio_irq = 0;
	esp.slip_context.maxlen = ESP_SPI_MAX_PACK_SIZE;
	esp.slip_context.output = esp.spi_rx_buf;

	mutex_init(&(esp.lock));

	res = esp_gpio_init();
	if(res)
	{
		printk(KERN_ALERT "esp_gpio_init: %d\n", res);
		return res;
	}

	res = esp_control_init();
	if(res)
	{
		printk(KERN_ALERT "esp_control_init: %d\n", res);
		return res;
	}

	res = esp_spi_init();
	if(res)
	{
		printk(KERN_ALERT "esp_spi_init: %d\n", res);
		return res;
	}

	res = esp_wq_init();
	if(res)
	{
		printk(KERN_ALERT "esp_spi_init: %d\n", res);
		return res;
	}

	res = esp_net_init();
	if(res)
	{
		printk(KERN_ALERT "esp_net_init: %d\n", res);
		return res;
	}

	return 0;
}

static void esp_deinit(void)
{
	spi_unregister_device(esp.spi_dev);
	misc_deregister(&(esp.ctrl_dev));

	esp_net_close(wifi_dev);
	unregister_netdev(wifi_dev);
	free_netdev(wifi_dev);

	esp_net_close(mesh_dev);
	unregister_netdev(mesh_dev);
	free_netdev(mesh_dev);

	destroy_workqueue(esp.wq);

	esp_off();
	// if(esp.data_gpio_irq > 0) free_irq(esp.data_gpio_irq, NULL);
	// if(esp.ready_gpio_irq > 0) free_irq(esp.ready_gpio_irq, NULL);
	gpio_free(ESP_PROG_GPIO);
	gpio_free(ESP_PWR_GPIO);
	gpio_free(ESP_READY_GPIO);
	gpio_free(ESP_HAS_DATA_GPIO);

	printk(KERN_INFO "ESP8266 is free\n"); 
}


static int __init ktest_module_init(void)
{
	int res = 0;
	res = esp_init();
	if(res)
	{
		printk(KERN_ALERT "esp_init: %d\n", res);
		return res;
	}


	printk( "ESP8266 under control\n" ); 
	return res;
}

static void __exit ktest_module_exit(void)
{
	esp_deinit();
}

MODULE_LICENSE("GPL");
 
module_init(ktest_module_init);
module_exit(ktest_module_exit);
