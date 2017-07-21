#include "esp8266.h"

struct net_device * wifi_dev;
struct net_device * mesh_dev;

static void esp_net_setup(struct net_device * netdev) 
{
	netdev->netdev_ops = &(esp.ndops);
	ether_setup(netdev);
} 
static void esp_net_mac_setup(struct net_device * netdev, unsigned char devaddr, int length) 
{
	memcpy(netdev->dev_addr, devaddr, length)
}

static int esp_net_start_tx(struct sk_buff * skb, struct net_device * netdev)
{ 
	esp_net_priv_t * esp_priv = netdev_priv(netdev);

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
	printk(KERN_INFO "esp_net_open\n");

	// GPIO irq registration
	res = request_irq(esp.rx_gpio_irq, esp_rx_gpio_handler, IRQF_TRIGGER_RISING, "esp_rx_gpio", esp_priv);
	if(res < 0)
	{
		printk(KERN_ALERT "request_irq: %d\n", res);
		return res;
	}

	netif_wake_queue(netdev); 

	return 0; 
}

static int esp_net_close(struct net_device * netdev)
{ 
	printk(KERN_INFO "esp_net_close\n");
	return 0; 
}


int esp_net_init(void)
{
	esp.ndops.ndo_open 			= esp_net_open;
	esp.ndops.ndo_stop 			= esp_net_close;
	esp.ndops.ndo_start_xmit 	= esp_net_start_tx;

	wifi_dev = alloc_netdev(sizeof(esp_net_priv_t), "espwifi%d", esp_net_setup);

	// !!! hardcode
	unsigned char devaddr_wifi[6] = {0xE5, 0x82, 0x66, 0x11, 0x11, 0xF1};
	esp_net_mac_setup(wifi_dev, devaddr_wifi, 6);

	res = register_netdev(wifi_dev);
	if(res != 0)
	{
		printk(KERN_INFO "Failed to register wifi interface\n" ); 
		free_netdev(wifi_dev);
		return res; 
	}

	mesh_dev = alloc_netdev(sizeof(esp_net_priv_t), "espmesh%d", esp_net_setup);

	// !!! hardcode
	unsigned char devaddr_wifi[6] = {0xE5, 0x82, 0x66, 0x11, 0x1E, 0x5F};
	esp_net_mac_setup(wifi_dev, devaddr_wifi, 6);

	res = register_netdev(mesh_dev);
	if(res != 0)
	{
		printk(KERN_INFO "Failed to register mesh interface\n" ); 
		free_netdev(mesh_dev);
		return res; 
	}

}