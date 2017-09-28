#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/syscalls.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/tty.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/socket.h>
#include <linux/timer.h>
#include <linux/wait.h>

#include <linux/etherdevice.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#ifdef SW_BASE
#include <linux/sierra_bsudefs.h>
#include <../arch/arm/mach-msm/board-9615.h>
#endif

#include "slip.h"
#include "crc.h"


#ifdef SW_BASE
//look at gpiolib.c by Sierra
#define ESP_PWR_GPIO		SWIMCU_GPIO_TO_SYS(5)	// ESP_GPIO_ON GPIO39 //78
#define ESP_PROG_GPIO		SWIMCU_GPIO_TO_SYS(4)	// ESP_GPIO_0  GPIO38
#define ESP_READY_GPIO		78						// ESP_GPIO_5  GPIO33
#define ESP_HAS_DATA_GPIO	30						// ESP_GPIO_4  GPIO32
#elif defined RPI_BASE
#define ESP_PWR_GPIO		17	// ESP_GPIO_ON	// RASPBERRY_PI_3 GPIO17(11)
#define ESP_PROG_GPIO		27	// ESP_GPIO_0	// RASPBERRY_PI_3 GPIO27(13)
#define ESP_READY_GPIO		3	// ESP_GPIO_5	// RASPBERRY_PI_3 GPIO03(03)
#define ESP_HAS_DATA_GPIO	2	// ESP_GPIO_4	// RASPBERRY_PI_3 GPIO02(05)
#endif

#define ESP_SPI_BUS_NUM		0
#define ESP_SPI_DEV_NUM		0
#define ESP_SPI_MAX_SPEED	4800000

#define ESP_SPI_BUF_HEAD_SIZE	2
#define ESP_SPI_BUF_DATA_SIZE	32
#define ESP_SPI_BUF_SIZE 		(ESP_SPI_BUF_HEAD_SIZE + ESP_SPI_BUF_DATA_SIZE) 
#define ESP_SPI_BUF_WORD_SIZE	((ESP_SPI_BUF_SIZE - ESP_SPI_BUF_HEAD_SIZE)/4)
#define ESP_SPI_MAX_PACK_SIZE	3002 // 1500 * 2 + 2 for slip

#define MASTER_WRITE_DATA_TO_SLAVE_CMD		2
#define MASTER_READ_DATA_FROM_SLAVE_CMD		3
#define MASTER_WRITE_STATUS_TO_SLAVE_CMD	1
#define MASTER_READ_STATUS_FROM_SLAVE_CMD	4

#define ESP_CMD_ON			"on\n"
#define ESP_CMD_OFF			"off\n"
#define ESP_CMD_RESET		"rst\n"
#define ESP_CMD_PROG		"prog\n"
#define ESP_MAX_CMD_LEN		10
#define ESP_RST_WAIT_MS		1000 // maybe reduce?
#define ESP_RST_WAIT1_MS	200  // maybe reduce?


#define ESP_UART_MODE		0
#ifdef SW_BASE
#define ESP_UART_DEV		"/dev/ttyHS0"

#define ESP_BOOT_FLASH		0
#define ESP_BOOT_UART		1
#elif defined RPI_BASE
#define ESP_UART_DEV		"/dev/ttyAMA0"	// RASPBERRY_PI_3 CONSOLE NEED TO BE SWITCHED OFF

#define ESP_BOOT_FLASH		1
#define ESP_BOOT_UART		0
#endif

#define ESP_MAX_MESH_NODES	512

#define ESP_TIMER_PERIOD_MS	60000

#define ESP_CONFIG_LENGTH	1024

/* Network device private data */
typedef struct
{
	struct net_device 	* net;
	struct sk_buff 		* skb;
}esp_net_priv_t;

typedef enum pack_type
{
	GATEWAY_ROUTE_INFO = 0,
	GATEWAY_CONFIG_WRITE = 1,
	GATEWAY_CONFIG_READ = 2
}packet_type_t;

typedef enum esp_mode
{
	MODE_ON,
	MODE_OFF,
	MODE_CONF,
	MODE_PROG
}esp_mode;

typedef struct
{
	int mode;

	// ESP switch control
	struct file_operations	ctrl_fops;
	struct miscdevice   	ctrl_dev;

	// ESP connected IP's information
	struct file_operations	ipinfo_fops;
	struct miscdevice   	ipinfo_dev;
	uint32_t 				ip_info_array[ESP_MAX_MESH_NODES];
	uint16_t 				ip_info_nodes_count;

	// ESP ginfig interface
	struct file_operations	config_fops;
	struct miscdevice   	config_dev;
	unsigned char 			config_buf[ESP_CONFIG_LENGTH];
	bool 					config_data_avail;


	// ESP SPI
	struct spi_master * 	master;
	struct spi_board_info	chip;
	struct spi_device * 	spi_dev;

	unsigned char spi_blk_tx_buf[ESP_SPI_BUF_SIZE];
	unsigned char spi_blk_rx_buf[ESP_SPI_BUF_SIZE];

	unsigned char spi_tx_buf[ESP_SPI_MAX_PACK_SIZE];

	slip_t slip_context;	
	unsigned char spi_rx_buf[ESP_SPI_MAX_PACK_SIZE];	// kolhoz or not kolhoz there is the point

	struct workqueue_struct	* wq;
	struct work_struct 		work;	// work to transmit or receive SPI data

	int 	data_gpio_irq;
	bool 	data_irq_captured;
	int 	ready_gpio_irq;
	bool 	ready_irq_captured;

	struct timer_list 		hang_timer;

	// ESP network interfaces
	struct net_device_ops 	ndops;
	// Network queque to tx to SPI
	struct sk_buff_head 	q_to_spi;
	struct sk_buff * 		skb;
}esp_t;
