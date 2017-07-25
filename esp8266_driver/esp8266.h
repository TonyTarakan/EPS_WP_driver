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
#include <linux/mutex.h>
#include <linux/tty.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/if_arp.h>

#include <linux/etherdevice.h>

#include <linux/spi/spi.h>

#include <linux/sierra_bsudefs.h>
#include <../arch/arm/mach-msm/board-9615.h>

#include "slip.h"

//look at gpiolib.c by Sierra
#define ESP_PWR_GPIO		SWIMCU_GPIO_TO_SYS(5)	// ESP_GPIO_ON GPIO39 //78
#define ESP_PROG_GPIO		SWIMCU_GPIO_TO_SYS(4)	// ESP_GPIO_0  GPIO38
#define ESP_BUSY_GPIO		78						// ESP_GPIO_5  GPIO33
#define ESP_HAS_DATA_GPIO	30						// ESP_GPIO_4  GPIO32

#define ESP_SPI_BUS_NUM		0
#define ESP_SPI_DEV_NUM		0
#define ESP_SPI_MAX_SPEED	5000000

#define ESP_SPI_BUF_HEAD_SIZE	2
#define ESP_SPI_BUF_SIZE 		(ESP_SPI_BUF_HEAD_SIZE + 32) 
#define ESP_SPI_BUF_WORD_SIZE	((ESP_SPI_BUF_SIZE - ESP_SPI_BUF_HEAD_SIZE)/4)
#define ESP_SPI_MAX_PACK_SIZE	2048

#define MASTER_WRITE_DATA_TO_SLAVE_CMD		2
#define MASTER_READ_DATA_FROM_SLAVE_CMD		3
#define MASTER_WRITE_STATUS_TO_SLAVE_CMD	1
#define MASTER_READ_STATUS_FROM_SLAVE_CMD	4

#define ESP_CMD_ON			"on\n"
#define ESP_CMD_OFF			"off\n"
#define ESP_CMD_RESET		"rst\n"
#define ESP_CMD_PROG		"prog\n"
#define ESP_MAX_CMD_LEN		10
#define ESP_RST_WAIT_MS		3000

#define ESP_UART_DEV		"/dev/ttyHS0"
#define ESP_UART_MODE		0

#define ESP_BOOT_FLASH		0
#define ESP_BOOT_UART		1

typedef enum esp_state
{
	IS_ON,
	IS_OFF,
	IS_IN_PROG,
}esp_state;

/* Network device private data */
typedef struct
{
	struct net_device 	* net;
	struct sk_buff 		* skb;
}esp_net_priv_t;

typedef struct
{
	int state;
	struct mutex lock;

	// ESP switch control
	struct file_operations	ctrl_fops;
	struct miscdevice   	ctrl_dev;


	// ESP SPI
	struct spi_master 		* master;
	struct spi_board_info	chip;
	struct spi_device		* spi_dev;

	unsigned char spi_blk_tx_buf[ESP_SPI_BUF_SIZE];
	unsigned char spi_blk_rx_buf[ESP_SPI_BUF_SIZE];

	unsigned char spi_tx_buf[ESP_SPI_MAX_PACK_SIZE];
	unsigned char spi_rx_buf[ESP_SPI_MAX_PACK_SIZE];	// kolhoz or not kolhoz there is the point

	struct workqueue_struct	* wq;
	struct work_struct work;	// work to transmit or receive SPI data

	int data_gpio_irq;
	int busy_gpio_irq;
	//int timer_irq;

	// ESP network interfaces
	struct net_device_ops 	ndops;
	// Network queque to tx to SPI
	struct sk_buff_head q_to_spi;
	struct sk_buff * skb;
}esp_t;

// int esp_init(void);
// int esp_deinit(void);
//ssize_t on_esp_cmd_received(struct file * file, const char __user * buf, size_t len, loff_t * ppos);
