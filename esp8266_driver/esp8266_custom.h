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

#include <linux/spi/spi.h>

#include <linux/sierra_bsudefs.h>
#include <../arch/arm/mach-msm/board-9615.h>

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

#define MASTER_WRITE_DATA_TO_SLAVE_CMD                      2
#define MASTER_READ_DATA_FROM_SLAVE_CMD                     3
#define MASTER_WRITE_STATUS_TO_SLAVE_CMD                    1
#define MASTER_READ_STATUS_FROM_SLAVE_CMD                   4

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



/* Network device private data */
typedef struct
{
	struct net_device 		* net;
	struct spi_device 		* spi;
	struct spi_master 		* master;
	struct spi_board_info	chip;

	
	unsigned char spi_blk_tx_buf[ESP_SPI_BUF_SIZE];
	unsigned char spi_blk_rx_buf[ESP_SPI_BUF_SIZE];

	unsigned char spi_tx_buf[ESP_SPI_MAX_PACK_SIZE];
	unsigned char spi_rx_buf[ESP_SPI_MAX_PACK_SIZE];

	struct sk_buff * tx_skb;

	struct workqueue_struct	* wq;
	struct work_struct 		tx_work;	// work to transmit data to SPI
	struct work_struct 		rx_work;	// work to receive data from SPI
}esp_net_priv_t;

typedef struct
{
	// ESP switch control
	struct file_operations	ctrl_fops;
	struct miscdevice   	ctrl_dev;
	// ESP SPI
	struct spi_master 		* master;
	struct spi_board_info	chip;
	struct spi_device		* spi_dev;

	int rx_gpio_irq;
	// ESP network interfaces
	struct net_device_ops 	wifi_ndops;
	struct net_device		* wifi_dev;
	struct net_device_ops 	mesh_ndops;
	struct net_device		* mesh_dev;
}esp_t;

