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

#define ESP_PWR_GPIO		30						// look at gpiolib by Sierra //78		//32
#define ESP_PROG_GPIO		SWIMCU_GPIO_TO_SYS(4)	//GPIO38
#define ESP_BUSY_GPIO		SWIMCU_GPIO_TO_SYS(5)	//GPIO39
#define ESP_HAS_DATA_GPIO	SWIMCU_GPIO_TO_SYS(6)	//GPIO40

#define ESP_CMD_ON			"on\n"
#define ESP_CMD_OFF			"off\n"
#define ESP_CMD_RESET		"rst\n"
#define ESP_CMD_PROG		"prog\n"
#define ESP_MAX_CMD_LEN		10

#define ESP_UART_DEV		"/dev/ttyHS0"
#define ESP_UART_MODE		0

#define ESP_BOOT_FLASH		0
#define ESP_BOOT_UART		1

/* Network device private data */
typedef struct
{
	struct net_device * net;
	struct spi_device * spi;

	struct mutex spi_lock;


	struct sk_buff * tx_skb;

	struct workqueue_struct	* wq;
	struct work_struct 		tx_work;
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
	// ESP network interfaces
	struct net_device_ops 	wifi_ndops;
	struct net_device		* wifi_dev;
	struct net_device_ops 	mesh_ndops;
	struct net_device		* mesh_dev;
}esp_t;

