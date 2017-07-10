#include <linux/kernel.h>
#include <linux/delay.h>
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

#include <linux/spi/spi.h>

#include <linux/sierra_bsudefs.h>
#include <../arch/arm/mach-msm/board-9615.h>

#define ESP_PWR_GPIO	30						// look at gpiolib by Sierra //78		//32
#define ESP_PROG_GPIO	SWIMCU_GPIO_TO_SYS(4)	// look at gpiolib by Sierra

#define ESP_CMD_ON		"on\n"
#define ESP_CMD_OFF		"off\n"
#define ESP_CMD_RESET	"rst\n"
#define ESP_CMD_PROG	"prog\n"
#define ESP_MAX_CMD_LEN	10

#define ESP_UART_DEV	"/dev/ttyHS0"
#define ESP_UART_MODE	0

typedef struct
{
	struct spi_device * spi_dev;
}
esp_transfer_t;