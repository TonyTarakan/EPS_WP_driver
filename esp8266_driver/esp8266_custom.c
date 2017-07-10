#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define ESP_CMD_ON		"on\n"
#define ESP_CMD_OFF		"off\n"
#define ESP_CMD_RESET	"rst\n"
#define ESP_CMD_PROG	"prog\n"
#define ESP_MAX_CMD_LEN	10

static ssize_t esp_cmd_write(struct file *file, const char __user *buf, size_t len, loff_t *ppos)
{
	ssize_t ret;
	void * cmd_buf;

	if(len > ESP_MAX_CMD_LEN)
	{
		pr_info("Too long ESP command\n");
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
    	pr_info("ESP ON\n");
	}
    else if(memcmp(buf, ESP_CMD_OFF, len) == 0)
    {
		pr_info("ESP OFF\n");
    }
	else if(memcmp(buf, ESP_CMD_RESET, len) == 0)
	{
		pr_info("ESP RESET\n");
	}
		else if(memcmp(buf, ESP_CMD_PROG, len) == 0)
	{
		pr_info("ESP switched to programming mode\n");
	}
	else
	{
		pr_info("Unknown ESP command\n");
	}

	kfree(cmd_buf);
    return len; 	// почему если ноль, то зацикливается?
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
	misc_deregister( &esp_ctrl ); 
	printk( "ESP8266 is free\n" ); 
}

MODULE_LICENSE("GPL");
 
module_init(ktest_module_init);
module_exit(ktest_module_exit);
