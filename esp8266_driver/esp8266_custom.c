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

static const struct file_operations esp_ctrl_fops = 
{
    .owner			= THIS_MODULE,
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
