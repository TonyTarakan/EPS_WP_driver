#include "esp8266.h"


static int __init ktest_module_init(void)
{
	int res = 0;
	res = esp_init();
	{
		printk(KERN_ALERT "esp_init: %d\n", res);
		return res;
	}


    printk( "ESP8266 under control\n" ); 
	return res;
}

static void __exit ktest_module_exit(void)
{
	esp_init();
	printk( "ESP8266 is free\n" ); 
}

MODULE_LICENSE("GPL");
 
module_init(ktest_module_init);
module_exit(ktest_module_exit);
