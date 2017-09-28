# Kernel module build command for ARM targets is:
# make ARCH=arm CROSS_COMPILE=arm-poky-linux-gnueabi-
#
# List of object file(s) to be built

obj-m+=esp8266.o
esp8266-y := esp8266_main.o slip.o crc.o
#
# Point to the directory containing sources for the kernel running on target
# system. The kernel directory must have the configuration build step executed,
# i.e. it must contain the target system's .config file.
#
# Legato tools contain such a directory, point to it.

KBUILD_DIR=~/WORK/AS100GW/firmware/rasp/linux

#
# Kernel module build dependency
all:
	ARCH=arm CROSS_COMPILE=~/WORK/AS100GW/firmware/rasp/tools/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi- make -C $(KBUILD_DIR) M=$(PWD) modules
#
# Kernel module clean dependency
clean:
	make -C $(KBUILD_DIR) M=$(PWD) clean
