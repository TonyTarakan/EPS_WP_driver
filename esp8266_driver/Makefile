# Kernel module build command for ARM targets is:
# make ARCH=arm CROSS_COMPILE=arm-poky-linux-gnueabi-
#
# List of object file(s) to be built

obj-m+=esp8266_main.o

#
# Point to the directory containing sources for the kernel running on target
# system. The kernel directory must have the configuration build step executed,
# i.e. it must contain the target system's .config file.
#
# Legato tools contain such a directory, point to it.

KBUILD_DIR=~/WORK/AS100GW/tmptests/buildtest/test040717/yocto/build_bin/tmp/work/swi_mdm9x15-poky-linux-gnueabi/linux-yocto/3.14.29+gitAUTOINC+961ca71325_ab5094eade-r0.1/linux-swi_mdm9x15-standard-build

export PATH:=$(PATH):/opt/swi/y17-ext/sysroots/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi
#
# Kernel module build dependency
all:
	ARCH=arm CROSS_COMPILE=arm-poky-linux-gnueabi- make -C $(KBUILD_DIR) M=$(PWD) modules
#
# Kernel module clean dependency
clean:
	make -C $(KBUILD_DIR) M=$(PWD) clean
