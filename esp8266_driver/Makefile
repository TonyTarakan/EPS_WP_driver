obj-m+=esp8266.o
esp8266-y := esp8266_main.o slip.o crc.o


ifdef rpi
ccflags-y := -DRPI_BASE
KBUILD_DIR_RPI=../../rasp/linux
all:
	ARCH=arm CROSS_COMPILE=../../rasp/tools/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi- make -C $(KBUILD_DIR_RPI) M=$(PWD) modules
else 

ifdef sw
ccflags-y := -DSW_BASE
KBUILD_DIR_SW=~/WORK/AS100GW/tmptests/buildtest/test040717/yocto/build_bin/tmp/work/swi_mdm9x15-poky-linux-gnueabi/linux-yocto/3.14.29+gitAUTOINC+961ca71325_1b095d98a4-r0.1/linux-swi_mdm9x15-standard-build
export PATH:=$(PATH):/opt/swi/y17-ext/sysroots/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi
all:
	ARCH=arm CROSS_COMPILE=arm-poky-linux-gnueabi- make -C $(KBUILD_DIR_SW) M=$(PWD) modules
endif

endif

clean:
	rm -f *.mod.c *.o *.ko *.order *.symvers
