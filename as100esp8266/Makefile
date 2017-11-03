#
# AS100 gateway kernel module for esp8266
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=as100esp8266
PKG_RELEASE:=1

PKG_MAINTAINER:=Vladimir Ovseychuk

include $(INCLUDE_DIR)/package.mk

define KernelPackage/as100esp8266
  SUBMENU:=Other modules
  TITLE:=ESP8266 spi device support on AS100GW
  DEPENDS:=
  FILES:=$(PKG_BUILD_DIR)/as100esp8266.ko
endef

define KernelPackage/as100esp8266/description
  Kernel module for ESP8266 spi device support on AS100GW
endef

MAKE_OPTS:= \
	ARCH="$(LINUX_KARCH)" \
	CROSS_COMPILE="$(TARGET_CROSS)" \
	SUBDIRS="$(PKG_BUILD_DIR)"

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		$(MAKE_OPTS) \
		modules
endef

$(eval $(call KernelPackage,as100esp8266))
