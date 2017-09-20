#
# Copyright (C) 2008 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=oid_sn9p702
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/oid_sn9p702
  SUBMENU:=Other modules
  TITLE:=OID SN9P702.....................................
  FILES:=$(PKG_BUILD_DIR)/oid_sn9p702.ko
  AUTOLOAD:=$(call AutoLoad,30,oid_sn9p702,1)
  KCONFIG:=
endef

define KernelPackage/oid_sn9p702/description
 ------------------  OID SN9P702 Driver -----------------.
endef

EXTRA_KCONFIG:= \
	CONFIG_OID_SN9P702=m

EXTRA_CFLAGS:= \
	$(patsubst CONFIG_%, -DCONFIG_%=1, $(patsubst %=m,%,$(filter %=m,$(EXTRA_KCONFIG)))) \
	$(patsubst CONFIG_%, -DCONFIG_%=1, $(patsubst %=y,%,$(filter %=y,$(EXTRA_KCONFIG)))) \

MAKE_OPTS:= \
	ARCH="$(LINUX_KARCH)" \
	CROSS_COMPILE="$(TARGET_CROSS)" \
	SUBDIRS="$(PKG_BUILD_DIR)" \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	$(EXTRA_KCONFIG)

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		$(MAKE_OPTS) \
		modules
endef

$(eval $(call KernelPackage,oid_sn9p702))
