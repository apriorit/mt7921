# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_MT76_CORE) += mt76.o
obj-$(CONFIG_MT76_USB) += mt76-usb.o
obj-$(CONFIG_MT76_SDIO) += mt76-sdio.o
obj-$(CONFIG_MT76x02_LIB) += mt76x02-lib.o
obj-$(CONFIG_MT76x02_USB) += mt76x02-usb.o
obj-$(CONFIG_MT76_CONNAC_LIB) += mt76-connac-lib.o
obj-$(CONFIG_MT792x_LIB) += mt792x-lib.o
obj-$(CONFIG_MT792x_USB) += mt792x-usb.o

mt76-y := \
	util.o dma.o mac80211.o eeprom.o \
	tx.o agg-rx.o mcu.o

mt76-$(CONFIG_NL80211_TESTMODE) += testmode.o

mt76-usb-y := usb.o
mt76-sdio-y := sdio.o sdio_txrx.o

CFLAGS_trace.o := -I$(src)
CFLAGS_usb_trace.o := -I$(src)
CFLAGS_mt76x02_trace.o := -I$(src)
CFLAGS_mt792x_trace.o := -I$(src)

mt76x02-lib-y := mt76x02_util.o mt76x02_mac.o mt76x02_mcu.o \
		 mt76x02_eeprom.o mt76x02_phy.o mt76x02_mmio.o \
		 mt76x02_txrx.o mt76x02_trace.o mt76x02_debugfs.o \
		 mt76x02_dfs.o mt76x02_beacon.o

mt76x02-usb-y := mt76x02_usb_mcu.o mt76x02_usb_core.o

mt76-connac-lib-y := mt76_connac_mcu.o mt76_connac_mac.o

mt792x-lib-y := mt792x_core.o mt792x_mac.o 

mt792x-lib-$(CONFIG_ACPI) += mt792x_acpi_sar.o
mt792x-usb-y := mt792x_usb.o

obj-$(CONFIG_MT76x0_COMMON) += mt76x0/
obj-$(CONFIG_MT76x2_COMMON) += mt76x2/
obj-$(CONFIG_MT7603E) += mt7603/
obj-$(CONFIG_MT7615_COMMON) += mt7615/
obj-$(CONFIG_MT7915E) += mt7915/
obj-$(CONFIG_MT7921_COMMON) += mt7921/
obj-$(CONFIG_MT7996E) += mt7996/
obj-$(CONFIG_MT7925_COMMON) += mt7925/
