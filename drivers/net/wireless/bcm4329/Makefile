# bcm4329
DHDCFLAGS = -DLINUX -DBCMDRIVER -DBCMDONGLEHOST -DDHDTHREAD -DBCMWPA2         \
	-DUNRELEASEDCHIP -Dlinux -DDHD_SDALIGN=64 -DMAX_HDR_READ=64           \
	-DDHD_FIRSTREAD=64 -DDHD_GPL -DDHD_SCHED -DBDC -DTOE -DDHD_BCMEVENTS  \
	-DSHOW_EVENTS -DBCMSDIO -DDHD_GPL -DBCMLXSDMMC -DBCMPLATFORM_BUS      \
	-Wall -Wstrict-prototypes -DOOB_INTR_ONLY -DCUSTOMER_HW2      		  \
	-DDHD_USE_STATIC_BUF -DMMC_SDIO_ABORT -DDHD_DEBUG_TRAP -DSOFTAP       \
	-DEMBEDDED_PLATFORM -DARP_OFFLOAD_SUPPORT -DPKT_FILTER_SUPPORT        \
	-DGET_CUSTOM_MAC_ENABLE -DSET_RANDOM_MAC_SOFTAP -DCSCAN -DHW_OOB      \
	-DKEEP_ALIVE -DPNO_SUPPORT                                            \
	-Idrivers/net/wireless/bcm4329 -Idrivers/net/wireless/bcm4329/include

DHDOFILES = \
	src/dhd/sys/dhd_linux.o \
	src/shared/linux_osl.o \
	src/shared/bcmutils.o \
	src/dhd/sys/dhd_common.o \
	src/dhd/sys/dhd_custom_gpio.o \
	src/wl/sys/wl_iw.o \
	src/shared/siutils.o \
	src/shared/sbutils.o \
	src/shared/aiutils.o \
	src/shared/hndpmu.o \
	src/shared/bcmwifi.o \
	src/dhd/sys/dhd_sdio.o \
	src/dhd/sys/dhd_linux_sched.o \
	src/dhd/sys/dhd_cdc.o \
	src/bcmsdio/sys/bcmsdh_sdmmc.o \
	src/bcmsdio/sys/bcmsdh.o \
	src/bcmsdio/sys/bcmsdh_linux.o \
	src/bcmsdio/sys/bcmsdh_sdmmc_linux.o \

obj-$(CONFIG_BCM4329) += wireless.o
wireless-objs += $(DHDOFILES)
EXTRA_CFLAGS = $(DHDCFLAGS)
EXTRA_LDFLAGS += --strip-debug
