#
# Makefile for PPS clients.
#

obj-$(CONFIG_PPS_CLIENT_KTIMER)	+= pps-ktimer.o
obj-$(CONFIG_PPS_CLIENT_LDISC)	+= pps-ldisc.o
obj-$(CONFIG_PPS_CLIENT_PARPORT) += pps_parport.o
obj-$(CONFIG_PPS_CLIENT_GPIO)	+= pps-gpio.o
obj-$(CONFIG_PPS_CLIENT_IMX_GPT)	+= pps-imx-gpt.o

ccflags-$(CONFIG_PPS_DEBUG) := -DDEBUG
