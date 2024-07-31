#!/bin/sh

INITRAMFS_DIR="/usr/src/wb-initramfs/$KERNEL_FLAVOUR"

setup_kernel_vars() {
	case "$KERNEL_FLAVOUR" in
		wb8)
			DEBARCH=arm64
			KERNEL_DEFCONFIG="defconfig wb8.config"
			KDEB_WBDESC="Wiren Board 8"
			;;
		*)
			echo "Unsupported KERNEL_FLAVOUR, please specify one of: wb8"
			return 1
	esac

	case "$KERNEL_FLAVOUR" in
		*-bootlet)
			if [ ! -d "$INITRAMFS_DIR" ]; then
				echo "Please install proper wb-bootlet-initramfs-X package"
				return 1
			fi

			INITRAMFS_VERSION="$(cat "$INITRAMFS_DIR/version")"
			;;
	esac


	case "$DEBARCH" in
		armel)
			CROSS_COMPILE=arm-linux-gnueabi-
			KERNEL_ARCH=arm
			KERNEL_IMAGE=zImage
			;;
		armhf)
			CROSS_COMPILE=arm-linux-gnueabihf-
			KERNEL_ARCH=arm
			KERNEL_IMAGE=zImage
			;;
		arm64)
			CROSS_COMPILE=aarch64-linux-gnu-
			KERNEL_ARCH=arm64
			KERNEL_IMAGE=Image.gz
			;;
		*)
			echo "Unsupported DEBARCH, please specify one of: armel, armhf, arm64"
			return 1
			;;
	esac
	export DEBARCH KERNEL_ARCH KERNEL_IMAGE KERNEL_DEFCONFIG KDEB_WBFLAVOUR_DESC CROSS_COMPILE BOOTLET_DTB INITRAMFS_VERSION KDEB_WBDESC BOOTLET_DEPS PROVIDES_BOOTLET_FOR_FITS
}
