#!/bin/bash -ex

BUILDDIR=.
TARGETDIR=root@10.42.84.2:mnt

ssh root@10.42.84.2 "mount /dev/mmcblk1p5 /root/mnt || true"
trap "ssh root@10.42.84.2 \"bash -c 'umount /dev/mmcblk1p5 ; sync; sync'\"" EXIT

if [ -z "$1" ]; then scp -C "$BUILDDIR/arch/arm64/boot/Image" "$TARGETDIR/boot/Image"; fi
scp -C "$BUILDDIR/arch/arm64/boot/dts/allwinner/sun50i-h616-wirenboard8xx.dtb" "$TARGETDIR/boot/boot.dtb"
