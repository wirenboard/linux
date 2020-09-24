#!/bin/bash

export CROSS_COMPILE=arm-linux-gnueabihf-

yell() { echo "$0: $*" >&2; }
die() { echo -e "\e[31m$*\e[0m"; exit 1; }
try() { "$@" || die "cannot $*"; }
set -x

#В конце WB6, чтобы осталась нормальная сборка в итоге.
for KERNEL_DEFCONFIG in mxs_wirenboard_defconfig imx6_wirenboard_defconfig; do 
  [ "$KERNEL_DEFCONFIG" = "mxs_wirenboard_defconfig" ] && DEBARCH=armel
  [ "$KERNEL_DEFCONFIG" = "imx6_wirenboard_defconfig" ] && DEBARCH=armhf
  echo $KERNEL_DEFCONFIG $DEBARCH
  try make -j8 ARCH=arm LOCALVERSION=ivz $KERNEL_DEFCONFIG
  try time make -j8 ARCH=arm KBUILD_DEBARCH=$DEBARCH LOCALVERSION=ivz zImage modules dtbs
done


