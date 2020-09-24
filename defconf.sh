#!/bin/bash

yell() { echo "$0: $*" >&2; }
die() { echo -e "\e[31m$*\e[0m"; exit 1; }
try() { "$@" || die "cannot $*"; }
set -x

if [ $1 ]; then KERNEL_DEFCONFIG=$1; else KERNEL_DEFCONFIG=imx6_wirenboard_defconfig; fi

try make -j8 ARCH=arm LOCALVERSION=ivz $KERNEL_DEFCONFIG
