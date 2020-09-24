#!/bin/bash

yell() { echo "$0: $*" >&2; }
die() { echo -e "\e[31m$*\e[0m"; exit 1; }
try() { "$@" || die "cannot $*"; }
set -x

try make -j8 ARCH=arm LOCALVERSION=ivz savedefconfig
mv defconfig arch/arm/configs/_defconfig