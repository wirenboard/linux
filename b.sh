#!/bin/bash

yell() { echo "$0: $*" >&2; }
die() { echo -e "\e[31m$*\e[0m"; exit 1; }
try() { "$@" || die "cannot $*"; }
set -v

INSTM=_mod
export CROSS_COMPILE=arm-linux-gnueabihf-

try time make -j8 ARCH=arm LOCALVERSION=ivz zImage modules
try time make -j8 ARCH=arm LOCALVERSION=ivz dtbs

rm -rf $INSTM/
try make ARCH=arm INSTALL_MOD_PATH=$INSTM modules_install



