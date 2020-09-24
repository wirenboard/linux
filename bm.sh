#!/bin/bash

#Делает только modules. Дальше надо put-modules.sh

yell() { echo "$0: $*" >&2; }
die() { echo -e "\e[31m$*\e[0m"; exit 1; }
try() { "$@" || die "cannot $*"; }
set -v

export CROSS_COMPILE=arm-linux-gnueabihf-

try time make -j8 ARCH=arm LOCALVERSION=ivz modules


