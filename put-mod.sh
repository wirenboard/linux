#!/bin/sh

#Отправляет модуль w1 на плату в каталог с модулями и перегружает их.

yell() { echo "$0: $*" >&2; }
die() { echo "\e[31m$*\e[0m"; exit 1; }
try() { "$@" || die "cannot $*"; }

#KERN получаем из include/generated/uts_release.h после успешной сборки.

#Получить строку вида 4.9.22ivz после сборки ядра.
KERN=$(cat "include/generated/utsrelease.h" | awk '{print $3}' | sed 's/\"//g' )
[ -z "$KERN" ] && die "?Cant get kernel version: KERN=$KERN"

set -x

try ./sftp2.sh "
put drivers/w1/slaves/w1_therm.ko /lib/modules/$KERN/kernel/drivers/w1/slaves/w1_therm.ko \n
put drivers/w1/wire.ko /lib/modules/$KERN/kernel/drivers/w1/wire.ko \n
put drivers/w1/masters/w1-gpio.ko /lib/modules/$KERN/kernel/drivers/w1/masters/w1-gpio.ko
"

#Перегрузка временно убрана.
#try ./plink2.sh "~/w1reload.sh"

