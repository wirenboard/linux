#!/bin/sh

#Отправка ядра, дерева и модулей на плату. Каталоги /boot/$KERN и /lib/modules/$KERN соотв. 
#Модули берутся из каталога _mod - make modules_install должен быть уже сделан.
#KERN получаем из include/generated/uts_release.h после успешной сборки.

DTB1=imx6ul-wirenboard660.dtb
DTB2=imx28-wirenboard58.dtb
TAR=mod.tgz
TARPATH=_mod/lib/modules
TARFULL=$TARPATH/$TAR

yell() { echo "$0: $*" >&2; }
die() { echo "\e[31m$*\e[0m"; exit 1; }
try() { "$@" || die "cannot $*"; }

#Получить строку вида 4.9.22ivz после сборки ядра. 
KERN=$(cat "include/generated/utsrelease.h" | awk '{print $3}' | sed 's/\"//g' )
[ -z "$KERN" ] && die "?Cant get kernel version: KERN=$KERN"

#Создаем архив с модулями.
#Чтобы не было ошибки file changed as we read it, можно сделать touch + --exclude (или не делать - не мешает).
touch $TARFULL
tar -czf $TARFULL -C $TARPATH --exclude=build --exclude=source --exclude=./$TAR .

#Ядро, дерево устройств, архив с модулями отправляем на плату.
try ./sftp2.sh "\
mkdir /boot/$KERN\n
put arch/arm/boot/zImage /boot/$KERN/zImage \n
put arch/arm/boot/dts/$DTB1 /boot/$KERN/$DTB1 \n
put arch/arm/boot/dts/$DTB2 /boot/$KERN/$DTB2 \n
put $TARFULL /lib/modules/$TAR \n
"
#Распаковываем архив с модулями.
try ./plink2.sh "rm -rf /lib/modules/$KERN; tar -xf /lib/modules/$TAR -C /lib/modules/"
