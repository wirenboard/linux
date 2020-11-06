#!/bin/sh

#Скрипт получает команду в кавычках в ком. строке - 1 параметр.
#Если команд несколько, разделять их ; 
#Переменная TARGET задает имя хоста.
#Переменная TPORT="-P 2222" задает имя порта, или пусто, если порт по умолчанию.

set -x

[ -z "$TARGET"] && TARGET="wb5"
[ -z "$BKEY" ] && BKEY="/home/ivan/my/sshkeys/zynq_root/id_rsa.ppk"

plink $TARGET $TPORT -l root -i $BKEY -batch $1

