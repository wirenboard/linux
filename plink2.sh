#!/bin/sh

#Скрипт получает команду в кавычках в ком. строке - 1 параметр.
#Если команд несколько, разделять их ; 

set -x

[ -z "$BKEY" ] && BKEY="/home/ivan/my/sshkeys/zynq_root/id_rsa.ppk"

plink wb6 -l root -i $BKEY -batch $1

