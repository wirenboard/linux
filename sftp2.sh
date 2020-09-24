#!/bin/sh

#Скрипт получает команду в кавычках в ком. строке - 1 параметр.
#Если команд несколько, разделять их \n  Новая строка с \ не дает \n в команде. Можно так:
# ./sftp2.sh "
# put a.a \n
# "

set -x

TMPF=_sftp2.cmd

[ -z "$BKEY" ] && BKEY="/home/ivan/my/sshkeys/zynq_root/id_rsa.ppk"

echo $1 > $TMPF

psftp wb6 -l root -i $BKEY -be -b $TMPF

rm $TMPF
