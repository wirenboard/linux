#!/bin/sh

#Выполняет команды sftp из файла, который идет параметром.

if [ -z "$BKEY" ]; then BKEY="/home/ivan/my/sshkeys/zynq_root/id_rsa.ppk"; fi
if ping -c 1 wb6 &> /dev/null; then
  psftp wb6 -l root -i $BKEY -be -b $@

else
  echo "? ping fail."
fi
