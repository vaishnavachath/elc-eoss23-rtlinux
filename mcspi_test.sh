#!/bin/bash

MCSPIFREQS=(24000000 48000000)
PACKETSIZE=(32 640 4096)
NUMTRANSACTIONS=100000

for frequency in ${MCSPIFREQS[@]}; do
	for packetsize in ${PACKETSIZE[@]}; do
		./spi_test -s $frequency -S $packetsize -n $NUMTRANSACTIONS
	done
done

