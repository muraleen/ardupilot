#!/bin/sh

nice fgfs \
	--aircraft=lynxUAS \
	--airport=YKRY \
	--fg-root='/home/narendran/fgfs/fgdata'
    --native-fdm=socket,in,10,,5503,udp \
    --fdm=external \
    --control=mouse \
    --wind=0@0 \
    $*
