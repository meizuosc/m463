#!/bin/bash

_p="m71"
[ ! -z "$1" ] && [ "$1" = "m71" -o "$1" = "m79" ] && _p="$1"

echo Building $_p kernel ...

# add cross-compiler toolchina path
export PATH=/meizuosc/toolchain/gcc-linaro-arm-linux-gnueabi-4.6.3/bin:$PATH

CUR_MODULE=kernel make -f mediatek/build/makemtk.mk check-dep CMD_ARGU="ENABLE_TEE=TRUE PROJECT=$_p FLAVOR= MKTOPDIR=$(pwd) PRODUCT=" ENABLE_TEE=TRUE PROJECT=$_p FLAVOR= MKTOPDIR=$(pwd) PRODUCT= CROSS_COMPILE=arm-linux-gnueabi-

[ $? -ne 0 ] && exit 1

echo Generating out/target/product/${_p}/kernel_${_p}.bin ...

make -f mediatek/build/makemtk.mk CMD_ARGU="ENABLE_TEE=TRUE PROJECT=$_p FLAVOR= MKTOPDIR=$(pwd) PRODUCT=" ENABLE_TEE=TRUE PROJECT=$_p FLAVOR= MKTOPDIR=$(pwd) PRODUCT= new CUR_MODULE=kernel CROSS_COMPILE=arm-linux-gnueabi-

