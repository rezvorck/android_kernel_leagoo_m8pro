#!/bin/bash

e="\x1b[";c=$e"39;49;00m";y=$e"93;01m";cy=$e"96;01m";r=$e"1;91m";g=$e"92;01m";

export KBUILD_BUILD_USER=rezvorck
export KBUILD_BUILD_HOST=home
export CONFIG_DEBUG_SECTION_MISMATCH=y

toolchain='https://releases.linaro.org/components/toolchain/binaries/5.3-2016.05/arm-linux-gnueabi/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabi.tar.xz'

run=$(date +%s)

mkdir ./tools/tools 2>/dev/null

if [ -f arch/arm/boot/zImage-dtb ]
then
    echo -e "$y >> Remove kernel... $c"
    rm arch/arm/boot/zImage*
fi

if ! [ -d ../$(basename $toolchain .tar.xz) ]
then
	echo -e "$y >> Download toolchain... $c"
	wget -qP ../ $toolchain && tar -xf ../$(basename $toolchain) -C ../ && rm ../$(basename $toolchain)
fi

echo -e "$y >> Export toolchain... $c"
export ARCH=arm && export export CROSS_COMPILE="../$(basename $toolchain .tar.xz)/bin/arm-linux-gnueabi-"

echo -e "$y >> Make defconfig... $c"
make leagoo_m8pro_32_defconfig >/dev/null

echo -e "$y >> Start build... $c"
make -j4 >/dev/null 2>errors_32.log

if [ ! -f arch/arm/boot/zImage-dtb ]
then
    echo -e "$r Build error! $c"
    echo "$(cat errors_32.log | grep error 2>/dev/null)"
else
    mv arch/arm/boot/zImage-dtb boot_32.img-kernel
    echo -e "$g Finish! Build time: $((($(date +%s) - run)/60)) min. $c"
fi

echo -e "$cy Press [y] to clean project... $c"
read -s -n1 answer; [ "$answer" = "y" ] && make ARCH=arm mrproper 1>/dev/null 2>/dev/null 
