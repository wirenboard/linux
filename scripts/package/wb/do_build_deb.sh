#!/bin/bash -e
#
#

[[ -d .git ]] || {
    echo "This script must be started in kernel root directory" >&2
    exit 2
}

CORES_DEFAULT=$(nproc || echo 1)
CORES=${CORES:-$CORES_DEFAULT}

case $1 in
*help)
    echo "Usage: KERNEL_FLAVOUR=<flavour> $0" >&2
    echo -e "\nOptional envvars:"
    echo -e "\tCORES\tNumber of threads to build kernel (default $CORES)"
    echo -e "\tFORCE_DEFAULT\tDefault answer about using exising defconfig (y/n)"
    echo -e "\tVERSION_SUFFIX\tCustom version suffix for non-dev branches"
    exit
    ;;
esac

source scripts/package/wb/version.sh
setup_kernel_vars || exit $?

export DEBEMAIL="info@wirenboard.com"
export DEBFULLNAME="Wirenboard robot"

export KBUILD_OUTPUT=".build-$KERNEL_FLAVOUR"

mkdir -p "$KBUILD_OUTPUT"

make_config() {
    if [[ -t 0 && -e "${KBUILD_OUTPUT}/.config" ]]; then
        echo ".config already present"
        if [[ -n "$FORCE_DEFAULT" ]]; then
            echo "Forced using $KERNEL_DEFCONFIG: $FORCE_DEFAULT"
            yn=$FORCE_DEFAULT
        else
            read -n 1 -p "Use $KERNEL_DEFCONFIG instead? (y/N) " yn
            echo
        fi
        if [[ "$yn" == "y" ]]; then
            make ARCH=arm $KERNEL_DEFCONFIG
        else
            echo "Using existing .config"
        fi
    else
        make ARCH=arm $KERNEL_DEFCONFIG
    fi
}

make_image() {
    make -j${CORES} ARCH=arm KBUILD_DEBARCH=${DEBARCH} zImage modules dtbs
}

make_deb () {
    fakeroot make -j${CORES} ARCH=arm KBUILD_DEBARCH=${DEBARCH} \
        LOCALVERSION='' WB_VERSION_SUFFIX=$VERSION_SUFFIX \
        KDEB_WBTARGET=${KERNEL_FLAVOUR} binwbdeb-pkg
}

echo "Building kernel packages for $KERNEL_FLAVOUR ($KDEB_WBDESC)"
echo "Architecture: ${DEBARCH}"
echo "Config: ${KERNEL_DEFCONFIG}"

make_config
make_image
make_deb
