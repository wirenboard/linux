#!/bin/bash -e
#
#

[[ -d .git ]] || {
    echo "This script must be started in kernel root directory" >&2
    exit 2
}

CORES_DEFAULT=$(nproc || echo 1)
CORES=${CORES:-$CORES_DEFAULT}

source scripts/package/wb/version.sh
setup_kernel_vars || exit $?

source wb_revision || echo "Warning: failed to source wb_revision file" >&2

if [[ -n "$WB_BRANCH_BASE" && -z "$WB_REVISION" ]]; then
    DEFAULT_WB_REVISION="+r$(git rev-list --count HEAD...${WB_BRANCH_BASE})"
elif [[ -z "$WB_REVISION" ]]; then
    echo "Set either WB_BRANCH_BASE or WB_REVISION variable to determine package version" >&2
    exit 2
fi

export WB_REVISION=${WB_REVISION:-${DEFAULT_WB_REVISION}}
export LOCALVERSION=${LOCALVERSION:-"-${KERNEL_FLAVOUR}"}

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
    KERNEL_RELEASE=$(cat ${KBUILD_OUTPUT}/include/config/kernel.release)
    echo "kernel release: $KERNEL_RELEASE"

    KDEB_PKGVERSION="${KERNEL_RELEASE}${WB_REVISION}"

    fakeroot make -j${CORES} ARCH=arm KBUILD_DEBARCH=${DEBARCH} \
        KDEB_PKGVERSION=${KDEB_PKGVERSION} KDEB_WBTARGET=${KERNEL_FLAVOUR} binwbdeb-pkg
}

echo "Building kernel packages for $KERNEL_FLAVOUR ($KDEB_WBDESC)"
echo "Architecture: ${DEBARCH}"
echo "Local version: ${LOCALVERSION}"
echo "WB revision: ${WB_REVISION}"
echo "Config: ${KERNEL_DEFCONFIG}"

make_config
make_image
make_deb
