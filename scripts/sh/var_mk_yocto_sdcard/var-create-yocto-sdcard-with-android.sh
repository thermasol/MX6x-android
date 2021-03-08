#!/bin/bash
set -e

#### Script version ####
SCRIPT_NAME=${0##*/}
readonly SCRIPT_VERSION="0.8"

#### Exports Variables ####
#### global variables ####
readonly ABSOLUTE_FILENAME=`readlink -e "$0"`
readonly ABSOLUTE_DIRECTORY=`dirname ${ABSOLUTE_FILENAME}`
readonly SCRIPT_POINT=`pwd`/sources/meta-variscite-fslc/scripts/

ANDROID_SCRIPTS_PATH=${SCRIPT_POINT}/var_mk_yocto_sdcard/variscite_scripts
ANDROID_BUILD_ROOT=~/var_imx-p10.0.0_1.0.0-ga/android_build

TEMP_DIR=./var_tmp
ROOTFS_MOUNT_DIR=${TEMP_DIR}/rootfs

help() {
	bn=`basename $0`
	echo " Usage: MACHINE=<imx8mq-var-dart|imx8mm-var-dart|imx8qxp-var-som|imx8qm-var-som|imx8mn-var-som|imx8mp-var-som> $bn device_node"
	echo
}

case $MACHINE in
	"imx8mq-var-dart")
		ANDROID_IMGS_PATH=${ANDROID_BUILD_ROOT}/out/target/product/dart_mx8mq
		;;
	"imx8mp-var-dart")
		ANDROID_IMGS_PATH=${ANDROID_BUILD_ROOT}/out/target/product/dart_mx8mp
		;;
	"imx8mm-var-dart")
		ANDROID_IMGS_PATH=${ANDROID_BUILD_ROOT}/out/target/product/dart_mx8mm
		;;
	"imx8qxp-var-som")
		ANDROID_IMGS_PATH=${ANDROID_BUILD_ROOT}/out/target/product/som_mx8q
		;;
	"imx8qm-var-som")
		ANDROID_IMGS_PATH=${ANDROID_BUILD_ROOT}/out/target/product/som_mx8q
		;;
	"imx8mn-var-som")
		ANDROID_IMGS_PATH=${ANDROID_BUILD_ROOT}/out/target/product/som_mx8mn
		;;
	*)
		help
		exit 1
esac

MACHINE=$MACHINE ${SCRIPT_POINT}/var_mk_yocto_sdcard/var-create-yocto-sdcard.sh "$@"

# Parse command line only to get ${node} and ${part}
moreoptions=1
node="na"
cal_only=0

while [ "$moreoptions" = 1 -a $# -gt 0 ]; do
	case $1 in
	    -h) ;;
	    -s) ;;
	    -a) ;;
	    -r) shift;
	    ;;
	    -n) shift;
	    ;;
	    *)  moreoptions=0; node=$1 ;;
	esac
	[ "$moreoptions" = 1 ] && shift
done

part=""
if [[ $node == *mmcblk* ]] || [[ $node == *loop* ]] ; then
	part="p"
fi

echo "========================================================"
echo "= Variscite recovery SD card creation script - Android ="
echo "========================================================"

function mount_parts
{
	mkdir -p ${ROOTFS_MOUNT_DIR}
	sync
	mount ${node}${part}1  ${ROOTFS_MOUNT_DIR}
}

function unmount_parts
{
	umount ${ROOTFS_MOUNT_DIR}
	rm -rf ${TEMP_DIR}
}

function copy_android
{
	echo
	echo "Copying Android images to /opt/images/"
	mkdir -p ${ROOTFS_MOUNT_DIR}/opt/images/Android

	cp ${ANDROID_IMGS_PATH}/u-boot-*.imx	${ROOTFS_MOUNT_DIR}/opt/images/Android/
	cp ${ANDROID_IMGS_PATH}/boot.img			${ROOTFS_MOUNT_DIR}/opt/images/Android/
	cp ${ANDROID_IMGS_PATH}/dtbo-*.img			${ROOTFS_MOUNT_DIR}/opt/images/Android/
	cp ${ANDROID_IMGS_PATH}/vbmeta-*.img			${ROOTFS_MOUNT_DIR}/opt/images/Android/

	if [[ "${MACHINE}" = "imx8qm-var-som" ]]; then
		echo "Copying firmware images to /opt/images/"
		cp ${ANDROID_IMGS_PATH}/vendor/firmware/hdmitxfw.bin	${ROOTFS_MOUNT_DIR}/opt/images/Android/
		cp ${ANDROID_IMGS_PATH}/vendor/firmware/dpfw.bin	${ROOTFS_MOUNT_DIR}/opt/images/Android/
	fi

	if [ -e "${ANDROID_IMGS_PATH}/super.img" ]; then
		echo "Copying super image to /opt/images/"
		pv ${ANDROID_IMGS_PATH}/super.img >		${ROOTFS_MOUNT_DIR}/opt/images/Android/super.img
		sync | pv -t
	else
		echo "Copying system image to /opt/images/"
		pv ${ANDROID_IMGS_PATH}/system.img >		${ROOTFS_MOUNT_DIR}/opt/images/Android/system.img
		sync | pv -t
		echo "Copying vendor image to /opt/images/"
		pv ${ANDROID_IMGS_PATH}/vendor.img >		${ROOTFS_MOUNT_DIR}/opt/images/Android/vendor.img
		sync | pv -t
		echo "Copying product image to /opt/images/"
		pv ${ANDROID_IMGS_PATH}/product.img >		${ROOTFS_MOUNT_DIR}/opt/images/Android/product.img
		sync | pv -t
	fi
}

function copy_android_scripts
{
	echo
	echo "Copying Android script"
	cp ${ANDROID_SCRIPTS_PATH}/mx8_install_android.sh		${ROOTFS_MOUNT_DIR}/usr/bin/install_android.sh
}

mount_parts
copy_android
copy_android_scripts

echo
echo "Syncing"
sync | pv -t

unmount_parts

echo
echo "Done"

exit 0
