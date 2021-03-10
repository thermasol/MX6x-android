#
# SoC-specific compile-time definitions.
#

BOARD_SOC_TYPE := IMX8MM
BOARD_TYPE := DART-IMX8MM
BOARD_HAVE_VPU := true
BOARD_VPU_TYPE := hantro
HAVE_FSL_IMX_GPU2D := true
HAVE_FSL_IMX_GPU3D := true
HAVE_FSL_IMX_IPU := false
HAVE_FSL_IMX_PXP := false
BOARD_KERNEL_BASE := 0x40400000
TARGET_GRALLOC_VERSION := v3
TARGET_HIGH_PERFORMANCE := true
TARGET_USES_HWC2 := true
TARGET_HWCOMPOSER_VERSION := v2.0
USE_OPENGL_RENDERER := true
TARGET_HAVE_VULKAN := true
ENABLE_CFI=false

SOONG_CONFIG_IMXPLUGIN += \
                          BOARD_HAVE_VPU \
                          BOARD_VPU_TYPE

SOONG_CONFIG_IMXPLUGIN_BOARD_SOC_TYPE = IMX8MM
SOONG_CONFIG_IMXPLUGIN_BOARD_HAVE_VPU = true
SOONG_CONFIG_IMXPLUGIN_BOARD_VPU_TYPE = hantro

#
# Product-specific compile-time definitions.
#

IMX_DEVICE_PATH := device/variscite/imx8m/dart_mx8mm

include device/fsl/imx8m/BoardConfigCommon.mk
ifeq ($(PREBUILT_FSL_IMX_CODEC),true)
-include $(FSL_CODEC_PATH)/fsl-codec/fsl-codec.mk
endif

BUILD_TARGET_FS ?= ext4
TARGET_USERIMAGES_USE_EXT4 := true

TARGET_RECOVERY_FSTAB = $(IMX_DEVICE_PATH)/fstab.freescale

# Support gpt
ifeq ($(IMX_NO_PRODUCT_PARTITION),true)
BOARD_BPT_INPUT_FILES += device/fsl/common/partition/device-partitions-13GB-ab-no-product.bpt
ADDITION_BPT_PARTITION = partition-table-28GB:device/fsl/common/partition/device-partitions-28GB-ab-no-product.bpt \
                         partition-table-dual:device/fsl/common/partition/device-partitions-13GB-ab-dual-bootloader-no-product.bpt \
                         partition-table-28GB-dual:device/fsl/common/partition/device-partitions-28GB-ab-dual-bootloader-no-product.bpt
else
BOARD_BPT_INPUT_FILES += device/fsl/common/partition/device-partitions-13GB-ab.bpt
ADDITION_BPT_PARTITION = partition-table-28GB:device/fsl/common/partition/device-partitions-28GB-ab.bpt \
                         partition-table-dual:device/fsl/common/partition/device-partitions-13GB-ab-dual-bootloader.bpt \
                         partition-table-28GB-dual:device/fsl/common/partition/device-partitions-28GB-ab-dual-bootloader.bpt
endif

# Vendor Interface manifest and compatibility
DEVICE_MANIFEST_FILE := $(IMX_DEVICE_PATH)/manifest.xml
DEVICE_MATRIX_FILE := $(IMX_DEVICE_PATH)/compatibility_matrix.xml

TARGET_BOOTLOADER_BOARD_NAME := EVK

USE_OPENGL_RENDERER := true

BOARD_WLAN_DEVICE            := bcmdhd
WPA_SUPPLICANT_VERSION       := VER_0_8_X
BOARD_WPA_SUPPLICANT_DRIVER  := NL80211
BOARD_HOSTAPD_DRIVER         := NL80211

BOARD_HOSTAPD_PRIVATE_LIB               := lib_driver_cmd_$(BOARD_WLAN_DEVICE)
BOARD_WPA_SUPPLICANT_PRIVATE_LIB        := lib_driver_cmd_$(BOARD_WLAN_DEVICE)

#BOARD_USE_SENSOR_FUSION := true

# we don't support sparse image.
TARGET_USERIMAGES_SPARSE_EXT_DISABLED := false

BOARD_HAVE_USB_CAMERA := true

USE_ION_ALLOCATOR := true
USE_GPU_ALLOCATOR := false

BOARD_AVB_ENABLE := true
BOARD_AVB_ALGORITHM := SHA256_RSA4096
# The testkey_rsa4096.pem is copied from external/avb/test/data/testkey_rsa4096.pem
BOARD_AVB_KEY_PATH := device/fsl/common/security/testkey_rsa4096.pem
TARGET_USES_MKE2FS := true

# define frame buffer count
NUM_FRAMEBUFFER_SURFACE_BUFFERS := 3

ifeq ($(PRODUCT_IMX_DRM),true)
CMASIZE=736M
else
CMASIZE=800M
endif

# Broadcom BT
BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR := $(IMX_DEVICE_PATH)/bluetooth
BOARD_CUSTOM_BT_CONFIG := $(BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR)/vnd_config.txt
BOARD_HAVE_BLUETOOTH_BCM := true

ifeq ($(TARGET_USERIMAGES_USE_UBIFS),true)
ifeq ($(TARGET_USERIMAGES_USE_EXT4),true)
$(error "TARGET_USERIMAGES_USE_UBIFS and TARGET_USERIMAGES_USE_EXT4 config open in same time, please only choose one target file system image")
endif
endif

BOARD_PREBUILT_DTBOIMAGE := out/target/product/dart_mx8mm/dtbo-imx8mm-var-dart-dt8mcustomboard.img

TARGET_BOARD_DTS_CONFIG := \
        imx8mm-var-dart-dt8mcustomboard:fsl-imx8mm-var-dart-dt8mcustomboard.dtb \
        imx8mm-var-dart-dt8mcustomboard-legacy:fsl-imx8mm-var-dart-dt8mcustomboard-legacy.dtb \
	imx8mm-var-som:fsl-imx8mm-var-som.dtb \
	imx8mm-var-som-legacy:fsl-imx8mm-var-som-legacy.dtb \
	imx8mm-var-som-7inch:fsl-imx8mm-var-som-7inch.dtb \
	imx8mm-var-som-10inch:fsl-imx8mm-var-som-10inch.dtb

BOARD_SEPOLICY_DIRS := \
       device/fsl/imx8m/sepolicy \
       $(IMX_DEVICE_PATH)/sepolicy

ifeq ($(PRODUCT_IMX_DRM),true)
BOARD_SEPOLICY_DIRS += \
       $(IMX_DEVICE_PATH)/sepolicy_drm
endif

TARGET_BOARD_KERNEL_HEADERS := device/fsl/common/kernel-headers

ALL_DEFAULT_INSTALLED_MODULES += $(BOARD_VENDOR_KERNEL_MODULES)
