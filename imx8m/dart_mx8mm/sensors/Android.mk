LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_VENDOR_MODULE := true
LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_SHARED_LIBRARIES := liblog
LOCAL_HEADER_LIBRARIES := libhardware_headers
LOCAL_SRC_FILES := sensors.cpp
LOCAL_MODULE := sensors.imx8
LOCAL_CFLAGS := -std=c++11

include $(BUILD_SHARED_LIBRARY)
