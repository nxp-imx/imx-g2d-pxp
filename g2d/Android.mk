ifeq ($(TARGET_FSL_IMX_2D),PXP2D)
LOCAL_PATH := $(call my-dir)

# Share library
include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
	g2d.c

LOCAL_SHARED_LIBRARIES := libutils libc liblog

LOCAL_CFLAGS:= -DBUILD_FOR_ANDROID -DLOG_TAG=\"g2d\"
LOCAL_C_INCLUDES += $(LOCAL_PATH)
LOCAL_MODULE := libg2d
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_TAGS := eng
include $(BUILD_SHARED_LIBRARY)
endif

