#/*
# *  Copyright (C) 2013-2014 Freescale Semiconductor, Inc.
# *  All Rights Reserved.
# *
# *  The following programs are the sole property of Freescale Semiconductor Inc.,
# *  and contain its proprietary and confidential information.
# *
# */
#
# Linux build file for g2d library
#
.PHONY: clean

AQROOT ?= ../../../driver/

include $(AQROOT)/makefile.linux.def

OBJS := g2d.o

# build as shared library
TARGET := libg2d.so

CFLAGS += -I ../include -fPIC -I $(AQROOT)/sdk/inc -I$(ROOTFS_USR)/include
ifeq ($(SDK_BUILD), 1)
LFLAGS = -L $(SDK_DIR)/drivers -L $(ROOTFS_USR)/lib -lpxp -lpthread
else
LFLAGS = -L $(ROOTFS_USR)/lib -lpxp -lpthread
endif

CC = $(CROSS_COMPILE)gcc

$(TARGET): $(OBJS)
	$(CC) -shared -fPIC -Wl,-soname,$@ -o $@ $(OBJS) $(LFLAGS)
ifeq ($(SDK_BUILD), 1)
	-@cp $@ $(SDK_DIR)/drivers/
endif
	-@mkdir -p ../lib/
	-@cp $@ ../lib/

clean:
	-@rm $(TARGET) $(OBJS)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<
