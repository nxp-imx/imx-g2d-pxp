CC ?=$(CROSS_COMPILE)gcc
AR ?=$(CROSS_COMPILE)ar

# list of platforms which want this test case
INCLUDE_LIST:=IMX50 IMX51 IMX5 IMX6Q IMX6S IMX7 IMX6UL IMX93 IMX943

OBJ = g2d.o

LIBNAME = libg2d
#API version
SONAMEVERSION := 2
API_VERSION := $(SONAMEVERSION).4.0
LDFLAGS += -lpthread
INCLUDE += -I../include
INCLUDE += -I$(SDKTARGETSYSROOT)/usr/include
INCLUDE += -I$(SDKTARGETSYSROOT)/usr/include/imx

libdir ?= "/usr/lib"

ifeq ($(PLATFORM), $(findstring $(PLATFORM), $(INCLUDE_LIST)))

ifeq ($(PLATFORM), IMX93)
	CFLAGS += -DENABLE_HARDWARE_VERSION=1
endif

ifeq ($(PLATFORM), IMX943)
	CFLAGS += -DENABLE_HARDWARE_VERSION=2
endif

all: $(LIBNAME).so

install: install_headers
	@mkdir -p $(DEST_DIR)${libdir}
	cp -P $(LIBNAME)* $(DEST_DIR)${libdir}

install_headers:
	@mkdir -p $(DEST_DIR)/usr/include
	cp g2d.h g2dExt.h $(DEST_DIR)/usr/include

else
all install :
endif

%.o: %.c
	$(CC) $(CFLAGS) -D$(PLATFORM) $(INCLUDE) -Wall -O2 -fPIC -c $^ -o $@

$(LIBNAME)-pxp.so.$(API_VERSION): $(OBJ)
	$(CC) -shared -nostartfiles -Wl,-soname,$(LIBNAME).so.$(SONAMEVERSION) $(LDFLAGS) $^ -o $@

$(LIBNAME).so: $(LIBNAME).so.$(SONAMEVERSION)
	ln -s $< $@

$(LIBNAME).so.$(SONAMEVERSION): $(LIBNAME)-pxp.so.$(API_VERSION)
	ln -s $< $@

.PHONY: clean
clean:
	rm -f $(LIBNAME)* $(OBJ)
