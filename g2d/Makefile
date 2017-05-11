CC ?=$(CROSS_COMPILE)gcc
AR ?=$(CROSS_COMPILE)ar

# list of platforms which want this test case
INCLUDE_LIST:=IMX50 IMX51 IMX5 IMX6Q IMX6S IMX7 IMX6UL

OBJ = g2d.o

LIBNAME = libg2d
SONAMEVERSION = 1.1
LDFLAGS += -lpthread

ifeq ($(PLATFORM), $(findstring $(PLATFORM), $(INCLUDE_LIST)))

all: $(LIBNAME).so

install: install_headers
	@mkdir -p $(DEST_DIR)/usr/lib
	cp -P $(LIBNAME)* $(DEST_DIR)/usr/lib

install_headers:
	@mkdir -p $(DEST_DIR)/usr/include
	cp g2d.h $(DEST_DIR)/usr/include

else
all install :
endif

%.o: %.c
	$(CC) -D$(PLATFORM) $(INCLUDE) -Wall -O2 -fPIC -c $^ -o $@

$(LIBNAME)-pxp.so.$(SONAMEVERSION): $(OBJ)
	$(CC) -shared -nostartfiles -Wl,-soname,$(LIBNAME).so.$(SONAMEVERSION) $(LDFLAGS) $^ -o $@

$(LIBNAME).so: $(LIBNAME).so.$(SONAMEVERSION)
	ln -s $< $@

$(LIBNAME).so.$(SONAMEVERSION): $(LIBNAME)-pxp.so.$(SONAMEVERSION)
	ln -s $< $@

.PHONY: clean
clean:
	rm -f $(LIBNAME)* $(OBJ)
