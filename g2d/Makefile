CC ?=$(CROSS_COMPILE)gcc
AR ?=$(CROSS_COMPILE)ar

# list of platforms which want this test case
INCLUDE_LIST:=IMX50 IMX51 IMX5 IMX6Q IMX6S

OBJ = g2d.o

LIBNAME = libg2d
SONAMEVERSION = 0.7
LFLAGS += -lpthread

ifeq ($(PLATFORM), $(findstring $(PLATFORM), $(INCLUDE_LIST)))

all: $(LIBNAME).so

install: install_headers
	@mkdir -p $(DEST_DIR)/usr/lib
	cp -P $(LIBNAME).* $(DEST_DIR)/usr/lib

install_headers:
	@mkdir -p $(DEST_DIR)/usr/include
else
all install :
endif

%.o: %.c
	$(CC) -D$(PLATFORM) $(INCLUDE) -Wall -O2 -fPIC -c $^ -o $@

$(LIBNAME)-pxp.so: $(OBJ)
	$(CC) -shared -nostartfiles -Wl,-soname,$(LIBNAME).so.$(SONAMEVERSION) $(LFLAGS) $^ -o $@

$(LIBNAME).so: $(LIBNAME).so.$(SONAMEVERSION)
	ln -s $< $@

$(LIBNAME).so.$(SONAMEVERSION): $(LIBNAME)-pxp.so
	ln -s $< $@

.PHONY: clean
clean:
	rm -f $(LIBNAME)* $(OBJ)
