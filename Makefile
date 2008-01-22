
subdirs = $(shell /bin/ls -d */)

%::
	@for X in $(subdirs); do        \
	    if [ -r "$$X/Makefile" ]; then \
	        $(MAKE) -C $$X $@; \
	    fi;                         \
	done
