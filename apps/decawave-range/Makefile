CONTIKI_PROJECT = dw-range-anchor dw-range-tag
all: $(CONTIKI_PROJECT)

# All root directories are defined in project_dependencies.include. Use 
# $(CONTIKI) to refer to the contiki installation and $(DRIVER) for 
# Decawave-Driver.
include project_dependencies.include

### This is for coap ###
# WITH_UIP6=1
# UIP_CONF_IPV6=1
# CFLAGS+= -DUIP_CONF_IPV6_RPL
# CFLAGS+= -DNODE_ID=0x21
# # WITH_COAP=13
# # CFLAGS += -DWITH_COAP=13
# # CFLAGS += -DREST=coap_rest_implementation
# # APPS += er-coap-13
# # APPS += erbium
# APPS += er-coap
# APPS += rest-engine
### end coap ###

#linker optimizations
SMALL=1

# Include local source files.
PROJECT_SOURCEFILES += dw-range.c dwFrameTypes.c

# Include Decawave-Driver with specific drivers for the mulle.
CONTIKIDIRS+=$(DRIVER)/Code/driver
CONTIKIDIRS+=$(DRIVER)/Code/driver/platform/mulle
PROJECT_SOURCEFILES += dw1000-base.c dw1000-spi.c mulle-spi.c mulle-hal.c

# Includes the whole of contiki!
# This must be after any declarations affecting compilation.
include $(CONTIKI)/Makefile.include

# This is under development. Test for dumping data to file instead of printing 
# to console.
SERIALDUMP=$(CONTIKI)/tools/sky/serialdump-linux 
DATA_DIR =./data
DATA_FILE=test
datadump:
	$(SERIALDUMP) -b115200 $(PORT) > $(DATA_DIR)/$(DATA_FILE)
