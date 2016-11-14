CONTIKI_PROJECT = dw-sync-demo
all: $(CONTIKI_PROJECT)

# All root directories are defined in project_dependencies.include. Use 
# $(CONTIKI) to refer to the contiki installation and $(DRIVER) for 
# Decawave-Driver.
include project_dependencies.include

# Includes the whole of contiki!
include $(CONTIKI)/Makefile.include

#linker optimizations
SMALL=1

# Include local source files.
PROJECT_SOURCEFILES +=

# Include Decawave-Driver with specific drivers for the mulle.
CONTIKIDIRS+=$(DRIVER)/Code/driver
CONTIKIDIRS+=$(DRIVER)/Code/driver/platform/mulle
PROJECT_SOURCEFILES += dw1000-base.c dw1000-spi.c mulle-spi.c mulle-hal.c

