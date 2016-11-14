# Simple makefile for producing the doxygen documentation of the driver code.
# The folder Decawave API is the proprietary drivers from decawave and are
# excluded.

doc:
	doxygen Doxyfile

.PHONY: doc