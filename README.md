Basic overview
==============
Hello and welcome! Now what is this?

This is the driver proper for the dw1000 ultra wideband tranceiver. It is designed to be portable. The main driver code found in dw1000-base.h makes only assumes that the hardware abstraction is set up properly. The low-level device specific communication is implemented in the dw1000-hal and dw1000-spi modules.

The driver should be initialised prior to the use of any other `dw-` function. This is done by calling 
```
	dw_init();
```
which initialises all required driver components and configure the device to be ready for use, it does this by applying a default configuration. More detail about this can be found in the documentation of the `dw_init` function.

Once the driver has been initialised, communication can commence. A typical use case for receiving a message is as follows
```
	dw_rx_conf_t rx_conf;
	rx_conf.is_delayed = 0;
	rx_conf.timeout    = 30000; // Unit, approximate milliseconds
	dw_conf_rx( &rx_conf );
	dw_receive(DW_TRANCEIVE_ASYNC);
```
More information can be found in the documentation for the dw_conf_rx and dw_conf_tx functions.

Coding convention
=================

The coding convention used through out the driver mimics the [contiki style](http://www.contiki-os.org/community.html) but there are a few additions.
	- From Hungarian notation is taken the notion to prefix all pointer variables with `p_`. This makes it easier to track when to use dereferencing and what not.
	- Length variables are suffixed with `_len`.

Combining the above rules gives the a common pattern, let's say we want to call an array `data`. A pointer to this array would be named `p_data` and the length of the array would be called `data_len` regardless of whether there exists an acutal variable named `data`.

Applications
============
The apps folder provides examples how to use the driver to build more complete applications. Build instructions can be found inside each application folder.

decawave-ranging
----------------

This is the full-fledged application that is capable of taking distance
measurements.

decawave-demo
-------------

Introduction, how to use the Decawave-Driver in conjunction with the Contiki
operating system.



TODO:
	describe
		Register address
		register length
		bitfields
		bitfiled_mask

TODO:
	organisation
		- HOST
		- DW1000
		- interrupt generation