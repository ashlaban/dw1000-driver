Syncronous demo
===============

This demo application introduces how to use the Decawave-Driver in the Contiki
operating system to communicate with the dw1000 UWB tranceiver on a mulle
device.

The application uses the programatically simple approach of synchronous
communication (implemented with busy wating of the host cpu until the
operation is complete on the dw1000) to send a 10 byte payload repeatedly. In
between successive sends it will try to receive a single data frame.

Two application running next to each other should (after a number of device
resets) be able to receive messages continually.

Building
--------

To successfully build this project you will have to do the following:

This project has the following pre-requisites:
    - Contiki        , set up a working clone with a working mulle port.
    - Decawave-Driver, set up somewhere.

The paths to these directories should be noted in project_dependencies.include
so that the build system can find them.

The code is now compilable via the following commands:

    make dw-sync-demo TARGET=mulle

in the decawave-demo folder.

If you want to run the application on physical devices you will need 2 mulle
boards and 2 dw1000's. Set up two terminal groups (or windows). In total you
will need 6 terminal tabs, 3 for each device.

First Device
------------
For the device that will act as anchor, start with running the following command
in the $(CONTIKI)/platform/mulle/tools directory:

    ./start_openocd #NUM

where #NUM is the number printed on the mulle expansion board. This is an FTDI
identification number used to simplifying device programming and communication.
This starts a GDB server that later can be connected to to program the mulle
device.

Then do the following in the decawave-demo directory (each in a separate tab):

    make dw-sync-demo.u TARGET=mulle
    sudo make login PORT=/dev/ttyUSB#USB

where ttyUSB#USB is the USB port assigned to your mulle board. This will trigger
the compilation of the anchor code path and upload it to the mulle. The second
command enables UART-communication via stdin and stdout.

Second Device
-------------

Now it is time to build the tag. This is approximately the same as for the
anchor but the commands will be extended slightly so that multiple GDB servers
can be run simultaneously.

In the $(CONTIKI)/platform/mulle/tools directory, run:

    ./start_openocd #NUM #PORT

where #NUM is the number printed on the mulle expansion board and #PORT is an
available port number that will be used for the GDB server.

Then do the following in the decawave-range directory (each in a separate tab):

    make dw-sync-demo.u TARGET=mulle GDB_PORT=#PORT
    sudo make login PORT=/dev/ttyUSB#USB

where #PORT is the same port number as used in the openocd command and
ttyUSB#USB is the USB port assigned to the tag mulle.
