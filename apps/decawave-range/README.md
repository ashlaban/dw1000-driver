Ranging
=======

This application implements a simplistic ranging application. It is designed to
run on a mulle device using contiki as operating system. It consists of two
parts, an anchor and a tag.

Anchor
------

Calculates a measurement. Listens for tag activity and responds. When enough
data is collected the anchor will calulate the distance. This program can be
extended so that the anchor will report the distance to the tag as well.

Tag
---

Initiates a measurement. Can be thought of as a beacon. At some point in time
the tag sends a blink. A paired anchor will then respond and the tag will in
turn respond with the final message. The anchor then proceeds to calculate the
distance between the nodes.

Protocol
--------

There are to states for the tag.
    
    UNPAIRED
    PAIRED

In the unpaired state the tag will announce itself to the world via BLINK
messages. An anchor seeing such a message will respond with a RANGING_INIT
message. This will pair the two devices. Paired devices communicates only with
each other.

Once paired, a distance measurement can be done. This is initiated by the tag
sending a POLL directed at the anchor. The ranging requires a set of six
timestamps to calculate the message time-of-flight. These are collected through
the RESPONSE (sent by anchor) and FINAL (sent by tag) messages. Once the
timestamps are collected the anchor can calculate the distance between the tag
and anchor using an estimated time-of-flight of the messages. Optionally the
measurement can then be reported back to the tag, however this is currently not
implemented.

All messages in order of appereance.
    
    BLINK            Announce tag to world.
    RANGING_INIT     Pair anchor and tag.
    POLL             Initiate measurement.
    RESPONSE         Part of ranging protocol.
    FINAL            After the reception of this message the anchor has enough 
                         information to calculate the distance between anchor 
                         and tag.
    (REPORT)         [Not implemented] Report the distance to the tag.

Building
========

To successfully build this project you will have to do the following:

This project has the following pre-requisites:
    - Contiki        , set up a working clone with a working mulle port.
    - Decawave-Driver, set up somewhere.

The paths to these directories should be noted in project_dependencies.include
so that the build system can find them.

The code is now compilable via the following commands:

    make dw-range-anchor TARGET=mulle
    make dw-range-tag    TARGET=mulle

in the decawave-range folder.

If you want to run the application on physical devices you will need 2 mulle
boards and 2 dw1000's. Set up two terminal groups (or windows). In total you
will need 6 terminal tabs, 3 for each device.

Anchor
------
For the device that will act as anchor, start with running the following command
in the $(CONTIKI)/platform/mulle/tools directory:

    ./start_openocd #NUM

where #NUM is the number printed on the mulle expansion board. This is an FTDI
identification number used to simplifying device programming and communication.
This starts a GDB server that later can be connected to to program the mulle
device.

Then do the following in the decawave-range directory (each in a separate tab):

    make dw-range-anchor.u TARGET=mulle
    sudo make login PORT=/dev/ttyUSB#USB

where ttyUSB#USB is the USB port assigned to your mulle board. This will trigger
the compilation of the anchor code path and upload it to the mulle. The second
command enables UART-communication via stdin and stdout.

Tag
---

Now it is time to build the tag. This is approximately the same as for the
anchor but the commands will be extended slightly so that multiple GDB servers
can be run simultaneously.

In the $(CONTIKI)/platform/mulle/tools directory, run:

    ./start_openocd #NUM #PORT

where #NUM is the number printed on the mulle expansion board and #PORT is an
available port number that will be used for the GDB server.

Then do the following in the decawave-range directory (each in a separate tab):

    make dw-range-anchor.u TARGET=mulle GDB_PORT=#PORT
    sudo make login PORT=/dev/ttyUSB#USB

where #PORT is the same port number as used in the openocd command and
ttyUSB#USB is the USB port assigned to the tag mulle.

Running the ranging application
-------------------------------

You are now ready to run the ranging application. If all is well, the ranging
should automatically start once you write continue and press enter in the two
GDB tabs.

The current configuration of the application will take 1000 measurements and
output some statistics for these measurements. The anchor will then enter an
infinite loop. Resetting the anchor mulle will enable you to take another 1000
measurements.
