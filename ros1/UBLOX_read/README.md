# UBLOX_read

A library for parsing UBLOX packets and interfacing with UBLOX gps receivers. It has been designed for use with the M8N and F9P GNSS receivers.

This library provides rather basic functionality and is designed to work under a linux environment, however it should probably work in Windows or Mac as it uses the cross-platform [`async_comm`](https://github.com/dpkoch/async_comm) library as the serial interface.

The UBX parsing functionality is abstracted into a library for easy integration in other projects.  Example usage is given in the `main.cpp` file.

Hardware documentation found [here](https://magiccvs.byu.edu/wiki/#!hw_guides/ublox_f9p.md)
Software documentation found [here](https://magiccvs.byu.edu/wiki/#!sw_guides/ublox_read.md)
