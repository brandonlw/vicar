Vicar
=====

Flash this to your USB development board (with attached USB host shield) to create the Vicar USB proxy device.

You can build the Teensy code with the Atmel AVR toolchain on Windows.
The C# code can be built with Visual Studio 2012 Express.

I'm too lazy to explain it, so just view the source for how to use it, if you're interested.

#### Hints
To run the test app project, pass these as command line arguments:
> /mode=[mode] /configs=[configs] /demo=[demo] [/verbose]
* [mode] is either "HID" or "Custom".
* [configs] is either "MassStorageDevice", "HumanInterfaceDevice", or 192 for both (0x40 + 0x80).
* [demo] is either "MassStorage", "MassStorageForwarding", or "SimulateKeyboard".
* /verbose is optional and, if specified, will output lots of stuff.

HID mode means the Teensy shows up as two HID devices (one for regular communication and a second for sending large (4KB) control requests). (Yes, this can be done with separate report IDs on one interface, but I had issues with this.)
Custom mode means the Teensy shows up as a WinUSB device.
The additional configurations only function in Custom mode and cause additional mass storage or HID keyboard interfaces to appear. They have no meaning in HID mode.

#### Demos
* MassStorage: implements the mass storage Bulk-Only Transport protocol and creates a simple shell for browinsg the first primary partition on the drive and getting/putting files.
* MassStorageForwarding: forwards communication between a connected flash drive and the Custom mass storage interface.
* SimulateKeyboard: echoes strings you enter back to you by "typing" them on the Custom HID keyboard interface 5 seconds later.

#### MassStorage demo shell commands:
* ls/dir: show files/directories.
* pwd: print working directory.
* cd: change directory.
* get: gets the file to the path/name you specify.
* put: puts the file you specify into the current directory.
* flush: flushes any cached writes to the drive. (Without this, the whole thing is read-only.)

Sorry I couldn't be more thorough, I'm just tired of working on it.
