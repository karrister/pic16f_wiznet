pic16f_wiznet
=============

A project to make a simple WEB server/client out of a PIC16F690 uC using Wiznet 5100 chip (ETH PHY & stack on chip over SPI).


The purpose of this project is to pass the Embedded Networking course project held in Helsinki Metropolia University of Applied Sciences.

Currently the aim is to make a simple WEB server that can be used to control the 4 LEDs on the PIC16F690 development board that is attached to the WIZNET 5100 CHIP over SPI.


Known problems:
1. At times the Wiznet chip does not boot correctly. Results in no observable problems other than the interface to Ethernet does not come up. Have attempted to solve by connecting the W5100’s Hardware Reset line to the PIC, initiating a Hardware reset during boot of PIC, which made the correct bootup percentage higher, but did not completely solve the issue.
2. The HTTP protocol implementation and communication is not entirely correct. I implemented it enough for the user to be able to see the website, and press the links on it (I have tested the site on Chrome). On purpose is left out for instance the “HTTP 1.0 OK” message. This can be seen when sniffing with Wireshark, with occasional RST TCP messages and such “ugly” behaviours.
3. Occasional rubbish characters seen on the client web browser. Sometimes it looks like the client misses the ending of the visible page, and instead after the end of the real page prints some random characters in some buffer, either the end of a packet received by the client, or a part of an internal buffer of the browser. The partial reason for this issue might be the issue number 2 above.