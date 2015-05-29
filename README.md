# UniversalPayloadInterface
Universal UAV Payload Interface Senior Project
Open Source Code & Hardware
Cal Poly SLO

Nolan Reker
Drew Troxell
David Troy

More documentation coming soon!

Getting Started

Requirements
UAV utilizing the Mavlink protocol
Onboard proxy device for running MAVProxy
Atmega328p on payload interface board (see pcb directory for hardware details)

Loading the source
Compile and upload the payload_atmega328p ino project to the atmega328p payload interface board using your favorite AVR compiler/loader.
Connect the interface board to the onboard device via UART.
Run MAVProxy on the onboard device w/ the master device set to the autopilot and the output device set to the UART connection to the interface board.
The onboard LED will light up after successfully receiving 10 heartbeat packets from the autopilot.

Modifying the source
This code is meant to provide a simple example and framework for communicating between a payload and an autopilot via Mavlink. Reference the mavlink library for functions to decode or send more messages via mavlink.
