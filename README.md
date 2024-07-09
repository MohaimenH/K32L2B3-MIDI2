# TinyUSB + K32L2B3 MIDI2.0 Device

The repo consists of a MIDI2.0 demo on the K32L2B3 board using TinyUSB. It uses drivers provided by the MIDI2.dev community.

## Functionality

Currently the code does the following:

- Send a Note On Message
- Send a Polyphonic Note + the Pressure
- Send a Pitch Bend + the Pitch

## Usage

This repository requires MCUXpresso and the K32L2B3 Board. Please ensure you have the software and the relevant SDK installed beforehand.

1. Clone this repository and add as a project to MCUXpresso.
    - In the event there is any linker error, please add the "tinyusb-midi2/src" folder as source and to the compiler's include path.
    - **Alternatively, the exported project ZIP can be downloaded from the Releases section.**
2. Connect K32L2B3 (OpenSDA Port)
3. Flash the K32L2B3 using the "Debug" option


## Testing

Using the [MIDI2 Workbench](https://github.com/midi2-dev/MIDI2.0Workbench/), the functionality of the board as a MIDI2 device, can be tested. Use the latest version under "Release".