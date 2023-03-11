# librelab::Big Dot

![These birds are chirping (CSS) with LoRa!](chirp.jpg)

> A SAML21-based board kit for your environmental sensing projects!

## Description

In this repository you will find the design and firmware files for librelab::Big Dot, our libre hardware kit for low-energy LoRa applications. The content in this repo is organized in the following way:

```bash
├── arduino-core (support files)
│   ├── core
│   └── tools
├── board (hardware design)
│   ├── lib
│   └── output
├── bootloader (firmware files)
│   ├── bootloaders
│   ├── openocd-conf
│   └── sketches
└── examples (applications)
    ├── fastled-ultrasonic
    └── ttn-pm-co2
```

## Installation

### Arduino support

To install the support files for librelab::Big Dot on Arduino, just include the following line in the field 'Additional Boards Manager URL' (under 'Preferences' menu) of your Arduino IDE:

```
https://raw.githubusercontent.com/UVA-DSI/bigdot/main/arduino-core/package_librelab_index.json
```

We are using the Arduino SAMD core (v1.6), but forked and modified by [MattairTech to add support for the chip SAML21](https://github.com/mattairtech/ArduinoCore-samd). You will need Arduino 1.8.13 or superior to use this core with our board.

This core has been tested on Linux (64-bit) and Mac OS 10.15 (64-bit only). If you need further instructions on how to install the Arduino core, [follow this link](https://www.arduino.cc/en/guide/cores).

### RIOT-OS support

If Arduino is suitable for your project, it is OK. All the paths lead to the same destination, but some are more direct than others. You may want to use RIOT-OS! [Our board is supported and can be found here](https://github.com/UVA-DSI/RIOT).

To use RIOT-OS with librelab::Big Dot, just pass its name to `make`:

```bash
$ BOARD=bigdot make all
```

If you are new to RIOT-OS, make sure to follow the [tutorials](https://github.com/RIOT-OS/Tutorials).

### Micropython support

_Coming up!_

## Prototyping

If you do not have a Big Dot at hand, you can use a SAML21 Xplained board for testing purposes. The information on how to flash the bootloader we use can be found in the [bootloader directory](bootloader/README.md). For your convenience, we also included pre-compiled firmware files in this repository as well.

We recommend the usage of the UF2 bootloader included in the `bootloaders` directory: `bigdot-uf2.bin`.

## Project info

Our hardware kit was funded by the "Network Public Spaces" (NPS) project at University of Virginia. NPS is a collaborative project involving the School of Data Science and the School of Architecture for the study and implementation of community-based environmental sensor networks. For more info, check out the [project website](https://example.com).

## License

All the documentation is released as CC-BY-4.0-International.
All the hardware code is released under CERN-OHL-v2 Strongly Reciprocal.
All software code is released under GPL v3, unless marked otherwise on the source file headers. 
See LICENSE and AUTHORS files for further information.

OSHW certification: _pending_.

__Happy hw hacking!__
