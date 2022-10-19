```
        __   _ __      _  __    ___  ___  ___  ___  ____ ___      ___  __  _______  ___
       / /  (_) /____ | |/_/___/ _ \/ _ \|_  |/ _ \/ / // _ \____/ _ \/  |/  / __ \/ _ \
      / /__/ / __/ -_)>  </___/ , _/ ___/ __// // /_  _/ // /___/ ___/ /|_/ / /_/ / // /
     /____/_/\__/\__/_/|_|   /_/|_/_/  /____/\___/ /_/ \___/   /_/  /_/  /_/\____/____/
                       Copyright (c) 2022, Enjoy-Digital & LiteX developers
```
![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)

<p align="center"><img src="https://user-images.githubusercontent.com/1450143/196745004-54713783-a5f5-4355-8f50-86d014b47cc9.JPG"></p>

[> Intro
--------
- You need a CPU in your FPGA based SoC but this one is using too much resources and/or slowing down synthesis, place and route too much?
- You want to easily run standalone applications with MicroPython/CircuitPython on your FPGA board without re-inventing everything?
- You want to extend the capabilities of the RP2040 and prototype this with an FPGA?
- You want to run a Soft-USB stack on the RP2040 and provide simple Keyboard/Mouse support to your FPGA?
- Etc...

-> Why not using a RP2040 as an FPGA PMOD? If interested, we've got you at covered :)

[> Build your RP2040 PMOD!
--------------------------

Building an RP2040 based FPGA PMOD is very easy. To build our PMOD we are using a RP2040-Zero module from  WaveShare. The PMOD can be created by just soldering a 6-pin header and add an additional wire to connect the ground:

<p align="center"><img src="https://user-images.githubusercontent.com/1450143/196745605-419eaa90-fd1e-40ed-9684-0c93717a3034.png" width="400"></p>

This simple PMOD will give us 4-wires to communicate with the FPGA, allowing 4-wires SPI. (For now since faster protocols could be supported in the future through RP2040's PIO).

[> Select your FPGA core/ design flow.
--------------------------------------

The current implementation is using 4-wires SPIBone protocol for the RP2040 <-> FPGA link. SPIBone is directly integrated in LiteX and can be added very easily in LiteX design with just 2
lines of Python code:

```python3
self.submodules.spibone = SPIBone(platform.request("rp2040_spi"))
self.bus.add_master(name="rp2040", master=self.spibone.bus)
```
This is convenient when the SoC is designed with LiteX, but less for users willing to just reuse the SPIBone core. To allow users to also use the core with regular flow, a standalone core generatore has been created; capable of generating SPIBone as a SPI <-> Wishbone or SPI <-> AXI-Lite core and the example design is also demonstrating the generation + integration of the
 core in the LiteX top level SoC:

<p align="center"><img src="https://user-images.githubusercontent.com/1450143/196749170-b7ea43ca-06bb-42b0-8345-aeb10cff739a.png"></p>

For integration in a traditional FPGA flow, the pre-generated SPIBone cores are also provided:

 - SPIBone Wishbone version: [here](https://github.com/enjoy-digital/litex_rp2040_pmod_test/files/9822865/spibone_core_wishbone.txt).
 - SPIBone AXI-Lite version: [here](https://github.com/enjoy-digital/litex_rp2040_pmod_test/files/9822859/spibone_core_axi_lite.txt).

> Note: This generator + re-integration of the standalone cores can also be a good example to see how to integrate Wishbone/AXI-Lite peripherals to LiteX and how to handle bus standard conversions.

[> Enjoy :)
-----------

<p align="center"><img src="https://user-images.githubusercontent.com/1450143/196745028-67871766-e8b2-4146-a008-af0d500acf24.JPG"></p>

A simple test design is provided with a minimal LiteX SoC running on the FPGA and MicroPython on the RP2040 controlling it:

<p align="center"><img src="https://user-images.githubusercontent.com/1450143/196760532-09bf9898-e229-45f6-8bb1-78a82c5eb351.png"></p>

**Build/Flash the design with the full LiteX flow:**

    ./icebreaker.py --mode=litex --build --flash

**Build/Flash the design with the standalone SPIBone Wishbone core re-integrated:**

    ./icebreaker.py --mode=hybrid-wishbone --build --flash

**Build/Flash the design with the standalone SPIBone Wishbone core re-integrated:**

    ./icebreaker.py --mode=hybrid-axi-lite --build --flash

**Run the MicroPython test script on the RP2040:**
On the RP2040, make sure to have MicroPython installed. Then just run:

    sudo ./pyboard.py test_spi_bone.py

The script will read the FPGA identifier and exercise the different peripherals:

<p align="center"><img src="https://user-images.githubusercontent.com/1450143/196763644-5f61d16c-85fb-4138-8ab8-90af98512524.png"></p>

Have fun! The RP2040 is a very capable chip so the possibilities shown here are probably only the beginning... :)