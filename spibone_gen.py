#!/usr/bin/env python3

#
# This file is part of LiteX-RP2040-PMOD-Test.
#
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
SPIBone standalone core generator

SPIBone aims to be directly used as a python package when the SoC is created using LiteX. However,
for some use cases it could be interesting to generate a standalone verilog file of the core:
- integration of the core in a SoC using a more traditional flow.
- need to version/package the core.
- avoid Migen/LiteX dependencies.
- etc...
"""

import argparse

from migen import *

from litex.build.generic_platform import *
from litex.build.xilinx.platform import XilinxPlatform
from litex.build.altera.platform import AlteraPlatform
from litex.build.lattice.platform import LatticePlatform

from litex.soc.interconnect import wishbone
from litex.soc.integration.soc import SoCBusHandler, SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litex.soc.cores.spi.spi_bone import SPIBone

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk / Rst.
    ("clk", 0, Pins(1)),
    ("rst", 1, Pins(1)),

    # SPI.
    ("spi", 0,
        Subsignal("clk",  Pins(1)),
        Subsignal("cs_n", Pins(1)),
        Subsignal("mosi", Pins(1)),
        Subsignal("miso", Pins(1)),
    )
]

# SPIBone Core -------------------------------------------------------------------------------------

class SPIBoneCore(SoCMini):
    def __init__(self, platform):
        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = CRG(platform.request("clk"), platform.request("rst"))

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, clk_freq=int(1e6))

        # SPIBone ----------------------------------------------------------------------------------
        self.submodules.spibone = spibone = SPIBone(platform.request("spi"))

        # MMAP Bus ---------------------------------------------------------------------------------
        platform.add_extension(spibone.bus.get_ios("bus"))
        self.comb += spibone.bus.connect_to_pads(self.platform.request("bus"), mode="master")

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="SPIBone standalone core generator.")
    parser.add_argument("--vendor", default="lattice", help="FPGA Vendor.")
    args = parser.parse_args()

    # Convert/Check Arguments ----------------------------------------------------------------------
    platform_cls = {
        "xilinx"  : XilinxPlatform,
        "altera"  : AlteraPlatform,
        "intel"   : AlteraPlatform,
        "lattice" : LatticePlatform
    }[args.vendor]

    # Generate core --------------------------------------------------------------------------------
    platform = platform_cls(device="", io=_io)
    core     = SPIBoneCore(platform)
    builder  = Builder(core, output_dir="build")
    builder.build(build_name=f"spibone_core_{args.vendor}", run=False)

if __name__ == "__main__":
    main()
