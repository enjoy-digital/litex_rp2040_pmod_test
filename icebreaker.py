#!/usr/bin/env python3

# RR2040 PMOD <-> LiteX SoC proof of concept.
#
# Copyright (c) 2021-2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse

from migen import *

from litex.build.generic_platform import *

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser

from litex_boards.platforms import icebreaker
from litex_boards.targets.icebreaker import _CRG

from litex.soc.cores.spi.spi_bone import SpiWishboneBridge

# IOs ----------------------------------------------------------------------------------------------

def rp2040_spi_ios(pmod="PMOD1"):
    return [
        ("rp2040_spi", 0,
            Subsignal("clk",  Pins(f"{pmod}:3")), # SPI1 SCK.
            Subsignal("cs_n", Pins(f"{pmod}:0")), # SPI1 CSn.
            Subsignal("mosi", Pins(f"{pmod}:2")), # SPI1 TX.
            Subsignal("miso", Pins(f"{pmod}:1")), # SPI1 RX.
            IOStandard("LVCMOS33"),
        )
    ]

# RP2040PMODTestSoC --------------------------------------------------------------------------------

class RP2040PMODTestSoC(SoCCore):
    def __init__(self, sys_clk_freq=int(50e6)):
        platform = icebreaker.Platform()
        platform.add_extension(rp2040_spi_ios(pmod="PMOD1A"))

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident         = "RP2040 PMOD <> LiteX SoC proof of concept on iCEBreaker.",
            ident_version = True
        )

        # RP2040 <-> MMAP over SPIBone -------------------------------------------------------------
        self.submodules.spibone = SpiWishboneBridge(platform.request("rp2040_spi"))
        self.bus.add_master(name="rp2040", master=self.spibone.wishbone)

        # Leds -------------------------------------------------------------------------------------
        self.submodules.leds = LedChaser(
            pads         = platform.request_all("user_led_n"),
            polarity     = 1,
            sys_clk_freq = sys_clk_freq)

# Flash --------------------------------------------------------------------------------------------

def flash(build_dir, build_name):
    from litex.build.lattice.programmer import IceStormProgrammer
    prog = IceStormProgrammer()
    prog.flash(0x00000000, f"{build_dir}/gateware/{build_name}.bin")

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="RP2040 PMOD <> LiteX SoC proof of concept.")
    parser.add_argument("--build",        action="store_true", help="Build design.")
    parser.add_argument("--load",         action="store_true", help="Load bitstream.")
    parser.add_argument("--flash",        action="store_true", help="Flash Bitstream and BIOS.")
    parser.add_argument("--sys-clk-freq", default=50e6,        help="System clock frequency (default: 50MHz)")
    args = parser.parse_args()

    soc = RP2040PMODTestSoC(sys_clk_freq=int(float(args.sys_clk_freq)))
    builder = Builder(soc, csr_csv="csv.csv")
    if args.build | args.load | args.flash:
        builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram", ext=".bin")) # FIXME

    if args.flash:
        flash(builder.output_dir, soc.build_name)

if __name__ == "__main__":
    main()
