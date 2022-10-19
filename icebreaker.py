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
from litex.soc.interconnect import wishbone
from litex.soc.interconnect import axi
from litex.soc.cores.spi.spi_bone import SPIBone
from litex.soc.cores.led import LedChaser
from litex.soc.cores.gpio import GPIOIn

from litex_boards.platforms import icebreaker
from litex_boards.targets.icebreaker import _CRG

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
    def __init__(self, sys_clk_freq=int(50e6), mode="litex"):
        platform = icebreaker.Platform()
        platform.add_extension(icebreaker.break_off_pmod)
        platform.add_extension(rp2040_spi_ios(pmod="PMOD1A"))

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident         = "RP2040 PMOD <> LiteX SoC proof of concept on iCEBreaker.",
            ident_version = True
        )

        # RP2040 <-> MMAP over SPIBone -------------------------------------------------------------
        assert mode in ["litex", "hybrid-wishbone", "hybrid-axi-lite"]

        # Full LiteX flow: Directly integrate SPIBone to the LiteX SoC.
        if mode == "litex":
            self.submodules.spibone = SPIBone(platform.request("rp2040_spi"))
            self.bus.add_master(name="rp2040", master=self.spibone.bus)

        # Hybrid LiteX flow: First generate SPIBone core with Wishbone interface and re-integrate
        # it to the LiteX SoC.
        if mode == "hybrid-wishbone":
            os.system(f"./spibone_gen.py --bus-standard=wishbone --vendor=lattice")
            bus = wishbone.Interface(address_width=32, data_width=32)
            spi = platform.request("rp2040_spi")
            self.specials += Instance("spibone_core",
                # Clk/Rst.
                i_clk = ClockSignal("sys"),
                i_rst = ResetSignal("sys"),

                # RP2040 SPI.
                i_spi_clk   = spi.clk,
                i_spi_cs_n  = spi.cs_n,
                i_spi_mosi  = spi.mosi,
                io_spi_miso = spi.miso,

                # Wishbone Bus.
                o_bus_adr   = bus.adr,
                o_bus_dat_w = bus.dat_w,
                i_bus_dat_r = bus.dat_r,
                o_bus_sel   = bus.sel,
                o_bus_cyc   = bus.cyc,
                o_bus_stb   = bus.stb,
                i_bus_ack   = bus.ack,
                o_bus_we    = bus.we,
                o_bus_cti   = bus.cti,
                o_bus_bte   = bus.bte,
                i_bus_err   = bus.err,
            )
            self.bus.add_master(name="rp2040", master=bus)
            platform.add_source("build/gateware/spibone_core.v")

        # Hybrid LiteX flow: First generate SPIBone core with AXI-Lite interface and re-integrate
        # it to the LiteX SoC.
        if mode == "hybrid-axi-lite":
            os.system(f"./spibone_gen.py --bus-standard=axi-lite --vendor=lattice")
            bus = axi.AXILiteInterface(address_width=32, data_width=32)
            spi = platform.request("rp2040_spi")
            self.specials += Instance("spibone_core",
                # Clk/Rst.
                i_clk = ClockSignal("sys"),
                i_rst = ResetSignal("sys"),

                # RP2040 SPI.
                i_spi_clk   = spi.clk,
                i_spi_cs_n  = spi.cs_n,
                i_spi_mosi  = spi.mosi,
                io_spi_miso = spi.miso,

                # AXI-Lite Bus.
                # AW.
                o_bus_awaddr   = bus.aw.addr,
                o_bus_awvalid  = bus.aw.valid,
                i_bus_awready  = bus.aw.ready,

                # W.
                o_bus_wdata    = bus.w.data,
                o_bus_wstrb    = bus.w.strb,
                o_bus_wvalid   = bus.w.valid,
                i_bus_wready   = bus.w.ready,

                # B.
                i_bus_bresp    = bus.b.resp,
                i_bus_bvalid   = bus.b.valid,
                o_bus_bready   = bus.b.ready,

                # AR.
                o_bus_araddr   = bus.ar.addr,
                o_bus_arvalid  = bus.ar.valid,
                i_bus_arready  = bus.ar.ready,

                # R.
                i_bus_rdata    = bus.r.data,
                i_bus_rresp    = bus.r.resp,
                i_bus_rvalid   = bus.r.valid,
                o_bus_rready   = bus.r.ready,
            )
            self.bus.add_master(name="rp2040", master=bus)
            platform.add_source("build/gateware/spibone_core.v")

        # Leds -------------------------------------------------------------------------------------
        self.submodules.leds = LedChaser(
            pads         = platform.request_all("user_led"),
            polarity     = 1,
            sys_clk_freq = sys_clk_freq)

        # Buttons ----------------------------------------------------------------------------------
        self.submodules.buttons = GPIOIn(
            pads = platform.request_all("user_btn")
        )

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
    parser.add_argument("--sys-clk-freq", default=50e6,        help="System clock frequency (default: 50MHz).")
    parser.add_argument("--mode",         default="litex",     help="Choose SPIBone instance mode(litex, hybrid-wishbone or hybrid-axi-lite).")
    args = parser.parse_args()

    soc = RP2040PMODTestSoC(
        sys_clk_freq = int(float(args.sys_clk_freq)),
        mode         = args.mode,
    )
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
