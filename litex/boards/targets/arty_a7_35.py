#!/usr/bin/env python3

import argparse

from migen import *

from litex.boards.platforms import arty_a7_35
from litex.soc.cores import uart
from litex.soc.cores.gpio import *

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import mem_decoder
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *

from litedram.modules import MT41K128M16
from litedram.phy import s7ddrphy

from liteeth.phy.mii import LiteEthPHYMII
from liteeth.core.mac import LiteEthMAC

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_sys4x = ClockDomain(reset_less=True)
        self.clock_domains.cd_sys4x_dqs = ClockDomain(reset_less=True)
        self.clock_domains.cd_clk200 = ClockDomain()

        self.submodules.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(~platform.request("cpu_reset"))
        pll.register_clkin(platform.request("clk100"), 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.create_clkout(self.cd_sys4x, 4*sys_clk_freq)
        pll.create_clkout(self.cd_sys4x_dqs, 4*sys_clk_freq, phase=90)
        pll.create_clkout(self.cd_clk200, 200e6)

        self.submodules.idelayctrl = S7IDELAYCTRL(self.cd_clk200)

        eth_clk = Signal()
        self.specials += [
            Instance("BUFR", p_BUFR_DIVIDE="4", i_CE=1, i_CLR=0, i_I=self.cd_sys.clk, o_O=eth_clk),
            Instance("BUFG", i_I=eth_clk, o_O=platform.request("eth_ref_clk")),
        ]

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCSDRAM):
    csr_map = {
        "ddrphy":       16,
        "uart_phy0" :   17,
        "uart0" :       18,
        "uart_phy1" :   19,
        "uart1" :       20,
        "switches" :    21,         
    }
    csr_map.update(SoCSDRAM.csr_map)   
    interrupt_map = {
        "uart0": 3,
        "uart1": 4,
    }
   
    interrupt_map.update(SoCSDRAM.interrupt_map)    
    def __init__(self, **kwargs):
        platform = arty_a7_35.Platform()
        sys_clk_freq = int(100e6)
        SoCSDRAM.__init__(self, platform, clk_freq=sys_clk_freq,
                         integrated_rom_size=0x8000,
                         integrated_sram_size=0x8000,
                         **kwargs)

        self.submodules.crg = _CRG(platform, sys_clk_freq)

     

        # sdram
        self.submodules.ddrphy = s7ddrphy.A7DDRPHY(platform.request("ddram"), sys_clk_freq=sys_clk_freq)
        sdram_module = MT41K128M16(sys_clk_freq, "1:4")
        self.register_sdram(self.ddrphy,
                            sdram_module.geom_settings,
                            sdram_module.timing_settings)

        self.submodules.uart_phy0 = uart.RS232PHY(platform.request("uart",0), self.clk_freq, 115200)
        self.submodules.uart0 = ResetInserter()(uart.UART(self.uart_phy0))

        self.submodules.uart_phy1 = uart.RS232PHY(platform.request("uart",1), self.clk_freq, 115200)
        self.submodules.uart1 = ResetInserter()(uart.UART(self.uart_phy1))

        leds = Cat(iter([platform.request("user_led", i) for i in range(4)]))
        self.submodules.leds = GPIOOut(leds)

        switches = Cat(iter([platform.request("user_sw", i) for i in range(4)]))
        self.submodules.switches = GPIOIn(switches)

        buttons = Cat(iter([platform.request("user_btn", i) for i in range(4)]))
        self.submodules.buttons = GPIOIn(buttons)

# EthernetSoC --------------------------------------------------------------------------------------

class EthernetSoC(BaseSoC):
    csr_map = {
        "ethphy": 22,
        "ethmac": 23
    }
    csr_map.update(BaseSoC.csr_map)

    interrupt_map = {
        "ethmac": 5,
    }
    interrupt_map.update(BaseSoC.interrupt_map)

    mem_map = {
        "ethmac": 0x30000000,  # (shadow @0xb0000000)
    }
    mem_map.update(BaseSoC.mem_map)

    def __init__(self, **kwargs):
        BaseSoC.__init__(self, **kwargs)

        self.submodules.ethphy = LiteEthPHYMII(self.platform.request("eth_clocks"),
                                               self.platform.request("eth"))
        self.submodules.ethmac = LiteEthMAC(phy=self.ethphy, dw=32,
            interface="wishbone", endianness=self.cpu.endianness)
        self.add_wb_slave(mem_decoder(self.mem_map["ethmac"]), self.ethmac.bus)
        self.add_memory_region("ethmac", self.mem_map["ethmac"] | self.shadow_base, 0x2000)

        self.crg.cd_sys.clk.attr.add("keep")
        self.ethphy.crg.cd_eth_rx.clk.attr.add("keep")
        self.ethphy.crg.cd_eth_tx.clk.attr.add("keep")
        self.platform.add_period_constraint(self.crg.cd_sys.clk, 10.0)
        self.platform.add_period_constraint(self.ethphy.crg.cd_eth_rx.clk, 80.0)
        self.platform.add_period_constraint(self.ethphy.crg.cd_eth_tx.clk, 80.0)
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.ethphy.crg.cd_eth_rx.clk,
            self.ethphy.crg.cd_eth_tx.clk)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on Arty A7 35")
    builder_args(parser)
    soc_sdram_args(parser)
    parser.add_argument("--with-ethernet", action="store_true",
                        help="enable Ethernet support")
    args = parser.parse_args()

    cls = EthernetSoC if args.with_ethernet else BaseSoC
    soc = cls(**soc_sdram_argdict(args))
    builder = Builder(soc, **builder_argdict(args))
    builder.build()


if __name__ == "__main__":
    main()
