import subprocess

from litex.build.generic_programmer import GenericProgrammer


class USBBlaster(GenericProgrammer):
    needs_bitreverse = False

    def __init__(self, cable_name="USB-Blaster", device_id=1):
        self.cable_name = cable_name
        self.device_id = device_id

    def load_bitstream(self, bitstream_file, cable_suffix=""):
        subprocess.call(["quartus_pgm", "-m", "jtag", "-c",
                         "{}{}".format(self.cable_name, cable_suffix), "-o",
                         "p;{}@{}".format(bitstream_file, self.device_id)])
