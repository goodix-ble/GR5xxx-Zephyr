"""Runner for flashing Goodix GR5xxx BLE SoC with GProgrammer console"""

from runners.core import ZephyrBinaryRunner, RunnerCaps
from pathlib import Path
from platform import system

GPROGRAMMER_DIR = "{GPROGRAMMER_DIR}"

class GprogrammerBinaryRunner(ZephyrBinaryRunner):
    """Runner front-end for GR5xxx"""

    def __init__(self, cfg):
        super().__init__(cfg)

    @classmethod
    def name(cls):
        return "gprogrammer"

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={"flash"})

    @classmethod
    def do_add_parser(cls, parser):
        pass

    @classmethod
    def do_create(cls, cfg, args):
        return GprogrammerBinaryRunner(cfg)

    def do_run(self, command, **kwargs):
        self.ensure_output("bin")

        grconsole_executable = "GR5xxx_console"
        if system() == "Windows":
            grconsole_executable = "GR5xxx_console.exe"

        cmd_flash = [
            grconsole_executable,
            "load",
            self.cfg.bin_file,
            "sector",
            "y",
        ]

        self.check_call(cmd_flash, cwd=Path(GPROGRAMMER_DIR))
