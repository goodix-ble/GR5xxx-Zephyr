# Copyright (c) 2024 Goodix
# SPDX-License-Identifier: Apache-2.0

import os
import platform
from pathlib import Path
from textwrap import dedent

from west import log
from west.commands import WestCommand


class InstallGpRunner(WestCommand):
    def __init__(self):
        super().__init__(
            "install-gp-runner",
            "install GProgrammer as a flash runner",
            dedent(
                """
            This command will copy `gprogrammer.py` to
            zephyr/scripts/west_commands/runners/ and
            add `gprogrammer` to
            zephyr/scripts/west_commands/runners/__init__.py's
            runner import list."""
            ),
            accepts_unknown_args=False,
        )

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(
            self.name, help=self.help, description=self.description
        )

        parser.add_argument("GPROGRAMMER_DIR", help="GProgrammer installed directory")

        return parser

    def do_run(self, args, unknown_args):
        # check if GR5xxx_console exists
        grconsole_executable = "GR5xxx_console"
        if platform.system() == "Windows":
            grconsole_executable = "GR5xxx_console.exe"

        gp_dir = Path(args.GPROGRAMMER_DIR).resolve()
        if not gp_dir.is_dir:
            gp_dir = gp_dir.parent

        if Path(gp_dir, grconsole_executable).exists():
            log.dbg(f"Found {grconsole_executable} in {gp_dir}")
        else:
            log.err(f"Cannot find {grconsole_executable} in {gp_dir}")
            return

        zephyr_base = os.environ["ZEPHYR_BASE"]
        runner_dir = Path(zephyr_base, "scripts/west_commands/runners")

        # copy gprogrammer.py to runner_dir and write proper GPROGRAMMER_DIR
        gprogrammer_py = Path(Path(__file__).parent, "gprogrammer.py")
        with open(gprogrammer_py, "r") as f:
            content = f.read()

        content = content.replace("{GPROGRAMMER_DIR}", f"{gp_dir}")

        with open(Path(runner_dir, "gprogrammer.py"), "w") as f:
            f.write(content)

        # add "gprogrammer" to __init__.py
        with open(Path(runner_dir, "__init__.py"), "r") as f:
            content = f.readlines()

        lf = content[0].replace(content[0].rstrip(), '')

        if f"    'gprogrammer',{lf}" in content:
            log.dbg("gprogrammer runner already inserted into _names, skip")
            return

        line_number = 0

        while not content[line_number].startswith("_names = ["):
            line_number += 1
        line_number += 1

        while "'gprogrammer'," > content[line_number].strip():
            line_number += 1

        content.insert(line_number, f"    'gprogrammer',{lf}")

        with open(Path(runner_dir, "__init__.py"), "w") as f:
            f.writelines(content)
