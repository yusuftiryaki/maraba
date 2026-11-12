"""Running external tools (st-info, openocd, pio, git, lsusb)."""
from __future__ import annotations

import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass


@dataclass
class CmdResult:
    returncode: int
    output: str                # stdout + stderr: OpenOCD logs to stderr

    @property
    def ok(self) -> bool:
        return self.returncode == 0

    def tail(self, n: int = 15) -> str:
        return '\n'.join(self.output.rstrip().splitlines()[-n:])


class RealRunner:
    def which(self, tool: str) -> str | None:
        return shutil.which(tool)

    def run(self, cmd: list[str], cwd: str | None = None, timeout: float = 60,
            stream: bool = False) -> CmdResult:
        """stream=True echoes lines as they come (builds take a minute)."""
        try:
            if not stream:
                p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True,
                                   timeout=timeout)
                return CmdResult(p.returncode, (p.stdout or '') + (p.stderr or ''))
            p = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, bufsize=1)
            lines = []
            assert p.stdout is not None
            for line in p.stdout:
                lines.append(line)
                sys.stdout.write('    │ ' + line)
                sys.stdout.flush()
            return CmdResult(p.wait(timeout=timeout), ''.join(lines))
        except FileNotFoundError:
            return CmdResult(127, f'{cmd[0]}: komut bulunamadı')
        except subprocess.TimeoutExpired:
            return CmdResult(124, f'{shlex.join(cmd)}: {timeout:.0f} sn içinde bitmedi')
