#!/usr/bin/env python3
"""Regenerate tb/REPORT/ plus the top-level DV dashboard from DV_REPORT.json."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


TB = Path(__file__).resolve().parents[1]
GEN = Path.home() / ".codex" / "skills" / "dv-workflow" / "scripts" / "dv_report_gen.py"


def main() -> int:
    cmd = [sys.executable, str(GEN), "--tb", str(TB)]
    subprocess.run(cmd, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
