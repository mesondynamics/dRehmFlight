#!/usr/bin/env python3
"""Disable unused CRSF telemetry so its UART flush cannot wait inside the flight loop."""
from pathlib import Path
import re
import sys

root = Path(sys.argv[1]).expanduser() if len(sys.argv) > 1 else Path.home() / "Arduino/libraries/CRSFforArduino"
config = root / "src/CFA_Config.hpp"
original = config.read_text()
updated, count = re.subn(r"(?m)^(#define[ \t]+CRSF_TELEMETRY_ENABLED[ \t]+)1[ \t]*$", r"\g<1>0", original)
if not count and not re.search(r"(?m)^#define[ \t]+CRSF_TELEMETRY_ENABLED[ \t]+0[ \t]*$", original):
    raise SystemExit(f"Could not locate CRSF_TELEMETRY_ENABLED in {config}")
if updated != original:
    config.write_text(updated)
print(f"CRSF_TELEMETRY_ENABLED=0: {config}")
