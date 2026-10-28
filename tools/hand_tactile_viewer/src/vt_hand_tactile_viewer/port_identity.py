"""Compare serial aliases without opening or assigning a device."""
import os
import re


def port_identity(port):
    value = str(port).strip()
    match = re.fullmatch(r"(?:\\\\\.\\)?COM(\d+)", value, re.IGNORECASE)
    if match:
        return "com" + str(int(match.group(1)))
    return os.path.normcase(os.path.realpath(value)) if value else ""
