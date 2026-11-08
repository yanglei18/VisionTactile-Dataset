"""Stable pure-wheel containers without changing package or RECORD contents."""
from datetime import datetime, timezone
from pathlib import Path
import os
import zipfile


SOURCE_DATE_EPOCH = 1700000000


def normalize_wheel(path):
    """Use platform-independent ZIP headers and no zlib-dependent compression."""
    path = Path(path)
    temporary = path.with_suffix(".normalizing")
    timestamp = datetime.fromtimestamp(SOURCE_DATE_EPOCH, timezone.utc).timetuple()[:6]
    try:
        with zipfile.ZipFile(path) as source, zipfile.ZipFile(temporary, "w") as destination:
            if len(source.namelist()) != len(set(source.namelist())):
                raise ValueError("Wheel contains duplicate members")
            for name in sorted(source.namelist()):
                if name.endswith("/"):
                    continue
                entry = zipfile.ZipInfo(name, date_time=timestamp)
                entry.create_system = 3
                entry.external_attr = 0o100644 << 16
                destination.writestr(entry, source.read(name), compress_type=zipfile.ZIP_STORED)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
