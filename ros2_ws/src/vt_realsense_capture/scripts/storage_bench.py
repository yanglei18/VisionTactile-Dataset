#!/usr/bin/python3
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable


BENCHMARK_DATA_FILENAME = ".vt_storage_bench.data"
BLOCK_SIZE_BYTES = 4 * 1024 * 1024
BENCHMARK_SIZE_BYTES = 16 * 1024 * 1024 * 1024
MINIMUM_MB_PER_SECOND = 540.0
MINIMUM_BYTES_PER_SECOND = 540_000_000
MAXIMUM_BYTES_PER_SECOND = (1 << 64) - 1
_REPOSITORY_ROOT = Path(__file__).resolve().parents[4]


@dataclass(frozen=True)
class BenchmarkResult:
    passed: bool
    bytes_per_second: int | None
    megabytes_per_second: float | None
    report_path: Path
    error: str | None


def validate_output_root(output_root: Path | str) -> Path:
    """Return a safe benchmark root outside the source repository."""

    root = Path(output_root)
    if not root.is_absolute():
        raise ValueError("output root must be absolute")
    if root.is_symlink():
        raise ValueError("output root must not be a symlink")
    if root.resolve(strict=False) != root:
        raise ValueError("output root must be canonical")
    if root == Path(root.anchor):
        raise ValueError("filesystem root is an unsafe benchmark target")
    if not root.is_dir():
        raise ValueError("output root must be an existing directory")
    if root == _REPOSITORY_ROOT or root.is_relative_to(_REPOSITORY_ROOT):
        raise ValueError("output root must be outside the repository")
    return root


def build_fio_command(output_root: Path | str) -> list[str]:
    """Build the fio argv for the production storage benchmark."""

    root = validate_output_root(output_root)
    data_path = root / BENCHMARK_DATA_FILENAME
    if data_path.exists() or data_path.is_symlink():
        raise FileExistsError(
            f"refusing to overwrite existing benchmark data: {data_path}"
        )
    return [
        "fio",
        "--name=vt_storage_bench",
        f"--filename={data_path}",
        "--rw=write",
        "--bs=4M",
        "--size=16G",
        "--direct=1",
        "--ioengine=libaio",
        "--iodepth=1",
        "--numjobs=1",
        "--group_reporting=1",
        "--output-format=json",
    ]


def parse_fio_bandwidth(payload: str | bytes) -> int:
    """Extract one fio job's write ``bw_bytes`` value."""

    try:
        document = json.loads(payload)
    except (ValueError, TypeError, UnicodeDecodeError) as exc:
        raise ValueError("invalid fio JSON output") from exc
    if not isinstance(document, dict):
        raise ValueError("fio JSON root must be an object")
    jobs = document.get("jobs")
    if not isinstance(jobs, list) or len(jobs) != 1:
        raise ValueError("fio JSON must contain exactly one job")
    job = jobs[0]
    if not isinstance(job, dict):
        raise ValueError("fio job must be an object")
    write = job.get("write")
    if not isinstance(write, dict):
        raise ValueError("fio job must contain write statistics")
    bandwidth = write.get("bw_bytes")
    if (
        type(bandwidth) is not int
        or bandwidth < 0
        or bandwidth > MAXIMUM_BYTES_PER_SECOND
    ):
        raise ValueError(
            "fio write.bw_bytes must be a non-negative uint64 integer"
        )
    return bandwidth


def _utc_timestamp(now: datetime) -> str:
    if now.tzinfo is None:
        raise ValueError("benchmark timestamp must be timezone-aware")
    return now.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")


def _atomic_write_text(path: Path, payload: str) -> None:
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
    )
    temporary = Path(temporary_name)
    try:
        stream = os.fdopen(descriptor, "w", encoding="utf-8")
        descriptor = -1
        with stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        temporary.unlink(missing_ok=True)


def run_storage_benchmark(
    output_root: Path | str,
    *,
    run_command: Callable[..., subprocess.CompletedProcess[str]] | None = None,
    now_utc: Callable[[], datetime] | None = None,
) -> BenchmarkResult:
    """Run fio, remove its owned data file, and write the audit report."""

    root = validate_output_root(output_root)
    command = build_fio_command(root)
    data_path = root / BENCHMARK_DATA_FILENAME
    report_path = root / "vt_storage_bench.json"
    runner = run_command or subprocess.run
    clock = now_utc or (lambda: datetime.now(timezone.utc))

    bandwidth: int | None = None
    fio_returncode: int | None = None
    error: str | None = None
    data_path.open("xb").close()
    try:
        try:
            completed = runner(
                command,
                capture_output=True,
                text=True,
                check=False,
                timeout=900.0,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            error = f"fio execution failed: {exc}"
        else:
            fio_returncode = completed.returncode
            if fio_returncode != 0:
                detail = (completed.stderr or "").strip()
                error = f"fio exited with status {fio_returncode}"
                if detail:
                    error += f": {detail}"
            else:
                try:
                    bandwidth = parse_fio_bandwidth(completed.stdout)
                except ValueError as exc:
                    error = str(exc)
    finally:
        data_path.unlink(missing_ok=True)

    megabytes_per_second = (
        None if bandwidth is None else bandwidth / 1_000_000
    )
    passed = (
        error is None
        and bandwidth is not None
        and bandwidth >= MINIMUM_BYTES_PER_SECOND
    )
    report = {
        "schema_version": 1,
        "timestamp_utc": _utc_timestamp(clock()),
        "output_root": str(root),
        "data_path": str(data_path),
        "command": command,
        "command_facts": {
            "operation": "sequential_write",
            "block_size_bytes": BLOCK_SIZE_BYTES,
            "size_bytes": BENCHMARK_SIZE_BYTES,
            "direct": True,
            "ioengine": "libaio",
            "iodepth": 1,
            "numjobs": 1,
        },
        "fio_returncode": fio_returncode,
        "measured_bytes_per_second": bandwidth,
        "measured_mb_per_second": megabytes_per_second,
        "minimum_mb_per_second": MINIMUM_MB_PER_SECOND,
        "passed": passed,
        "error": error,
    }
    _atomic_write_text(
        report_path, json.dumps(report, indent=2, sort_keys=True) + "\n"
    )
    return BenchmarkResult(
        passed=passed,
        bytes_per_second=bandwidth,
        megabytes_per_second=megabytes_per_second,
        report_path=report_path,
        error=error,
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run the 16 GiB direct-write capture storage benchmark."
    )
    parser.add_argument(
        "--output-root",
        required=True,
        type=Path,
        help="Absolute recording filesystem directory outside the repository.",
    )
    arguments = parser.parse_args(argv)
    try:
        result = run_storage_benchmark(arguments.output_root)
    except (OSError, ValueError) as exc:
        print(f"storage benchmark failed: {exc}", file=sys.stderr)
        return 2

    measured = (
        "unavailable"
        if result.megabytes_per_second is None
        else f"{result.megabytes_per_second:.6f} MB/s"
    )
    outcome = "PASS" if result.passed else "FAIL"
    print(f"{outcome}: {measured}; report={result.report_path}")
    if result.error:
        print(result.error, file=sys.stderr)
    return 0 if result.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
