#!/usr/bin/env python3
"""Reproducible cross-build. Does not connect to or flash any device."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parent

def build(target: str, prefix: str, output: Path, jobs: int) -> None:
    slave = target == "slave"
    project = ROOT / ("IMU_SLV_MCU" if slave else "IMU_MST_2")
    family = "STM32C0xx" if slave else "STM32F4xx"
    cpu = ["-mcpu=cortex-m0plus"] if slave else ["-mcpu=cortex-m4", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
    flags = [*cpu, "-mthumb", "-std=gnu11", "-Os", "-g3", "-ffunction-sections", "-fdata-sections", "-Wall", "-Wextra", "-DUSE_HAL_DRIVER", "-D" + ("STM32C011xx" if slave else "STM32F446xx")]
    includes = [project / p for p in ("Core/Inc", f"Drivers/{family}_HAL_Driver/Inc", f"Drivers/{family}_HAL_Driver/Inc/Legacy", f"Drivers/CMSIS/Device/ST/{family}/Include", "Drivers/CMSIS/Include")]
    sources = list((project / "Core/Src").glob("*.c")) + list((project / f"Drivers/{family}_HAL_Driver/Src").glob("*.c")) + list((project / "Core/Startup").glob("*.s"))
    if not slave:
        usb = project / "Middlewares/ST/STM32_USB_Device_Library"
        includes += [project / "USB_DEVICE/App", project / "USB_DEVICE/Target", usb / "Core/Inc", usb / "Class/CDC/Inc"]
        sources += list((project / "USB_DEVICE").rglob("*.c")) + list(usb.rglob("*.c"))
    directory = output / target
    directory.mkdir(parents=True, exist_ok=True)
    objects = [directory / (str(p.relative_to(project)).replace("/", "_") + ".o") for p in sorted(sources)]
    cc = prefix + "gcc"
    if not shutil.which(cc):
        raise SystemExit(f"Compiler not found: {cc}; pass --prefix /path/to/bin/arm-none-eabi-")
    def compile_one(item):
        source, obj = item
        subprocess.run([cc, *flags, *["-I" + str(p) for p in includes], "-c", str(source), "-o", str(obj)], check=True)
    with ThreadPoolExecutor(max_workers=jobs) as pool:
        list(pool.map(compile_one, zip(sorted(sources), objects)))
    linker = project / ("STM32C011F6UX_FLASH.ld" if slave else "STM32F446RCTX_FLASH.ld")
    elf = directory / (target + ".elf")
    subprocess.run([cc, *cpu, "-mthumb", "--specs=nano.specs", "--specs=nosys.specs", "-T" + str(linker), "-Wl,--gc-sections", "-Wl,-Map=" + str(directory / (target + ".map")), "-o", str(elf), *map(str, objects), "-Wl,--start-group", "-lc", "-lm", "-Wl,--end-group"], check=True)
    subprocess.run([prefix + "objcopy", "-O", "binary", str(elf), str(directory / (target + ".bin"))], check=True)
    subprocess.run([prefix + "size", str(elf)], check=True)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", choices=("slave", "master", "all"), default="all")
    parser.add_argument("--prefix", default="arm-none-eabi-")
    parser.add_argument("--output", type=Path, default=ROOT / "build")
    parser.add_argument("--jobs", type=int, default=4)
    args = parser.parse_args()
    for target in ("slave", "master") if args.target == "all" else (args.target,):
        build(target, args.prefix, args.output.resolve(), args.jobs)
