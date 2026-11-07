"""Compile the real C module for ARM and run its boundary tests in Unicorn.

Requires arm-none-eabi-gcc and Unicorn. Test dependencies can stay local:
python -m pip install --target build/test-deps unicorn==2.1.4
python tests/run_task_monitor_test.py
No hardware, RTOS scheduler or GPIO peripheral is emulated by these module tests.
"""

from pathlib import Path
import struct
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "build" / "test-deps"))
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB  # noqa: E402
from unicorn.arm_const import UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_SP  # noqa: E402

output = root / "build" / "task-monitor-test"
output.mkdir(parents=True, exist_ok=True)
elf = output / "task_monitor_test.elf"
subprocess.run([
    "arm-none-eabi-gcc", "-mcpu=cortex-m3", "-mthumb", "-std=c11", "-Os",
    "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections",
    "-I", str(root / "modules" / "task_monitor"),
    str(root / "modules" / "task_monitor" / "task_monitor.c"),
    str(root / "tests" / "task_monitor_test.c"),
    "-nostartfiles", "--specs=nosys.specs",
    "-Wl,-Ttext=0x10000,-e,RunTaskMonitorTests,--gc-sections", "-o", str(elf),
], check=True)

data = elf.read_bytes()
assert data[:6] == b"\x7fELF\x01\x01", "Expected little-endian ELF32"
entry, phoff = struct.unpack_from("<II", data, 24)
phsize, phcount = struct.unpack_from("<HH", data, 42)
segments = []
pages = set()
for index in range(phcount):
    kind, offset, address, _, file_size, memory_size, _, _ = struct.unpack_from(
        "<8I", data, phoff + index * phsize)
    if kind == 1 and memory_size:
        segments.append((offset, address, file_size))
        pages.update(range(address & ~4095, (address + memory_size + 4095) & ~4095, 4096))

machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
for page in sorted(pages):
    machine.mem_map(page, 4096)
for offset, address, file_size in segments:
    machine.mem_write(address, data[offset:offset + file_size])

machine.mem_map(0x20000000, 0x10000)
stop = 0x40000000
machine.mem_map(stop, 4096)
machine.reg_write(UC_ARM_REG_SP, 0x20010000)
machine.reg_write(UC_ARM_REG_LR, stop | 1)
machine.emu_start(entry | 1, stop, count=1000000)
assert machine.reg_read(UC_ARM_REG_PC) == stop, "Test exceeded instruction budget"
result = machine.reg_read(UC_ARM_REG_R0)
assert result == 0, f"Task monitor test failed at tests/task_monitor_test.c:{result}"
print("PASS: startup, multiple heartbeats, blink timing, timeout/recovery, uint32 wrap, capacity")
