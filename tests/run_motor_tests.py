"""Run actual EMM bus/motor and UART BSP C code on ARM under Unicorn.

The HAL shim exercises DMA event/ring bookkeeping, not real peripheral timing.
Uses the same local Unicorn dependency as run_task_monitor_test.py.
"""

from pathlib import Path
import struct
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "build" / "test-deps"))
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB  # noqa: E402
from unicorn.arm_const import UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_SP  # noqa: E402

output = root / "build" / "motor-test"
output.mkdir(parents=True, exist_ok=True)
elf = output / "motor_test.elf"
subprocess.run([
    "arm-none-eabi-gcc", "-mcpu=cortex-m3", "-mthumb", "-std=c11", "-Os",
    "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections",
    "-I", str(root / "tests" / "stubs"),
    "-I", str(root / "bsp" / "uart"),
    "-I", str(root / "modules" / "motor" / "emm"),
    "-I", str(root / "application" / "chassis"),
    "-I", str(root / "modules" / "host_link"),
    "-I", str(root / "bsp" / "usb"),
    "-I", str(root / "system"),
    "-I", str(root / "board"),
    "-I", str(root / "application"),
    str(root / "bsp" / "uart" / "bsp_uart.c"),
    str(root / "modules" / "motor" / "emm" / "emm_bus.c"),
    str(root / "modules" / "motor" / "emm" / "emm_motor.c"),
    str(root / "application" / "robot.c"),
    str(root / "application" / "chassis" / "chassis_odometry.c"),
    str(root / "application" / "chassis" / "chassis_position_control.c"),
    str(root / "application" / "chassis" / "omni_kinematics.c"),
    str(root / "application" / "chassis" / "chassis_control.c"),
    str(root / "modules" / "host_link" / "host_protocol.c"),
    str(root / "modules" / "host_link" / "vofa_protocol.c"),
    str(root / "bsp" / "usb" / "bsp_usb.c"),
    str(root / "system" / "host_service.c"),
    str(root / "tests" / "host_link_test.c"),
    str(root / "tests" / "omni_kinematics_test.c"),
    str(root / "tests" / "host_control_test.c"),
    str(root / "tests" / "emm_motor_test.c"),
    str(root / "tests" / "bsp_uart_test.c"),
    "-nostartfiles", "--specs=nosys.specs",
    "-Wl,-Ttext=0x10000,-e,RunMotorTests,--gc-sections", "-lm", "-o", str(elf),
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
machine.emu_start(entry | 1, stop, count=150000000)
assert machine.reg_read(UC_ARM_REG_PC) == stop, "Test exceeded instruction budget"
result = machine.reg_read(UC_ARM_REG_R0)
source = ("host_link_test.c" if result >= 60000 else
          "host_control_test.c" if result >= 50000 else
          "omni_kinematics_test.c" if result >= 30000 else
          "bsp_uart_test.c" if result >= 10000 else "emm_motor_test.c")
line = result % 10000
assert result == 0, f"Motor test failed at tests/{source}:{line}"
print("PASS: EMM framing, four addresses, stop priority, command lease, feedback, timeout/wrap;")
print("      UART DMA wrap/events, ring overflow, TX ownership/deadline and HAL error recovery")
print("      X omni kinematics: geometric projection, unscaled forward/inverse, invalid input")
print("      USB CRC/framing, FIFO overflow, TX ownership, automatic velocity start, ten-second held target, arbitrary sequence, STOP, reconnect and device faults")
print("      VOFA FireWater text/CRLF, integer bounds, fixed decimals, 22 channels, validity masks and mode selection")

print("      USB absolute position goal convergence, mode switching, STOP/link/device/pose cancellation and shortest yaw")
