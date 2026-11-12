"""Run actual crane USB protocol, Robot arbitration and feedback on ARM; no hardware."""
from pathlib import Path
import struct
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "build" / "test-deps"))
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB  # noqa: E402
from unicorn.arm_const import UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_SP  # noqa: E402

output = root / "build" / "crane-host-test"
output.mkdir(parents=True, exist_ok=True)
elf = output / "crane_host_test.elf"
includes = ["tests/stubs", "bsp/usb", "bsp/uart", "bsp/pwm", "board", "modules/motor/emm", "modules/motor/x",
            "modules/host_link", "modules/ops", "modules/servo", "application", "application/chassis", "system"]
sources = ["modules/motor/x/x_motor.c", "modules/motor/emm/emm_motor.c", "modules/motor/emm/emm_bus.c",
           "modules/host_link/host_protocol.c", "modules/host_link/vofa_protocol.c", "bsp/usb/bsp_usb.c",
           "application/robot.c", "application/crane_action.c", "application/chassis/chassis_control.c",
           "application/chassis/chassis_localization.c", "application/chassis/chassis_position_control.c",
           "application/chassis/omni_kinematics.c", "system/host_service.c", "tests/crane_host_test.c",
           "bsp/uart/bsp_uart.c", "bsp/pwm/bsp_pwm.c", "modules/servo/servo.c",
           "board/board_crane_action.c", "tests/crane_action_test.c",
           "tests/host_link_test.c", "tests/emm_motor_test.c", "tests/bsp_uart_test.c"]
subprocess.run([
    "arm-none-eabi-gcc", "-mcpu=cortex-m3", "-mthumb", "-std=c11", "-Os",
    "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections",
    *[arg for folder in includes for arg in ("-I", str(root / folder))],
    *[str(root / source) for source in sources],
    "-nostartfiles", "--specs=nosys.specs",
    "-Wl,-Ttext=0x10000,-e,RunCraneHostTests,--gc-sections", "-lm", "-o", str(elf),
], check=True)

data = elf.read_bytes()
assert data[:6] == b"\x7fELF\x01\x01", "Expected little-endian ELF32"
entry, phoff = struct.unpack_from("<II", data, 24)
phsize, phcount = struct.unpack_from("<HH", data, 42)
segments, pages = [], set()
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
machine.emu_start(entry | 1, stop, count=40000000)
assert machine.reg_read(UC_ARM_REG_PC) == stop, "Test exceeded instruction budget"
result = machine.reg_read(UC_ARM_REG_R0)
source = ("crane_action_test.c" if result >= 40000 else "bsp_uart_test.c" if result >= 30000 else "emm_motor_test.c" if result >= 20000
          else "host_link_test.c" if result >= 10000 else "crane_host_test.c")
assert result == 0, f"Crane regression failed at tests/{source}:{result % 10000}"
print("PASS: USB crane signed 64-bit targets, per-axis profile config, reserved/range rejection;")
print("      deduplication/replacement, STOP batch priority, link/overflow cancellation;")
print("      0x83 feedback layout/CRC, original STATUS/POSE scheduling and FireWater isolation")
print("      PE5 servo pulse parsing, 1600 us PWM update and out-of-range rejection")
print("      PE6 gripper PWM, shared timer channel isolation, range/length rejection and STOP hold")
print("      existing EMM/UART/USB protocol and FireWater regressions")
print("      pick/place real FD ACK/fresh-arrival sequencing, PWM timing, exclusive ownership and deduplication;")
print("      every-stage STOP/axis STOP/link/overflow/fault/timeout cancellation, config validation and 0x84 feedback")
print("      0x0C per-axis disable golden frames, batch priority, invalid axis/length and every-stage action cancellation")
