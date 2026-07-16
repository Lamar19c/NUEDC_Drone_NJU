"""
flash_stm32.py — STM32F103 串口烧录 (无依赖, 纯 Python + Win32 API)

用法:
    python flash_stm32.py <firmware.bin> [COMx]

需要先将 STM32 进入 bootloader 模式:
    1. BOOT0 跳线帽接到 1
    2. 按一下 RESET
    3. 执行本脚本
    4. 烧录完成后 BOOT0 接回 0, 按 RESET 运行

协议: STM32 USART bootloader (AN2606 / AN3155)
"""

import ctypes
from ctypes import wintypes
import sys
import os
import time
import struct

# ── Win32 API constants ──
GENERIC_READ  = 0x80000000
GENERIC_WRITE = 0x40000000
OPEN_EXISTING = 3
FILE_FLAG_OVERLAPPED = 0x40000000
INVALID_HANDLE_VALUE = wintypes.HANDLE(-1).value

DCB_BINARY   = 0x00000001
DCB_PARITY   = 0x00000002
DCB_EVENPARITY = 0x0400
EVENPARITY   = 2
ONESTOPBIT   = 0
RTS_CONTROL_ENABLE = 0x2000
DTR_CONTROL_ENABLE = 0x0001

PURGE_TXCLEAR = 0x0004
PURGE_RXCLEAR = 0x0008

kernel32 = ctypes.windll.kernel32

# ── STM32 bootloader constants ──
STM32_ACK  = 0x79
STM32_NACK = 0x1F
PAGE_SIZE  = 1024  # STM32F103C8 每页 1KB
FLASH_BASE = 0x08000000

def serial_open(port: str, baud: int) -> int:
    """Open COM port with 8E1. Returns handle."""
    path = rf"\\.\{port}"
    h = kernel32.CreateFileW(
        path, GENERIC_READ | GENERIC_WRITE, 0, None,
        OPEN_EXISTING, 0, None
    )
    if h == INVALID_HANDLE_VALUE:
        err = kernel32.GetLastError()
        raise OSError(f"Cannot open {port} (error {err})")

    # Set comm state
    class DCB(ctypes.Structure):
        _fields_ = [
            ("DCBlength", wintypes.DWORD),
            ("BaudRate",  wintypes.DWORD),
            ("fBinary",   wintypes.DWORD),
            ("fParity",   wintypes.DWORD),
            ("fOutxCtsFlow", wintypes.DWORD),
            ("fOutxDsrFlow", wintypes.DWORD),
            ("fDtrControl",  wintypes.DWORD),
            ("fDsrSensitivity", wintypes.DWORD),
            ("fTXContinueOnXoff", wintypes.DWORD),
            ("fOutX",      wintypes.DWORD),
            ("fInX",       wintypes.DWORD),
            ("fErrorChar", wintypes.DWORD),
            ("fNull",      wintypes.DWORD),
            ("fRtsControl", wintypes.DWORD),
            ("fAbortOnError", wintypes.DWORD),
            ("fDummy2",    wintypes.DWORD),
            ("wReserved",  wintypes.WORD),
            ("XonLim",     wintypes.WORD),
            ("XoffLim",    wintypes.WORD),
            ("ByteSize",   wintypes.BYTE),
            ("Parity",     wintypes.BYTE),
            ("StopBits",   wintypes.BYTE),
            ("XonChar",    ctypes.c_char),
            ("XoffChar",   ctypes.c_char),
            ("ErrorChar",  ctypes.c_char),
            ("EofChar",    ctypes.c_char),
            ("EvtChar",    ctypes.c_char),
            ("wReserved1", wintypes.WORD),
        ]

    dcb = DCB()
    dcb.DCBlength = ctypes.sizeof(DCB)
    if not kernel32.GetCommState(h, ctypes.byref(dcb)):
        raise OSError("GetCommState failed")

    dcb.BaudRate = baud
    dcb.ByteSize = 8
    dcb.Parity   = EVENPARITY  # STM32 bootloader requires 8E1
    dcb.StopBits = ONESTOPBIT
    dcb.fBinary  = 1
    dcb.fParity  = 1
    dcb.fDtrControl = DTR_CONTROL_ENABLE
    dcb.fRtsControl = RTS_CONTROL_ENABLE

    if not kernel32.SetCommState(h, ctypes.byref(dcb)):
        raise OSError("SetCommState failed")

    # Set timeouts
    class COMMTIMEOUTS(ctypes.Structure):
        _fields_ = [
            ("ReadIntervalTimeout",         wintypes.DWORD),
            ("ReadTotalTimeoutMultiplier",  wintypes.DWORD),
            ("ReadTotalTimeoutConstant",    wintypes.DWORD),
            ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
            ("WriteTotalTimeoutConstant",   wintypes.DWORD),
        ]

    to = COMMTIMEOUTS()
    to.ReadIntervalTimeout         = 50     # inter-char timeout
    to.ReadTotalTimeoutMultiplier  = 10
    to.ReadTotalTimeoutConstant    = 500    # total read timeout
    to.WriteTotalTimeoutMultiplier = 10
    to.WriteTotalTimeoutConstant   = 500
    kernel32.SetCommTimeouts(h, ctypes.byref(to))

    return h

def serial_close(h: int):
    kernel32.CloseHandle(h)

def serial_write(h: int, data: bytes):
    written = wintypes.DWORD(0)
    kernel32.WriteFile(h, data, len(data), ctypes.byref(written), None)
    return written.value

def serial_read(h: int, n: int, timeout_ms: int = 1000) -> bytes:
    """Read up to n bytes with timeout."""
    result = bytearray()
    start = time.time()
    while len(result) < n:
        if time.time() - start > timeout_ms / 1000.0:
            break
        buf = (ctypes.c_uint8 * (n - len(result)))()
        read = wintypes.DWORD(0)
        kernel32.ReadFile(h, buf, len(buf), ctypes.byref(read), None)
        if read.value > 0:
            result.extend(buf[:read.value])
            start = time.time()  # reset timeout on data
        else:
            time.sleep(0.001)
    return bytes(result)

def serial_purge(h: int):
    kernel32.PurgeComm(h, PURGE_TXCLEAR | PURGE_RXCLEAR)


# ── STM32 bootloader protocol ──

def bootloader_sync(h: int) -> bool:
    """Send auto-baud byte and wait for ACK. Multiple tries for robustness."""
    for attempt in range(5):
        serial_purge(h)
        serial_write(h, b'\x7F')
        time.sleep(0.05)
        resp = serial_read(h, 1, timeout_ms=500)
        if len(resp) == 1 and resp[0] == STM32_ACK:
            return True
    return False

def bootloader_cmd(h: int, cmd: int) -> bool:
    """Send command byte and its complement, expect ACK."""
    serial_write(h, bytes([cmd, cmd ^ 0xFF]))
    resp = serial_read(h, 1, timeout_ms=500)
    return len(resp) == 1 and resp[0] == STM32_ACK

def bootloader_read_bytes(h: int, count: int) -> bytes:
    """Read N bytes from bootloader."""
    data = serial_read(h, count + 1, timeout_ms=1000)  # +1 for ACK
    if len(data) >= count:
        return data[:count]
    return data

def bootloader_get_cmd(h: int) -> int:
    """Get command — returns number of bytes to follow, or -1."""
    cmd = serial_read(h, 2, timeout_ms=1000)
    if len(cmd) < 1:
        return -1
    if cmd[0] == STM32_NACK:
        return -1
    return cmd[0]

def bootloader_erase_all(h: int):
    """Erase all flash using standard erase command (0x43)."""
    # Standard erase command: 0x43 0xBC
    serial_purge(h)
    time.sleep(0.01)
    serial_write(h, bytes([0x43, 0x43 ^ 0xFF]))
    resp = serial_read(h, 1, timeout_ms=2000)
    if len(resp) != 1:
        print(f"[ERASE cmd: no resp, got {len(resp)} bytes]", end=" ")
        return False
    if resp[0] != STM32_ACK:
        print(f"[ERASE cmd: got 0x{resp[0]:02X} not ACK]", end=" ")
        return False
    # Global erase: special code 0xFF with XOR 0xFF
    serial_write(h, b'\xFF\xFF')
    resp = serial_read(h, 1, timeout_ms=30000)
    if len(resp) == 1 and resp[0] == STM32_ACK:
        return True
    print(f"[ERASE global: got 0x{resp[0]:02X} not ACK]" if len(resp) > 0 else "[ERASE global: timeout]", end=" ")
    return False

def bootloader_write_memory(h: int, addr: int, data: bytes):
    """Write data to flash at address."""
    # Write memory command: 0x31 0xCE
    if not bootloader_cmd(h, 0x31):
        raise RuntimeError(f"Write command failed at 0x{addr:08X}")

    # Send address (4 bytes, big-endian) + checksum
    addr_bytes = struct.pack('>I', addr)
    addr_xor = addr_bytes[0] ^ addr_bytes[1] ^ addr_bytes[2] ^ addr_bytes[3]
    serial_write(h, addr_bytes + bytes([addr_xor]))
    if serial_read(h, 1, timeout_ms=500) != bytes([STM32_ACK]):
        raise RuntimeError(f"Write address ACK failed at 0x{addr:08X}")

    # Send data: count byte (N-1), data, checksum
    count = len(data) - 1
    data_xor = count
    for b in data:
        data_xor ^= b
    serial_write(h, bytes([count]) + data + bytes([data_xor]))
    if serial_read(h, 1, timeout_ms=2000) != bytes([STM32_ACK]):
        raise RuntimeError(f"Write data ACK failed at 0x{addr:08X}")

def bootloader_go(h: int, addr: int = FLASH_BASE):
    """Jump to user code."""
    if not bootloader_cmd(h, 0x21):
        print("  [WARN] Go command failed, manually reset with BOOT0=0")
        return
    addr_bytes = struct.pack('>I', addr)
    addr_xor = addr_bytes[0] ^ addr_bytes[1] ^ addr_bytes[2] ^ addr_bytes[3]
    serial_write(h, addr_bytes + bytes([addr_xor]))
    resp = serial_read(h, 1, timeout_ms=500)
    if resp == bytes([STM32_ACK]):
        print("  Jumping to user code...")
    else:
        print("  [WARN] Go ACK not received, try manual reset")


def detect_com_port() -> str:
    """Try COM1..COM32 to find STM32 bootloader."""
    for i in range(1, 33):
        port = f"COM{i}"
        try:
            h = serial_open(port, 115200)
        except OSError:
            continue
        print(f"  Trying {port}...", end=" ", flush=True)
        if bootloader_sync(h):
            print("STM32 detected!")
            serial_close(h)
            return port
        serial_close(h)
        print("no response")
    return None


def flash_firmware(port: str, bin_path: str):
    """Main flash routine."""
    with open(bin_path, 'rb') as f:
        firmware = f.read()

    # Pad to page boundary
    if len(firmware) % PAGE_SIZE != 0:
        firmware += b'\xFF' * (PAGE_SIZE - len(firmware) % PAGE_SIZE)

    print(f"  Firmware: {len(firmware)} bytes ({len(firmware)//PAGE_SIZE} pages)")
    print(f"  Opening {port} at 115200 8E1...")
    h = serial_open(port, 115200)

    print("  Synchronizing bootloader...", end=" ", flush=True)
    if not bootloader_sync(h):
        # Try lower baud rates
        for baud in [57600, 38400, 19200, 9600]:
            serial_close(h)
            h = serial_open(port, baud)
            if bootloader_sync(h):
                print(f"OK at {baud} baud")
                break
        else:
            raise RuntimeError(
                "Cannot sync with bootloader.\n"
                "  Make sure:\n"
                "  1. BOOT0 jumper = 1\n"
                "  2. Press RESET\n"
                "  3. USB-TTL connected to PA9(TX)/PA10(RX)"
            )
    else:
        print("OK")

    # Erase
    print("  Erasing flash...", end=" ", flush=True)
    if bootloader_erase_all(h):
        print("OK")
    else:
        raise RuntimeError("Erase failed")

    # Write
    total = len(firmware)
    for offset in range(0, total, PAGE_SIZE):
        addr = FLASH_BASE + offset
        page_data = firmware[offset:offset+PAGE_SIZE]
        pct = (offset + PAGE_SIZE) * 100 // total
        print(f"\r  Writing... {min(offset+PAGE_SIZE, total)}/{total} ({pct}%)", end="", flush=True)
        bootloader_write_memory(h, addr, page_data)
    print(" OK")

    # Verify (quick: just jump and hope; full verify would re-read)
    print("  Flash complete!")

    serial_close(h)


def flash_auto(bin_path: str):
    """Auto-detect STM32 bootloader and flash without closing port."""
    with open(bin_path, 'rb') as f:
        firmware = f.read()
    if len(firmware) % PAGE_SIZE != 0:
        firmware += b'\xFF' * (PAGE_SIZE - len(firmware) % PAGE_SIZE)

    print(f"  Firmware: {len(firmware)} bytes ({len(firmware)//PAGE_SIZE} pages)")

    # Try each COM port
    for i in range(1, 33):
        port = f"COM{i}"
        h = 0
        try:
            h = serial_open(port, 115200)
        except OSError:
            continue

        print(f"  Trying {port}...", end=" ", flush=True)
        time.sleep(0.05)  # wait for DTR/RTS to settle

        # Sync — try multiple baud rates
        synced = False
        for baud in [115200, 57600, 38400, 19200, 9600]:
            if baud != 115200:
                serial_close(h)
                try:
                    h = serial_open(port, baud)
                except OSError:
                    continue
                time.sleep(0.05)
            if bootloader_sync(h):
                synced = True
                break
        if not synced:
            serial_close(h)
            print("no response")
            continue
        print("STM32 detected!")

        # Erase
        print("  Erasing flash...", end=" ", flush=True)
        if not bootloader_erase_all(h):
            serial_close(h)
            print("FAILED")
            continue
        print("OK")

        # Write
        total = len(firmware)
        try:
            for offset in range(0, total, PAGE_SIZE):
                addr = FLASH_BASE + offset
                page_data = firmware[offset:offset+PAGE_SIZE]
                pct = (offset + PAGE_SIZE) * 100 // total
                print(f"\r  Writing... {min(offset+PAGE_SIZE, total)}/{total} ({pct}%)",
                      end="", flush=True)
                bootloader_write_memory(h, addr, page_data)
            print(" OK")
        except Exception as e:
            serial_close(h)
            print(f"\n  ERROR at offset {offset}: {e}")
            return False

        serial_close(h)
        print(f"\n  DONE! Set BOOT0=0 and press RESET to run.")
        return True

    print("\nERROR: No STM32 found in bootloader mode.")
    print("  Check: BOOT0=1, press RESET, USB-TTL on PA9(TX)/PA10(RX)")
    return False


def main():
    bin_path = sys.argv[1] if len(sys.argv) > 1 else "Debug/stm32_uwb.bin"
    port     = sys.argv[2] if len(sys.argv) > 2 else None

    if not os.path.exists(bin_path):
        print(f"ERROR: Firmware not found: {bin_path}")
        print(f"  Looking in: {os.path.abspath(bin_path)}")
        sys.exit(1)

    print("=" * 56)
    print("  STM32F103 Serial Flasher (Win32 API)")
    print("=" * 56)
    print(f"  Binary: {bin_path}")

    if port:
        print(f"  Port:   {port} (user-specified)")
        print("  (Make sure BOOT0=1, then press RESET)")
        flash_firmware(port, bin_path)
        print(f"\n  DONE. Set BOOT0=0 and press RESET to run.")
    else:
        print("  Scanning for STM32 bootloader...")
        print("  (Make sure BOOT0=1, then press RESET)")
        if not flash_auto(bin_path):
            sys.exit(1)


if __name__ == '__main__':
    main()
