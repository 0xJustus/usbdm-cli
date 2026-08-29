# usbdm-cli

A Zig command-line tool for the **FZ0622C** USBDM programmer/debugger.

## Commands

```text
usbdm list [--all]                        # enumerate BDM interfaces
usbdm version | probe | status            # firmware / capabilities / connection
usbdm connect --target hcs08              # bring up the target (retry ladder)
usbdm identify --target hcs12             # read the SDID, name the part(s)
usbdm reset [--normal|--special] [--hardware|--software|--power]
usbdm halt | go | step
usbdm regs | reg pc [0x1234]              # read/write core registers
usbdm read 0x1000 0x100 | write 0x80 aa bb | fill 0x80 0x40 0xff
usbdm dump 0x8000 0x4000 -o fw.s19        # bin/srec/ihex by --format or suffix
usbdm load image.s19                      # write an image into RAM (not flash)
usbdm pins bkgd=low reset=3state | release

usbdm program app.s19 --part mc9s08jm60   # erase affected sectors, program, verify
usbdm verify | erase [0xADDR] | blank-check --part ...
usbdm unsecure | secure --part ...        # clear / set flash security
usbdm eeprom img.s19 --part mc9s12g240    # EEPROM/data-flash (S12G GMMC parts)
usbdm gdb --port 1234                     # GDB remote stub over TCP
```

Exit codes: `0` ok, `2` usage, `3` no device, `4` BDM/target, `5` flash,
`6` verify mismatch, `7` file I/O.

## Build

Requires **Zig 0.16**.

```sh
zig build                              # -> zig-out/bin/usbdm
zig build test                         # run tests
zig build -Dtarget=x86_64-linux-gnu    # cross-compile
zig build -Dsystem-libusb              # link system libusb
```

Builds natively on Linux/macOS/Windows. Windows uses WinUSB.

## `--sim`

Runs any command against a software virtual target. A CPU emulator
executes the real vendored flash routines, so `program` and `gdb` single-step
behave as on-target. USB-only commands (`probe`/`version`/`list`) are declined.

## Flash programming

Reference RAM flash-routine method: vendored routines run on the target CPU.
Geometry (`--part`) comes from a build-time table of 188 parts (vendored USBDM
device DB), which also backs `identify`. Families: HCS08/RS08, HCS12, CFV1, plus
S12G EEPROM. Destructive commands confirm unless `--force`.

## GDB remote stub

`usbdm gdb` serves the GDB remote protocol over TCP: registers, memory,
single-step, `continue`/`vCont`, flash `load` (`vFlash*`), `monitor reset`/`halt`.
Hardware breakpoints on HCS08, classic-S12 HCS12, and CFV1; S12G and watchpoints
use GDB software breakpoints.

## Layout

- `usb` / `protocol` / `transport` / `session` - libusb wrapper, command set,
  bulk framing, typed commands
- `target` / `device` - register/status decode; per-device flash+RAM geometry + SDID map
- `ramflash` / `flash` - RAM flash-routine engine; BDM byte access
- `hwbreak` / `gdb` / `gdbstub` - hardware breakpoints; GDB RSP dispatch + framing
- `hexfile` / `hexdump` / `tty` - S-record/IHEX codec; output formatting
- `sim` - software virtual dongle+target (`--sim`)
- `cpu08` / `cfv1` / `cpu12` - functional CPU + flash-module emulators
- `main` - CLI + GDB TCP server
- `vendor/` - pinned upstream: libusb, `Commands.h`, flash-routine blobs, device XML

## License

GPL-2.0-or-later. See [`LICENSE`](LICENSE) and
[`THIRD_PARTY_LICENSES.md`](THIRD_PARTY_LICENSES.md).
