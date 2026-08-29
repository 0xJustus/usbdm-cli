# Third-party components

The combined binary is **GPL-2.0-or-later** (see `LICENSE`), forced by the GPL
USBDM sources it derives from.

| Component | Where | License | Notes |
|---|---|---|---|
| **libusb** 1.0.27 | `vendor/libusb/` | LGPL-2.1-or-later | Statically linked; full source vendored (LGPL relink). `-Dsystem-libusb` links it dynamically. |
| **USBDM** `Commands.h` | `vendor/usbdm/` | GPL-2.0-or-later | Protocol header, compiled in. |
| **USBDM** flash routines | `vendor/flash-routines/*.s19` | GPL-2.0-or-later | RAM routine blobs, embedded. |
| **USBDM** device database | `vendor/usbdm-devices/*.xml` | GPL-2.0-or-later | Parsed at build time; not in the binary. |

Upstream: [libusb](https://libusb.info/), [USBDM](https://github.com/podonoghue).
