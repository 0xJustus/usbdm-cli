/* libusb config for Windows built with Zig's clang toolchain (not MSVC).
 * Needs a WinUSB/libusbK driver bound to the device (e.g. via Zadig) at runtime. */
#define PLATFORM_WINDOWS 1
#define ENABLE_LOGGING 1
#define DEFAULT_VISIBILITY
#define PRINTF_FORMAT(a, b)
