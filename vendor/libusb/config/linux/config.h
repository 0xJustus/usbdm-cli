/* libusb config for Linux (adapted from the upstream android/config.h, which
 * targets Linux). HAVE_LIBUDEV is intentionally NOT defined so the self-
 * contained netlink hotplug backend is used instead of libudev - keeping the
 * binary dependency-free. */
#define DEFAULT_VISIBILITY __attribute__((visibility("default")))
#define ENABLE_LOGGING 1
#define HAVE_ASM_TYPES_H 1
#define HAVE_CLOCK_GETTIME 1
#define HAVE_EVENTFD 1
#define HAVE_TIMERFD 1
#define HAVE_NFDS_T 1
#define HAVE_PIPE2 1
#define HAVE_SYS_TIME_H 1
#define PLATFORM_POSIX 1
#define PRINTF_FORMAT(a, b) __attribute__((__format__(__printf__, a, b)))
#define USE_SYSTEM_LOGGING_FACILITY 1
#define _GNU_SOURCE 1
