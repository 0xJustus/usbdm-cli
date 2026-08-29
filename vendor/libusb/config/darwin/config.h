/* libusb config for macOS (adapted from the upstream Xcode/config.h). */
#include <AvailabilityMacros.h>
#define DEFAULT_VISIBILITY __attribute__((visibility("default")))
#define ENABLE_LOGGING 1
#if MAC_OS_X_VERSION_MIN_REQUIRED >= 1060
#define HAVE_PTHREAD_THREADID_NP 1
#endif
#define HAVE_NFDS_T 1
#define HAVE_SYS_TIME_H 1
#define PLATFORM_POSIX 1
#define PRINTF_FORMAT(a, b) __attribute__((__format__(__printf__, a, b)))
#define _GNU_SOURCE 1
