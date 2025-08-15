#pragma once

#if defined(__APPLE__)
#include <libkern/OSByteOrder.h>

#define htole16(x) OSSwapHostToLittleInt16(x)
#define htole32(x) OSSwapHostToLittleInt32(x)
#define htole64(x) OSSwapHostToLittleInt64(x)

#define le16toh(x) OSSwapLittleToHostInt16(x)
#define le32toh(x) OSSwapLittleToHostInt32(x)
#define le64toh(x) OSSwapLittleToHostInt64(x)

#elif defined(__linux__)
#include <endian.h> // or #include <sys/endian.h> on older systems

#else
#error "Endian conversion functions not defined for this platform"
#endif