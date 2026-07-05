#pragma once

#ifdef _WIN32
#  define MOTUS_NATIVE_API __declspec(dllexport)
#else
#  define MOTUS_NATIVE_API __attribute__((visibility("default")))
#endif

#define MOTUS_STATUS_OK 0
#define MOTUS_STATUS_ERR -1
#define MOTUS_STATUS_UNAVAILABLE -2

/* Row-major 4x4 homogeneous transform (m[row*4+col]). */
typedef struct motus_transform {
    double m[16];
} motus_transform;

#ifdef __cplusplus
extern "C" {
#endif

MOTUS_NATIVE_API const char* motus_last_error(void);

#ifdef __cplusplus
}
#endif
