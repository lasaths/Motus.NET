#pragma once

#include "motus_native.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTUS_FCL_OK MOTUS_STATUS_OK
#define MOTUS_FCL_ERR MOTUS_STATUS_ERR
#define MOTUS_FCL_UNAVAILABLE MOTUS_STATUS_UNAVAILABLE

MOTUS_NATIVE_API int motus_fcl_is_available(void);

typedef struct motus_fcl_world motus_fcl_world;

MOTUS_NATIVE_API motus_fcl_world* motus_fcl_world_create(void);
MOTUS_NATIVE_API void motus_fcl_world_destroy(motus_fcl_world* world);

/* Geometry is copied at registration; buffers are not retained. */
MOTUS_NATIVE_API int motus_fcl_add_obstacle_box(
    motus_fcl_world* world, uint32_t id, const motus_transform* pose,
    double half_x, double half_y, double half_z);

MOTUS_NATIVE_API int motus_fcl_attach_box(
    motus_fcl_world* world, uint32_t id, const motus_transform* world_pose,
    double half_x, double half_y, double half_z);

MOTUS_NATIVE_API int motus_fcl_detach(motus_fcl_world* world, uint32_t id);

MOTUS_NATIVE_API int motus_fcl_remove(motus_fcl_world* world, uint32_t id);

MOTUS_NATIVE_API int motus_fcl_upsert_sphere(
    motus_fcl_world* world, uint32_t id, const motus_transform* pose, double radius);

MOTUS_NATIVE_API int motus_fcl_upsert_box(
    motus_fcl_world* world, uint32_t id, const motus_transform* pose,
    double half_x, double half_y, double half_z);

MOTUS_NATIVE_API int motus_fcl_upsert_capsule(
    motus_fcl_world* world, uint32_t id, const motus_transform* pose,
    double radius, double half_length);

MOTUS_NATIVE_API int motus_fcl_set_robot_link(
    motus_fcl_world* world, int link_index, const motus_transform* link_pose,
    const float* vertices, uint32_t vertex_count, const uint32_t* indices, uint32_t index_count);

MOTUS_NATIVE_API int motus_fcl_set_allowed_pair(motus_fcl_world* world, uint32_t a, uint32_t b);

MOTUS_NATIVE_API int motus_fcl_check(motus_fcl_world* world, int* out_a, int* out_b);

#ifdef __cplusplus
}
#endif
