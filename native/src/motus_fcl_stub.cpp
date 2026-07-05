#include "motus_fcl.h"

extern void motus_set_last_error(const char*);

int motus_fcl_is_available(void) { return 0; }

motus_fcl_world* motus_fcl_world_create(void) { return nullptr; }
void motus_fcl_world_destroy(motus_fcl_world* world) { (void)world; }

int motus_fcl_add_obstacle_box(
    motus_fcl_world* world, uint32_t id, const motus_transform* pose,
    double half_x, double half_y, double half_z)
{
    (void)world; (void)id; (void)pose; (void)half_x; (void)half_y; (void)half_z;
    motus_set_last_error("FCL not linked");
    return MOTUS_FCL_UNAVAILABLE;
}

int motus_fcl_attach_box(
    motus_fcl_world* world, uint32_t id, const motus_transform* world_pose,
    double half_x, double half_y, double half_z)
{
    (void)world; (void)id; (void)world_pose; (void)half_x; (void)half_y; (void)half_z;
    return MOTUS_FCL_UNAVAILABLE;
}

int motus_fcl_detach(motus_fcl_world* world, uint32_t id)
{
    (void)world; (void)id;
    return MOTUS_FCL_UNAVAILABLE;
}

int motus_fcl_remove(motus_fcl_world* world, uint32_t id)
{
    (void)world; (void)id;
    return MOTUS_FCL_UNAVAILABLE;
}

int motus_fcl_upsert_sphere(motus_fcl_world* world, uint32_t id, const motus_transform* pose, double radius)
{
    (void)world; (void)id; (void)pose; (void)radius;
    return MOTUS_FCL_UNAVAILABLE;
}

int motus_fcl_upsert_box(
    motus_fcl_world* world, uint32_t id, const motus_transform* pose,
    double half_x, double half_y, double half_z)
{
    (void)world; (void)id; (void)pose; (void)half_x; (void)half_y; (void)half_z;
    return MOTUS_FCL_UNAVAILABLE;
}

int motus_fcl_upsert_capsule(
    motus_fcl_world* world, uint32_t id, const motus_transform* pose,
    double radius, double half_length)
{
    (void)world; (void)id; (void)pose; (void)radius; (void)half_length;
    return MOTUS_FCL_UNAVAILABLE;
}

int motus_fcl_set_robot_link(
    motus_fcl_world* world, int link_index, const motus_transform* link_pose,
    const float* vertices, uint32_t vertex_count, const uint32_t* indices, uint32_t index_count)
{
    (void)world; (void)link_index; (void)link_pose;
    (void)vertices; (void)vertex_count; (void)indices; (void)index_count;
    return MOTUS_FCL_UNAVAILABLE;
}

int motus_fcl_set_allowed_pair(motus_fcl_world* world, uint32_t a, uint32_t b)
{
    (void)world; (void)a; (void)b;
    return MOTUS_FCL_UNAVAILABLE;
}

int motus_fcl_check(motus_fcl_world* world, int* out_a, int* out_b)
{
    (void)world;
    if (out_a) *out_a = -1;
    if (out_b) *out_b = -1;
    return MOTUS_FCL_UNAVAILABLE;
}
