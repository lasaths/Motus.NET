#include "motus_fcl.h"

#ifdef MOTUS_HAS_FCL

#include <fcl/fcl.h>
#include <unordered_map>
#include <vector>

struct motus_fcl_world {
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> manager;
    std::unordered_map<uint32_t, std::shared_ptr<fcl::CollisionObjectd>> objects;
    std::vector<std::pair<uint32_t, uint32_t>> allowed_pairs;
};

static fcl::Transform3d TransformFromMotus(const motus_transform* t)
{
    fcl::Transform3d tf = fcl::Transform3d::Identity();
    if (!t) return tf;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            tf(r, c) = t->m[r * 4 + c];
    return tf;
}

extern void motus_set_last_error(const char*);

extern "C" {

int motus_fcl_is_available(void) { return 1; }

motus_fcl_world* motus_fcl_world_create(void)
{
    auto* w = new motus_fcl_world();
    w->manager = std::make_shared<fcl::BroadPhaseCollisionManagerd>();
    return w;
}

void motus_fcl_world_destroy(motus_fcl_world* world)
{
    delete world;
}

static bool PairAllowed(const motus_fcl_world* world, uint32_t a, uint32_t b)
{
    for (const auto& p : world->allowed_pairs)
        if ((p.first == a && p.second == b) || (p.first == b && p.second == a))
            return true;
    return false;
}

static void UnregisterId(motus_fcl_world* world, uint32_t id)
{
    auto it = world->objects.find(id);
    if (it == world->objects.end()) return;
    world->manager->unregisterObject(it->second.get());
    world->objects.erase(it);
}

static int UpsertObject(motus_fcl_world* world, uint32_t id,
    const std::shared_ptr<fcl::CollisionGeometryd>& geom, const motus_transform* pose)
{
    if (!world) return MOTUS_FCL_ERR;
    UnregisterId(world, id);
    auto obj = std::make_shared<fcl::CollisionObjectd>(geom, TransformFromMotus(pose));
    world->objects[id] = obj;
    world->manager->registerObject(obj.get());
    world->manager->setup();
    return MOTUS_FCL_OK;
}

static int AddBox(motus_fcl_world* world, uint32_t id, const motus_transform* pose,
    double half_x, double half_y, double half_z)
{
    auto box = std::make_shared<fcl::Boxd>(half_x * 2, half_y * 2, half_z * 2);
    return UpsertObject(world, id, box, pose);
}

int motus_fcl_add_obstacle_box(
    motus_fcl_world* world, uint32_t id, const motus_transform* pose,
    double half_x, double half_y, double half_z)
{
    if (!world) return MOTUS_FCL_ERR;
    return AddBox(world, id, pose, half_x, half_y, half_z);
}

int motus_fcl_attach_box(
    motus_fcl_world* world, uint32_t id, const motus_transform* world_pose,
    double half_x, double half_y, double half_z)
{
    if (!world) return MOTUS_FCL_ERR;
    return AddBox(world, id, world_pose, half_x, half_y, half_z);
}

int motus_fcl_detach(motus_fcl_world* world, uint32_t id)
{
    return motus_fcl_remove(world, id);
}

int motus_fcl_remove(motus_fcl_world* world, uint32_t id)
{
    if (!world) return MOTUS_FCL_ERR;
    auto it = world->objects.find(id);
    if (it == world->objects.end()) return MOTUS_FCL_ERR;
    UnregisterId(world, id);
    world->manager->setup();
    return MOTUS_FCL_OK;
}

int motus_fcl_upsert_sphere(
    motus_fcl_world* world, uint32_t id, const motus_transform* pose, double radius)
{
    if (!world || radius <= 0) return MOTUS_FCL_ERR;
    auto sphere = std::make_shared<fcl::Sphered>(radius);
    return UpsertObject(world, id, sphere, pose);
}

int motus_fcl_upsert_box(
    motus_fcl_world* world, uint32_t id, const motus_transform* pose,
    double half_x, double half_y, double half_z)
{
    return AddBox(world, id, pose, half_x, half_y, half_z);
}

int motus_fcl_upsert_capsule(
    motus_fcl_world* world, uint32_t id, const motus_transform* pose,
    double radius, double half_length)
{
    if (!world || radius <= 0 || half_length <= 0) return MOTUS_FCL_ERR;
    auto cap = std::make_shared<fcl::Capsuled>(radius, half_length * 2);
    return UpsertObject(world, id, cap, pose);
}

int motus_fcl_set_robot_link(
    motus_fcl_world* world, int link_index, const motus_transform* link_pose,
    const float* vertices, uint32_t vertex_count, const uint32_t* indices, uint32_t index_count)
{
    (void)world; (void)link_index; (void)link_pose; (void)vertices; (void)vertex_count; (void)indices; (void)index_count;
    motus_set_last_error("mesh robot links not implemented in FCL binding yet");
    return MOTUS_FCL_ERR;
}

int motus_fcl_set_allowed_pair(motus_fcl_world* world, uint32_t a, uint32_t b)
{
    if (!world) return MOTUS_FCL_ERR;
    world->allowed_pairs.emplace_back(a, b);
    return MOTUS_FCL_OK;
}

int motus_fcl_check(motus_fcl_world* world, int* out_a, int* out_b)
{
    if (!world) return MOTUS_FCL_ERR;
    struct CbData {
        bool hit = false;
        uint32_t a = 0, b = 0;
        const motus_fcl_world* w;
    } data{ false, 0, 0, world };

    world->manager->collide([&](fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2) {
        uint32_t id1 = 0, id2 = 0;
        for (const auto& kv : world->objects)
        {
            if (kv.second.get() == o1) id1 = kv.first;
            if (kv.second.get() == o2) id2 = kv.first;
        }
        if (PairAllowed(world, id1, id2)) return false;
        fcl::CollisionRequestd req;
        fcl::CollisionResultd res;
        fcl::collide(o1, o2, req, res);
        if (res.isCollision())
        {
            data.hit = true;
            data.a = id1;
            data.b = id2;
            return true;
        }
        return false;
    });

    if (data.hit)
    {
        if (out_a) *out_a = static_cast<int>(data.a);
        if (out_b) *out_b = static_cast<int>(data.b);
        return MOTUS_FCL_ERR;
    }
    return MOTUS_FCL_OK;
}

} // extern "C"

#endif
