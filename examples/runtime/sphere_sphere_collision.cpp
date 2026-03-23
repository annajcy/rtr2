#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include <pbpt/math/math.h>

#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/component/physics/rigid_body/sphere_collider.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/framework/integration/physics/rigid_body_scene_sync.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_system.hpp"

int main() {
    try {
        rtr::system::physics::rb::RigidBodySystem physics_world;
        rtr::framework::core::Scene        scene(1);

        auto& left = scene.create_game_object("left_sphere");
        left.node().set_local_position(pbpt::math::Vec3{-1.0f, 0.0f, 0.0f});
        auto& left_body = left.add_component<rtr::framework::component::RigidBody>();
        (void)left.add_component<rtr::framework::component::SphereCollider>(0.5f);
        left_body.set_use_gravity(false);

        auto& right = scene.create_game_object("right_sphere");
        right.node().set_local_position(pbpt::math::Vec3{1.0f, 0.0f, 0.0f});
        auto& right_body = right.add_component<rtr::framework::component::RigidBody>();
        (void)right.add_component<rtr::framework::component::SphereCollider>(0.5f);
        right_body.set_use_gravity(false);

        left_body.set_linear_velocity(pbpt::math::Vec3{1.0f, 0.0f, 0.0f});
        right_body.set_linear_velocity(pbpt::math::Vec3{-1.0f, 0.0f, 0.0f});

        constexpr double kFixedDt   = 0.1;
        constexpr int    kTickCount = 20;

        std::cout << "Sphere-sphere collision demo" << '\n';
        std::cout << "tick,left_x,left_vx,right_x,right_vx" << '\n';

        for (int tick = 0; tick < kTickCount; ++tick) {
            const rtr::framework::core::FixedTickContext fixed_ctx{
                .fixed_delta_seconds = kFixedDt,
                .fixed_tick_index    = static_cast<std::uint64_t>(tick),
            };
            rtr::framework::integration::physics::sync_scene_to_rigid_body(scene, physics_world);
            physics_world.step(static_cast<float>(fixed_ctx.fixed_delta_seconds));
            rtr::framework::integration::physics::sync_rigid_body_to_scene(scene, physics_world);

            std::cout << tick << ',' << std::fixed << std::setprecision(4) << left_body.position().x() << ','
                      << left_body.linear_velocity().x() << ',' << right_body.position().x() << ','
                      << right_body.linear_velocity().x() << '\n';
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
