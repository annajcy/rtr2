#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include <pbpt/math/math.h>

#include "rtr/framework/component/physics/rigid_body_component.hpp"
#include "rtr/framework/component/physics/sphere_collider_component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/system/physics/physics_system.hpp"

int main() {
    try {
        rtr::system::physics::PhysicsSystem physics_system;
        rtr::framework::core::Scene         scene(1);

        auto& left = scene.create_game_object("left_sphere");
        left.node().set_local_position(pbpt::math::Vec3{-1.0f, 0.0f, 0.0f});
        auto& left_body = left.add_component<rtr::framework::component::RigidBody>(physics_system.world());
        (void)left.add_component<rtr::framework::component::SphereCollider>(physics_system.world(), 0.5f);
        left_body.set_use_gravity(false);

        auto& right = scene.create_game_object("right_sphere");
        right.node().set_local_position(pbpt::math::Vec3{1.0f, 0.0f, 0.0f});
        auto& right_body = right.add_component<rtr::framework::component::RigidBody>(physics_system.world());
        (void)right.add_component<rtr::framework::component::SphereCollider>(physics_system.world(), 0.5f);
        right_body.set_use_gravity(false);

        physics_system.world().get_rigid_body(left_body.rigid_body_id()).state().translation.linear_velocity =
            pbpt::math::Vec3{1.0f, 0.0f, 0.0f};
        physics_system.world().get_rigid_body(right_body.rigid_body_id()).state().translation.linear_velocity =
            pbpt::math::Vec3{-1.0f, 0.0f, 0.0f};

        constexpr double kFixedDt   = 0.1;
        constexpr int    kTickCount = 20;

        std::cout << "Sphere-sphere collision demo" << '\n';
        std::cout << "tick,left_x,left_vx,right_x,right_vx" << '\n';

        for (int tick = 0; tick < kTickCount; ++tick) {
            physics_system.fixed_tick(scene, rtr::framework::core::FixedTickContext{
                                                 .fixed_delta_seconds = kFixedDt,
                                                 .fixed_tick_index    = static_cast<std::uint64_t>(tick),
                                             });

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
