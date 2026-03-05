#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include "rtr/framework/component/physics/rigid_body_component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/system/physics/physics_system.hpp"

int main() {
    try {
        rtr::system::physics::PhysicsSystem physics_system;
        rtr::framework::core::Scene scene(1);

        auto& mover = scene.create_game_object("ping_pong_mover");
        mover.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
        auto& rigid_body = mover.add_component<rtr::framework::component::RigidBodyComponent>(
            physics_system.world(), pbpt::math::Vec3{1.2f, 0.0f, 0.0f}
        );

        constexpr double kFixedDt = 1.0 / 30.0;
        constexpr int kTickCount = 180;
        constexpr float kMinX = -1.0f;
        constexpr float kMaxX = 1.0f;

        std::cout << "Physics ping-pong demo" << '\n';
        std::cout << "tick,position_x,velocity_x" << '\n';

        for (int tick = 0; tick < kTickCount; ++tick) {
            physics_system.fixed_tick(scene, rtr::framework::core::FixedTickContext{
                                                 .fixed_delta_seconds = kFixedDt,
                                                 .fixed_tick_index = static_cast<std::uint64_t>(tick),
                                             });

            auto& state = physics_system.world().get_rigid_body(rigid_body.rigid_body_id()).state();
            if (state.position.x() > kMaxX) {
                state.position.x() = kMaxX;
                state.linear_velocity.x() = -std::abs(state.linear_velocity.x());
            } else if (state.position.x() < kMinX) {
                state.position.x() = kMinX;
                state.linear_velocity.x() = std::abs(state.linear_velocity.x());
            }

            scene.scene_graph().node(mover.id()).set_local_position(state.position);
            scene.scene_graph().update_world_transforms();

            std::cout << tick << ','
                      << std::fixed << std::setprecision(4)
                      << state.position.x() << ','
                      << state.linear_velocity.x() << '\n';
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
