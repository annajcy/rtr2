#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include <pbpt/math/math.h>

#include "rtr/framework/component/physics/rigid_body/reset_position.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/framework/integration/physics/rigid_body_scene_sync.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_system.hpp"

int main() {
    try {
        rtr::system::physics::rb::RigidBodySystem physics_world;
        rtr::framework::core::Scene        scene(1);

        auto& mover = scene.create_game_object("gravity_reset_mover");
        mover.node().set_local_position(pbpt::math::Vec3{0.0f, 2.0f, 0.0f});

        auto& rigid_body = mover.add_component<rtr::framework::component::RigidBody>();
        auto& reset = mover.add_component<rtr::framework::component::ResetPosition>();
        reset.set_threshold_y(-1.0f);
        reset.set_reset_position(pbpt::math::Vec3{0.0f, 2.0f, 0.0f});

        constexpr double kFixedDt   = 1.0 / 30.0;
        constexpr int    kTickCount = 180;

        std::cout << "Gravity reset demo" << '\n';
        std::cout << "tick,position_y,velocity_y" << '\n';

        for (int tick = 0; tick < kTickCount; ++tick) {
            const rtr::framework::core::FixedTickContext fixed_ctx{
                .fixed_delta_seconds = kFixedDt,
                .fixed_tick_serial    = static_cast<std::uint64_t>(tick),
            };
            scene.fixed_tick(fixed_ctx);
            rtr::framework::integration::physics::sync_scene_to_rigid_body(scene, physics_world);
            physics_world.step(static_cast<float>(fixed_ctx.fixed_delta_seconds));
            rtr::framework::integration::physics::sync_rigid_body_to_scene(scene, physics_world);

            std::cout << tick << ',' << std::fixed << std::setprecision(4) << rigid_body.position().y() << ','
                      << rigid_body.linear_velocity().y() << '\n';
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
