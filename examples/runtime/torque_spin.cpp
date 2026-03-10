#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include <pbpt/math/math.h>

#include "rtr/framework/component/physics/rigid_body.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/framework/integration/physics/scene_physics_sync.hpp"
#include "rtr/system/physics/physics_world.hpp"

int main() {
    try {
        rtr::system::physics::PhysicsWorld physics_world;
        rtr::framework::core::Scene        scene(1);

        auto& spinner = scene.create_game_object("torque_spinner");
        spinner.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});

        auto& rigid_body = spinner.add_component<rtr::framework::component::RigidBody>(physics_world);
        rigid_body.set_use_gravity(false);
        pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
        inverse_inertia_tensor_ref[1][1] = 1.0f;
        rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

        constexpr double kFixedDt   = 1.0 / 60.0;
        constexpr int    kTickCount = 120;

        std::cout << "Torque spin demo" << '\n';
        std::cout << "tick,angular_velocity_y,orientation_y,orientation_w" << '\n';

        for (int tick = 0; tick < kTickCount; ++tick) {
            rigid_body.add_torque(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});
            const rtr::framework::core::FixedTickContext fixed_ctx{
                .fixed_delta_seconds = kFixedDt,
                .fixed_tick_index    = static_cast<std::uint64_t>(tick),
            };
            rtr::framework::integration::physics::sync_scene_to_physics(scene, physics_world);
            physics_world.tick(static_cast<float>(fixed_ctx.fixed_delta_seconds));
            rtr::framework::integration::physics::sync_physics_to_scene(scene, physics_world);

            const auto orientation = rigid_body.orientation();
            const auto omega       = rigid_body.angular_velocity();
            std::cout << tick << ',' << std::fixed << std::setprecision(4) << omega.y() << ',' << orientation.y()
                      << ',' << orientation.w() << '\n';
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
