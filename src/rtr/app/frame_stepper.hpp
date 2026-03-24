#pragma once

#include <cstdint>

#include "rtr/app/frame_time_policy.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/framework/core/world.hpp"
#include "rtr/framework/integration/physics/scene_physics_step.hpp"
#include "rtr/system/physics/physics_system.hpp"

namespace rtr::app {

struct FrameStepperContext {
    framework::core::World& world;
    system::physics::PhysicsSystem& physics_system;
    std::uint64_t& fixed_tick_serial;
    std::uint64_t frame_serial{0};
};

class FrameStepper {
public:
    void run_fixed_steps(const FrameExecutionPlan& plan, FrameStepperContext& ctx) const {
        for (std::uint32_t i = 0; i < plan.fixed_steps_to_run; ++i) {
            const std::uint64_t fixed_tick_serial = ctx.fixed_tick_serial++;
            const framework::core::FixedTickContext fixed_ctx{
                .fixed_delta_seconds = plan.fixed_dt,
                .fixed_tick_serial = fixed_tick_serial,
            };

            ctx.world.fixed_tick(fixed_ctx);
            if (auto* active_scene = ctx.world.active_scene(); active_scene != nullptr) {
                framework::integration::physics::step_scene_physics(
                    *active_scene,
                    ctx.physics_system,
                    static_cast<float>(fixed_ctx.fixed_delta_seconds)
                );
            }
        }
    }

    void run_variable_update(const FrameExecutionPlan& plan, FrameStepperContext& ctx) const {
        const framework::core::FrameTickContext tick_ctx{
            .delta_seconds = plan.frame_delta_seconds,
            .unscaled_delta_seconds = plan.frame_delta_seconds,
            .frame_serial = ctx.frame_serial,
        };
        ctx.world.tick(tick_ctx);
        ctx.world.late_tick(tick_ctx);
    }

    void execute(const FrameExecutionPlan& plan, FrameStepperContext& ctx) const {
        const bool skip_all = plan.fixed_steps_to_run == 0 &&
                              plan.fixed_dt == 0.0 &&
                              plan.frame_delta_seconds == 0.0;
        if (skip_all) {
            return;
        }

        run_fixed_steps(plan, ctx);
        run_variable_update(plan, ctx);
    }
};

} // namespace rtr::app
