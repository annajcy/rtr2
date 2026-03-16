#include <gtest/gtest.h>

#include "../../../examples/editor/sine_wave_deformer.hpp"

#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/resource/resource_manager.hpp"

namespace {

rtr::utils::ObjMeshData make_wave_mesh() {
    rtr::utils::ObjMeshData mesh{};
    mesh.vertices = {
        {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}},
        {{1.0f, 0.0f, 0.5f}, {1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}},
        {{0.0f, 1.0f, 1.0f}, {0.0f, 1.0f}, {0.0f, 1.0f, 0.0f}},
    };
    mesh.indices = {0, 1, 2};
    return mesh;
}

} // namespace

TEST(SineWaveDeformerTest, DeformsOnlyDuringFixedTick) {
    rtr::resource::ResourceManager resources{};
    const auto handle = resources.create<rtr::resource::DeformableMeshResourceKind>(make_wave_mesh());

    rtr::framework::core::Scene scene(1);
    auto& go = scene.create_game_object("wave_mesh");
    auto& mesh = go.add_component<rtr::framework::component::DeformableMeshComponent>(resources, handle);
    auto& deformer = go.add_component<rtr::examples::SineWaveDeformer>();
    deformer.amplitude = 0.25f;
    deformer.spatial_frequency = 2.0f;
    deformer.time_speed = 3.0f;

    const auto original = mesh.local_vertices();

    scene.tick({.delta_seconds = 0.25, .unscaled_delta_seconds = 0.25, .frame_index = 0});
    const auto after_frame_tick = mesh.local_vertices();
    ASSERT_EQ(after_frame_tick.size(), original.size());
    for (std::size_t i = 0; i < original.size(); ++i) {
        EXPECT_FLOAT_EQ(after_frame_tick[i].x(), original[i].x());
        EXPECT_FLOAT_EQ(after_frame_tick[i].y(), original[i].y());
        EXPECT_FLOAT_EQ(after_frame_tick[i].z(), original[i].z());
    }

    scene.fixed_tick({.fixed_delta_seconds = 0.25, .fixed_tick_index = 0});
    const auto after_fixed_tick = mesh.local_vertices();

    bool any_changed = false;
    for (std::size_t i = 0; i < original.size(); ++i) {
        if (after_fixed_tick[i].y() != original[i].y()) {
            any_changed = true;
            break;
        }
    }
    EXPECT_TRUE(any_changed);

    scene.tick({.delta_seconds = 0.25, .unscaled_delta_seconds = 0.25, .frame_index = 1});
    const auto after_second_frame_tick = mesh.local_vertices();
    for (std::size_t i = 0; i < after_fixed_tick.size(); ++i) {
        EXPECT_FLOAT_EQ(after_second_frame_tick[i].x(), after_fixed_tick[i].x());
        EXPECT_FLOAT_EQ(after_second_frame_tick[i].y(), after_fixed_tick[i].y());
        EXPECT_FLOAT_EQ(after_second_frame_tick[i].z(), after_fixed_tick[i].z());
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
