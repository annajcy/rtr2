#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <future>
#include <stdexcept>
#include <string>
#include <thread>

#include "rtr/framework/core/camera.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/component/mesh_renderer.hpp"
#include "rtr/framework/component/pbpt_light.hpp"
#include "rtr/framework/component/pbpt_mesh.hpp"
#include "rtr/framework/integration/pbpt_offline_render_service.hpp"

namespace rtr::framework::integration::testing {

using namespace std::chrono_literals;

namespace {

struct TempDir {
    std::filesystem::path path{};

    explicit TempDir(const std::string& prefix) {
        const auto stamp = std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()
        );
        path = std::filesystem::temp_directory_path() / (prefix + "_" + stamp);
        std::filesystem::create_directories(path);
    }

    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
};

void setup_scene_with_camera(core::Scene& scene) {
    auto& camera_go = scene.create_game_object("camera");
    auto& camera = scene.camera_manager().create_perspective_camera(camera_go.id());
    camera.set_aspect_ratio(1.0f);
    (void)scene.set_active_camera(camera_go.id());
    scene.scene_graph().update_world_transforms();
}

void add_minimal_pbpt_emitter_shape(core::Scene& scene) {
    auto& go = scene.create_game_object("pbpt_test_shape");
    (void)go.add_component<component::MeshRenderer>(
        "assets/models/colored_quad.obj",
        "assets/textures/default_checkerboard_512.png"
    );
    (void)go.add_component<component::PbptMesh>();
    (void)go.add_component<component::PbptLight>();
    scene.scene_graph().update_world_transforms();
}

bool wait_for_terminal_state(
    const PbptOfflineRenderService& service,
    std::chrono::milliseconds timeout = 3000ms
) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (!service.is_running()) {
            return true;
        }
        std::this_thread::sleep_for(5ms);
    }
    return !service.is_running();
}

} // namespace

TEST(PbptOfflineRenderServiceTest, StartTransitionsToRunningAndThenSucceeded) {
    core::Scene scene(1, "offline_service_scene");
    setup_scene_with_camera(scene);
    add_minimal_pbpt_emitter_shape(scene);

    TempDir temp_dir("rtr_pbpt_offline_success");
    const auto scene_xml = (temp_dir.path / "runtime_scene.xml").string();
    const auto output_exr = (temp_dir.path / "runtime_output.exr").string();

    std::promise<void> backend_entered;
    auto backend_entered_future = backend_entered.get_future();
    std::atomic<bool> allow_finish{false};

    PbptOfflineRenderService service(
        [&](const OfflineRenderConfig&,
            const PbptOfflineRenderService::ProgressCallback& on_progress,
            const PbptOfflineRenderService::CancelQuery&) {
            if (on_progress) {
                on_progress(0.25f);
            }
            backend_entered.set_value();
            while (!allow_finish.load()) {
                std::this_thread::sleep_for(5ms);
            }
            if (on_progress) {
                on_progress(1.0f);
            }
        }
    );

    ASSERT_TRUE(service.start(scene, OfflineRenderConfig{
        .scene_xml_path = scene_xml,
        .output_exr_path = output_exr,
        .spp = 8
    }));

    ASSERT_EQ(backend_entered_future.wait_for(500ms), std::future_status::ready);
    EXPECT_EQ(service.state(), OfflineRenderState::Running);
    EXPECT_GE(service.progress_01(), 0.25f);

    allow_finish.store(true);
    ASSERT_TRUE(wait_for_terminal_state(service));

    EXPECT_EQ(service.state(), OfflineRenderState::Succeeded);
    EXPECT_FLOAT_EQ(service.progress_01(), 1.0f);
    EXPECT_TRUE(std::filesystem::exists(scene_xml));
}

TEST(PbptOfflineRenderServiceTest, RequestCancelTransitionsToCanceled) {
    core::Scene scene(1, "offline_service_scene");
    setup_scene_with_camera(scene);
    add_minimal_pbpt_emitter_shape(scene);

    TempDir temp_dir("rtr_pbpt_offline_cancel");
    const auto scene_xml = (temp_dir.path / "runtime_scene.xml").string();
    const auto output_exr = (temp_dir.path / "runtime_output.exr").string();

    std::promise<void> backend_entered;
    auto backend_entered_future = backend_entered.get_future();

    PbptOfflineRenderService service(
        [&](const OfflineRenderConfig&,
            const PbptOfflineRenderService::ProgressCallback& on_progress,
            const PbptOfflineRenderService::CancelQuery& is_cancel_requested) {
            backend_entered.set_value();
            while (true) {
                if (on_progress) {
                    on_progress(0.5f);
                }
                if (is_cancel_requested && is_cancel_requested()) {
                    throw PbptOfflineRenderService::RenderCanceled("cancel");
                }
                std::this_thread::sleep_for(5ms);
            }
        }
    );

    ASSERT_TRUE(service.start(scene, OfflineRenderConfig{
        .scene_xml_path = scene_xml,
        .output_exr_path = output_exr,
        .spp = 4
    }));
    ASSERT_EQ(backend_entered_future.wait_for(500ms), std::future_status::ready);

    service.request_cancel();
    ASSERT_TRUE(wait_for_terminal_state(service));

    EXPECT_EQ(service.state(), OfflineRenderState::Canceled);
}

TEST(PbptOfflineRenderServiceTest, BackendFailureTransitionsToFailed) {
    core::Scene scene(1, "offline_service_scene");
    setup_scene_with_camera(scene);
    add_minimal_pbpt_emitter_shape(scene);

    TempDir temp_dir("rtr_pbpt_offline_failed");
    const auto scene_xml = (temp_dir.path / "runtime_scene.xml").string();
    const auto output_exr = (temp_dir.path / "blocked" / "runtime_output.exr").string();

    PbptOfflineRenderService service(
        [&](const OfflineRenderConfig& config,
            const PbptOfflineRenderService::ProgressCallback&,
            const PbptOfflineRenderService::CancelQuery&) {
            throw std::runtime_error("Failed to write output image: " + config.output_exr_path);
        }
    );

    ASSERT_TRUE(service.start(scene, OfflineRenderConfig{
        .scene_xml_path = scene_xml,
        .output_exr_path = output_exr,
        .spp = 2
    }));
    ASSERT_TRUE(wait_for_terminal_state(service));

    EXPECT_EQ(service.state(), OfflineRenderState::Failed);
    EXPECT_NE(service.last_message().find("Failed to write output image"), std::string::npos);
}

} // namespace rtr::framework::integration::testing
