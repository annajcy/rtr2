#include <pbpt/math/math.h>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/framework/component/camera_control/free_look_camera_controller.hpp"
#include "rtr/framework/component/camera_control/trackball_camera_controller.hpp"
#include "rtr/framework/component/component.hpp"
#include "rtr/framework/core/world.hpp"
#include "rtr/framework/integration/pbpt/pbpt_offline_render_service.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/rhi/context.hpp"
#include "rtr/rhi/device.hpp"
#include "rtr/rhi/window.hpp"
#include "rtr/system/input/input_state.hpp"
#include "rtr/system/input/input_types.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::utils::test {

namespace {

struct TempDir {
    std::filesystem::path path{};

    TempDir()
        : path(
              std::filesystem::temp_directory_path() /
              ("rtr_log_system_test_" +
               std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()))
          ) {
        std::filesystem::remove_all(path);
        std::filesystem::create_directories(path);
    }

    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
};

bool file_contains(
    const std::filesystem::path& file_path,
    const std::string& needle,
    int retries = 30
) {
    for (int i = 0; i < retries; ++i) {
        std::ifstream in(file_path, std::ios::binary);
        if (in.is_open()) {
            const std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
            if (content.find(needle) != std::string::npos) {
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

bool gpu_tests_enabled() {
    const char* value = std::getenv("RTR_RUN_GPU_TESTS");
    return value != nullptr && std::string(value) == "1";
}

rhi::ContextCreateInfo make_context_create_info(rhi::Window& window) {
    rhi::ContextCreateInfo info{};
    info.app_name = "LogSystemTest";
    info.instance_extensions = window.required_extensions();
    info.surface_creator = [&window](const vk::raii::Instance& instance) {
        return window.create_vk_surface(instance);
    };
    info.enable_validation_layers = false;
    return info;
}

resource::MeshHandle create_triangle_mesh(resource::ResourceManager& resources) {
    utils::ObjMeshData mesh{};

    utils::ObjVertex v0{};
    v0.position = pbpt::math::vec3(0.0f, 0.0f, 0.0f);
    v0.uv = pbpt::math::vec2(0.0f, 0.0f);
    v0.normal = pbpt::math::vec3(0.0f, 0.0f, 1.0f);

    utils::ObjVertex v1{};
    v1.position = pbpt::math::vec3(1.0f, 0.0f, 0.0f);
    v1.uv = pbpt::math::vec2(1.0f, 0.0f);
    v1.normal = pbpt::math::vec3(0.0f, 0.0f, 1.0f);

    utils::ObjVertex v2{};
    v2.position = pbpt::math::vec3(0.0f, 1.0f, 0.0f);
    v2.uv = pbpt::math::vec2(0.0f, 1.0f);
    v2.normal = pbpt::math::vec3(0.0f, 0.0f, 1.0f);

    mesh.vertices = {v0, v1, v2};
    mesh.indices = {0, 1, 2};

    return resources.create<rtr::resource::MeshResourceKind>(std::move(mesh));
}

class DummyFrameworkComponent final : public framework::component::Component {
public:
    explicit DummyFrameworkComponent(framework::core::GameObject& owner)
        : Component(owner) {}
};

} // namespace

TEST(LogSystemTest, SubscribeOnlyReceivesEventsAfterSubscription) {
    shutdown_logging();
    LogConfig config{};
    config.enable_console = false;
    config.enable_file = false;
    config.level = LogLevel::debug;
    init_logging(config);

    auto logger = get_logger("log.subscribe.test");
    const std::string before_token = "subscribe-before-token";
    const std::string after_token = "subscribe-after-token";
    logger->info("{}", before_token);

    std::mutex mutex{};
    std::vector<std::string> messages{};
    const auto handle = subscribe_logs([&](const LogEntry& entry) {
        std::scoped_lock lock(mutex);
        messages.push_back(entry.message);
    });

    logger->info("{}", after_token);
    ASSERT_TRUE(unsubscribe_logs(handle));

    bool found_before = false;
    bool found_after = false;
    {
        std::scoped_lock lock(mutex);
        for (const auto& message : messages) {
            if (message.find(before_token) != std::string::npos) {
                found_before = true;
            }
            if (message.find(after_token) != std::string::npos) {
                found_after = true;
            }
        }
    }

    EXPECT_FALSE(found_before);
    EXPECT_TRUE(found_after);
    shutdown_logging();
}

TEST(LogSystemTest, UnsubscribeStopsReceivingEvents) {
    shutdown_logging();
    LogConfig config{};
    config.enable_console = false;
    config.enable_file = false;
    config.level = LogLevel::debug;
    init_logging(config);

    auto logger = get_logger("log.unsubscribe.test");
    std::mutex mutex{};
    std::vector<std::string> messages{};

    const auto handle = subscribe_logs([&](const LogEntry& entry) {
        std::scoped_lock lock(mutex);
        messages.push_back(entry.message);
    });
    logger->info("unsubscribe-before");
    ASSERT_TRUE(unsubscribe_logs(handle));
    logger->info("unsubscribe-after");

    int before_count = 0;
    int after_count = 0;
    {
        std::scoped_lock lock(mutex);
        for (const auto& message : messages) {
            if (message.find("unsubscribe-before") != std::string::npos) {
                ++before_count;
            }
            if (message.find("unsubscribe-after") != std::string::npos) {
                ++after_count;
            }
        }
    }

    EXPECT_GE(before_count, 1);
    EXPECT_EQ(after_count, 0);
    shutdown_logging();
}

TEST(LogSystemTest, MultipleSubscribersReceiveRealtimeEvents) {
    shutdown_logging();
    LogConfig config{};
    config.enable_console = false;
    config.enable_file = false;
    config.level = LogLevel::debug;
    init_logging(config);

    auto logger = get_logger("log.multi_subscriber.test");
    int subscriber_a_count = 0;
    int subscriber_b_count = 0;

    const auto handle_a = subscribe_logs([&](const LogEntry& entry) {
        if (entry.message.find("multi-subscriber-token") != std::string::npos) {
            ++subscriber_a_count;
        }
    });
    const auto handle_b = subscribe_logs([&](const LogEntry& entry) {
        if (entry.message.find("multi-subscriber-token") != std::string::npos) {
            ++subscriber_b_count;
        }
    });

    logger->info("multi-subscriber-token");
    ASSERT_TRUE(unsubscribe_logs(handle_a));
    ASSERT_TRUE(unsubscribe_logs(handle_b));

    EXPECT_EQ(subscriber_a_count, 1);
    EXPECT_EQ(subscriber_b_count, 1);
    shutdown_logging();
}

TEST(LogSystemTest, InitIsIdempotentAndLoggerIsCachedAndLevelCanChange) {
    shutdown_logging();
    TempDir temp_dir{};
    const auto log_file = temp_dir.path / "rtr.log";

    LogConfig config{};
    config.enable_console = false;
    config.enable_file = true;
    config.file_path = log_file.string();
    config.level = LogLevel::debug;
    init_logging(config);
    init_logging(config);

    auto logger_a = get_logger("rhi.mesh");
    auto logger_b = get_logger("rhi.mesh");
    ASSERT_EQ(logger_a.get(), logger_b.get());
    EXPECT_TRUE(logger_a->should_log(spdlog::level::debug));

    set_level(LogLevel::warn);
    EXPECT_FALSE(logger_a->should_log(spdlog::level::info));

    logger_a->warn("log-system-test-warn");
    logger_a->flush();
    EXPECT_TRUE(file_contains(log_file, "log-system-test-warn"));

    shutdown_logging();
}

TEST(LogSystemTest, ModuleNamesAreWrittenToFile) {
    shutdown_logging();
    TempDir temp_dir{};
    const auto log_file = temp_dir.path / "rtr.log";

    LogConfig config{};
    config.enable_console = false;
    config.enable_file = true;
    config.file_path = log_file.string();
    config.level = LogLevel::debug;
    init_logging(config);

    auto resource_logger = get_logger("resource.manager");
    auto mesh_logger = get_logger("rhi.mesh");
    resource_logger->debug("resource-manager-semantic-log");
    mesh_logger->debug("rhi-mesh-upload-detail-log");
    resource_logger->flush();
    mesh_logger->flush();

    EXPECT_TRUE(file_contains(log_file, "[resource.manager]"));
    EXPECT_TRUE(file_contains(log_file, "[rhi.mesh]"));
    EXPECT_TRUE(file_contains(log_file, "resource-manager-semantic-log"));
    EXPECT_TRUE(file_contains(log_file, "rhi-mesh-upload-detail-log"));

    shutdown_logging();
}

TEST(LogSystemTest, FrameworkCoreLifecycleLogsAreWritten) {
    shutdown_logging();
    TempDir temp_dir{};
    const auto log_file = temp_dir.path / "rtr.log";

    LogConfig config{};
    config.enable_console = false;
    config.enable_file = true;
    config.file_path = log_file.string();
    config.level = LogLevel::debug;
    init_logging(config);

    resource::ResourceManager resources{};
    framework::core::World world{resources};
    auto& scene_a = world.create_scene("scene_a");
    auto& scene_b = world.create_scene("scene_b");

    auto& camera_go = scene_a.create_game_object("camera_go");
    (void)camera_go.add_component<DummyFrameworkComponent>();
    auto& camera = camera_go.add_component<framework::component::PerspectiveCamera>();
    camera.set_active(true);
    EXPECT_TRUE(scene_a.destroy_game_object(camera_go.id()));

    EXPECT_TRUE(world.set_active_scene(scene_b.id()));
    EXPECT_TRUE(world.destroy_scene(scene_b.id()));

    get_logger("framework.core.world")->flush();
    get_logger("framework.core.scene")->flush();
    get_logger("framework.core.game_object")->flush();

    EXPECT_TRUE(file_contains(log_file, "[framework.core.world]"));
    EXPECT_TRUE(file_contains(log_file, "[framework.core.scene]"));
    EXPECT_TRUE(file_contains(log_file, "[framework.core.game_object]"));

    shutdown_logging();
}

TEST(LogSystemTest, ControllerNodeChangeTraceLogsAppearAtTraceLevel) {
    shutdown_logging();
    TempDir temp_dir{};
    const auto log_file = temp_dir.path / "rtr.log";

    LogConfig config{};
    config.enable_console = false;
    config.enable_file = true;
    config.file_path = log_file.string();
    config.level = LogLevel::trace;
    init_logging(config);

    framework::core::Scene scene(1, "controller_trace_scene");

    system::input::InputState input{};
    auto& free_look_go = scene.create_game_object("free_look_camera");
    auto& free_look_camera = free_look_go.add_component<framework::component::PerspectiveCamera>();
    free_look_camera.set_active(true);
    (void)free_look_go.add_component<framework::component::FreeLookCameraController>(input);

    auto& trackball_go = scene.create_game_object("trackball_camera");
    auto& trackball_camera = trackball_go.add_component<framework::component::PerspectiveCamera>();
    trackball_camera.set_active(false);
    trackball_go.node().set_world_position({0.0f, 0.0f, -5.0f});
    (void)trackball_go.add_component<framework::component::TrackBallCameraController>(input);

    input.update_key(system::input::KeyCode::W, system::input::KeyAction::PRESS, system::input::KeyMod::NONE);
    scene.tick({.delta_seconds = 1.0, .unscaled_delta_seconds = 1.0, .frame_index = 0});
    input.update_key(system::input::KeyCode::W, system::input::KeyAction::RELEASE, system::input::KeyMod::NONE);

    free_look_camera.set_active(false);
    trackball_camera.set_active(true);
    input.reset_deltas();
    input.update_mouse_button(
        system::input::MouseButton::LEFT,
        system::input::KeyAction::PRESS,
        system::input::KeyMod::NONE
    );
    input.update_mouse_position(64.0, 24.0);
    scene.tick({.delta_seconds = 0.0, .unscaled_delta_seconds = 0.0, .frame_index = 1});

    get_logger("framework.component.free_look")->flush();
    get_logger("framework.component.trackball")->flush();

    EXPECT_TRUE(file_contains(log_file, "[framework.component.free_look]"));
    EXPECT_TRUE(file_contains(log_file, "[framework.component.trackball]"));
    EXPECT_TRUE(file_contains(log_file, "FreeLook node position updated"));
    EXPECT_TRUE(file_contains(log_file, "TrackBall node orbit updated"));

    shutdown_logging();
}

TEST(LogSystemTest, PbptServiceLifecycleLogsAreWritten) {
    shutdown_logging();
    TempDir temp_dir{};
    const auto log_file = temp_dir.path / "rtr.log";

    LogConfig config{};
    config.enable_console = false;
    config.enable_file = true;
    config.file_path = log_file.string();
    config.level = LogLevel::debug;
    init_logging(config);

    framework::core::Scene scene(1, "pbpt_service_log_scene");
    resource::ResourceManager resources{};
    framework::integration::PbptOfflineRenderService service{};

    EXPECT_FALSE(service.start(scene, resources, framework::integration::OfflineRenderConfig{
        .scene_xml_path = "",
        .output_exr_path = (temp_dir.path / "offline.exr").string(),
        .spp = 1
    }));

    get_logger("framework.integration.pbpt.offline_service")->flush();
    EXPECT_TRUE(file_contains(log_file, "[framework.integration.pbpt.offline_service]"));
    EXPECT_TRUE(file_contains(log_file, "Offline render start"));

    shutdown_logging();
}

TEST(LogSystemTest, ResourceManagerAndRhiMeshLogsAppearDuringFirstGpuUpload) {
    if (!gpu_tests_enabled()) {
        GTEST_SKIP() << "Set RTR_RUN_GPU_TESTS=1 to run GPU log integration scenario.";
    }

    shutdown_logging();
    TempDir temp_dir{};
    const auto log_file = temp_dir.path / "rtr.log";

    LogConfig config{};
    config.enable_console = false;
    config.enable_file = true;
    config.file_path = log_file.string();
    config.level = LogLevel::debug;
    init_logging(config);

    rhi::Window window(320, 240, "rtr_log_system_gpu_test");
    rhi::Context context(make_context_create_info(window));
    rhi::Device device(context);

    resource::ResourceManager resources("./assets/");
    const auto handle = create_triangle_mesh(resources);
    (void)resources.require_gpu<rtr::resource::MeshResourceKind>(handle, device);
    device.wait_idle();

    get_logger("resource.manager")->flush();
    get_logger("rhi.mesh")->flush();

    EXPECT_TRUE(file_contains(log_file, "[resource.manager]"));
    EXPECT_TRUE(file_contains(log_file, "[rhi.mesh]"));
    EXPECT_TRUE(file_contains(log_file, "triggering first GPU upload"));
    EXPECT_TRUE(file_contains(log_file, "Uploading mesh to GPU"));

    shutdown_logging();
}

} // namespace rtr::utils::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
