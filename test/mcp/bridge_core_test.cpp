#include <filesystem>
#include <string>

#include "gtest/gtest.h"

#include "rtr/mcp/bridge_core.hpp"

namespace rtr::mcp::test {

namespace {

using json = nlohmann::json;

struct TempDir {
    std::filesystem::path path{};

    explicit TempDir(const std::string& name)
        : path(std::filesystem::temp_directory_path() / name) {
        std::filesystem::remove_all(path);
        std::filesystem::create_directories(path);
    }

    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
};

json invoke(BridgeCore& core, const std::string& method, json params = json::object(), json id = 1) {
    return core.handle_request(json{
        {"id", id},
        {"method", method},
        {"params", std::move(params)},
    });
}

std::string project_assets_dir() {
    return (std::filesystem::path(RTR_PROJECT_SOURCE_DIR) / "assets").string();
}

json make_scene_spec(bool include_emitter = true, bool active_camera = true) {
    json nodes = json::array({
        json{
            {"type", "perspective_camera"},
            {"name", "camera"},
            {"active", active_camera},
            {"fov_degrees", 50.0},
            {"near", 0.1},
            {"far", 200.0},
            {"transform", json{
                {"position", json{{"x", 0.0}, {"y", 1.0}, {"z", 6.0}}},
                {"rotation_quat", json{{"w", 1.0}, {"x", 0.0}, {"y", 0.0}, {"z", 0.0}}},
                {"scale", json{{"x", 1.0}, {"y", 1.0}, {"z", 1.0}}},
            }},
        },
        json{
            {"type", "mesh_object"},
            {"name", "bunny"},
            {"mesh_path", "models/stanford_bunny.obj"},
            {"base_color", json{{"x", 0.9}, {"y", 0.8}, {"z", 0.7}, {"w", 1.0}}},
            {"pbpt_mesh", true},
        },
    });

    if (include_emitter) {
        nodes.push_back(json{
            {"type", "emissive_mesh_object"},
            {"name", "light_quad"},
            {"mesh_path", "models/colored_quad.obj"},
            {"base_color", json{{"x", 1.0}, {"y", 1.0}, {"z", 1.0}, {"w", 1.0}}},
            {"radiance_spectrum", json::array({
                json{{"lambda_nm", 400.0}, {"value", 4.0}},
                json{{"lambda_nm", 500.0}, {"value", 4.0}},
                json{{"lambda_nm", 600.0}, {"value", 4.0}},
                json{{"lambda_nm", 700.0}, {"value", 4.0}},
            })},
        });
    }

    return json{
        {"scene_name", "mcp_scene"},
        {"nodes", std::move(nodes)},
    };
}

std::string require_session_id(const json& response) {
    return response.at("data").at("session_id").get<std::string>();
}

} // namespace

TEST(BridgeCoreTest, SessionCreateInspectAndResetReturnStableShape) {
    BridgeCore core;

    const auto created = invoke(core, "session_create", json{
        {"resource_root_dir", project_assets_dir()},
    });
    ASSERT_TRUE(created.at("ok").get<bool>());

    const std::string session_id = require_session_id(created);
    const auto inspected = invoke(core, "scene_inspect", json{{"session_id", session_id}});
    ASSERT_TRUE(inspected.at("ok").get<bool>());
    EXPECT_EQ(inspected.at("data").at("scene_name").get<std::string>(), "main_scene");
    EXPECT_EQ(inspected.at("data").at("game_object_count").get<std::size_t>(), 0u);

    const auto reset = invoke(core, "session_reset", json{
        {"session_id", session_id},
        {"scene_name", "reset_scene"},
    });
    ASSERT_TRUE(reset.at("ok").get<bool>());
    EXPECT_EQ(reset.at("data").at("scene").at("scene_name").get<std::string>(), "reset_scene");
}

TEST(BridgeCoreTest, SceneReplaceBuildsCameraMeshAndEmitter) {
    BridgeCore core;
    const auto created = invoke(core, "session_create", json{
        {"resource_root_dir", project_assets_dir()},
    });
    const std::string session_id = require_session_id(created);

    const auto replaced = invoke(core, "scene_replace", json{
        {"session_id", session_id},
        {"scene_spec", make_scene_spec()},
    });
    ASSERT_TRUE(replaced.at("ok").get<bool>()) << replaced.dump(2);

    const auto& data = replaced.at("data");
    EXPECT_EQ(data.at("scene_name").get<std::string>(), "mcp_scene");
    EXPECT_EQ(data.at("game_object_count").get<std::size_t>(), 3u);
    EXPECT_EQ(data.at("export_checks").at("active_camera_count").get<std::size_t>(), 1u);
    EXPECT_EQ(data.at("export_checks").at("exportable_shape_count").get<std::size_t>(), 2u);
    EXPECT_EQ(data.at("export_checks").at("emissive_shape_count").get<std::size_t>(), 1u);
    EXPECT_TRUE(data.at("export_checks").at("can_export_pbpt").get<bool>());
    EXPECT_TRUE(data.at("export_checks").at("can_start_offline_render").get<bool>());
}

TEST(BridgeCoreTest, SceneExportReturnsStructuredFailureForMissingOfflineRenderPrerequisites) {
    BridgeCore core;
    const auto created = invoke(core, "session_create", json{
        {"resource_root_dir", project_assets_dir()},
    });
    const std::string session_id = require_session_id(created);

    const auto replaced = invoke(core, "scene_replace", json{
        {"session_id", session_id},
        {"scene_spec", make_scene_spec(false, false)},
    });
    ASSERT_TRUE(replaced.at("ok").get<bool>());

    TempDir temp_dir("rtr_mcp_bridge_export_failure");
    const auto scene_xml = (temp_dir.path / "scene.xml").string();

    const auto export_result = invoke(core, "scene_export_pbpt", json{
        {"session_id", session_id},
        {"scene_xml_path", scene_xml},
        {"spp", 4},
    });
    ASSERT_FALSE(export_result.at("ok").get<bool>());
    EXPECT_EQ(export_result.at("error_code").get<std::string>(), "export_failed");
    EXPECT_NE(export_result.at("message").get<std::string>().find("active camera"), std::string::npos);
}

TEST(BridgeCoreTest, ImportThenExportPbptRoundTripsSupportedScene) {
    BridgeCore core;
    const auto created = invoke(core, "session_create", json{
        {"resource_root_dir", project_assets_dir()},
    });
    const std::string session_id = require_session_id(created);

    const auto import_result = invoke(core, "scene_import_pbpt", json{
        {"session_id", session_id},
        {"scene_xml_path", (std::filesystem::path(RTR_PROJECT_SOURCE_DIR) / "assets" / "pbpt_scene" / "cbox" / "cbox.xml").string()},
        {"mode", "replace"},
    });
    ASSERT_TRUE(import_result.at("ok").get<bool>()) << import_result.dump(2);

    const auto inspect_result = invoke(core, "scene_inspect", json{{"session_id", session_id}});
    ASSERT_TRUE(inspect_result.at("ok").get<bool>());
    EXPECT_EQ(inspect_result.at("data").at("export_checks").at("active_camera_count").get<std::size_t>(), 1u);
    EXPECT_GT(inspect_result.at("data").at("export_checks").at("exportable_shape_count").get<std::size_t>(), 0u);

    TempDir temp_dir("rtr_mcp_bridge_import_export");
    const auto export_path = (temp_dir.path / "roundtrip.xml").string();
    const auto export_result = invoke(core, "scene_export_pbpt", json{
        {"session_id", session_id},
        {"scene_xml_path", export_path},
        {"spp", 2},
    });
    ASSERT_TRUE(export_result.at("ok").get<bool>()) << export_result.dump(2);
    EXPECT_TRUE(std::filesystem::exists(export_path));
}

} // namespace rtr::mcp::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
