#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <pbpt/math/math.h>
#include <nlohmann/json.hpp>

#include "rtr/framework/component/camera/perspective_camera.hpp"
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/pbpt/pbpt_light.hpp"
#include "rtr/framework/component/pbpt/pbpt_mesh.hpp"
#include "rtr/framework/core/world.hpp"
#include "rtr/framework/integration/pbpt/pbpt_offline_render_service.hpp"
#include "rtr/framework/integration/pbpt/serde/model/compatible_info.hpp"
#include "rtr/framework/integration/pbpt/serde/scene_loader.hpp"
#include "rtr/framework/integration/pbpt/serde/scene_writer.hpp"
#include "rtr/resource/resource_manager.hpp"

#ifndef RTR_MCP_BRIDGE_VERSION
#define RTR_MCP_BRIDGE_VERSION "0.1.0"
#endif

#ifndef RTR_PROJECT_SOURCE_DIR
#define RTR_PROJECT_SOURCE_DIR "."
#endif

namespace rtr::mcp {

using json = nlohmann::json;

class BridgeError final : public std::runtime_error {
public:
    explicit BridgeError(std::string code, std::string message)
        : std::runtime_error(std::move(message)),
          m_code(std::move(code)) {}

    const std::string& code() const { return m_code; }

private:
    std::string m_code;
};

struct CommandResult {
    json data{json::object()};
    std::string message{};
};

inline json vec3_to_json(const pbpt::math::Vec3& v) {
    return json{{"x", v.x()}, {"y", v.y()}, {"z", v.z()}};
}

inline json quat_to_json(const pbpt::math::Quat& q) {
    return json{{"w", q.w()}, {"x", q.x()}, {"y", q.y()}, {"z", q.z()}};
}

inline json vec4_to_json(const pbpt::math::Vec4& v) {
    return json{{"x", v.x()}, {"y", v.y()}, {"z", v.z()}, {"w", v.w()}};
}

inline pbpt::math::Vec3 parse_vec3(const json& value, const pbpt::math::Vec3& fallback) {
    if (value.is_null()) {
        return fallback;
    }
    if (value.is_array() && value.size() == 3) {
        return pbpt::math::Vec3{
            value.at(0).get<float>(),
            value.at(1).get<float>(),
            value.at(2).get<float>(),
        };
    }
    if (value.is_object()) {
        return pbpt::math::Vec3{
            value.value("x", fallback.x()),
            value.value("y", fallback.y()),
            value.value("z", fallback.z()),
        };
    }
    throw BridgeError("validation_error", "Expected vec3 as array[3] or object{x,y,z}.");
}

inline pbpt::math::Quat parse_quat(const json& value, const pbpt::math::Quat& fallback) {
    if (value.is_null()) {
        return fallback;
    }
    if (value.is_array() && value.size() == 4) {
        return pbpt::math::Quat{
            value.at(0).get<float>(),
            value.at(1).get<float>(),
            value.at(2).get<float>(),
            value.at(3).get<float>(),
        };
    }
    if (value.is_object()) {
        return pbpt::math::Quat{
            value.value("w", fallback.w()),
            value.value("x", fallback.x()),
            value.value("y", fallback.y()),
            value.value("z", fallback.z()),
        };
    }
    throw BridgeError("validation_error", "Expected rotation_quat as array[4] or object{w,x,y,z}.");
}

inline pbpt::math::Vec4 parse_vec4(const json& value, const pbpt::math::Vec4& fallback) {
    if (value.is_null()) {
        return fallback;
    }
    if (value.is_array() && value.size() == 4) {
        return pbpt::math::Vec4{
            value.at(0).get<float>(),
            value.at(1).get<float>(),
            value.at(2).get<float>(),
            value.at(3).get<float>(),
        };
    }
    if (value.is_object()) {
        return pbpt::math::Vec4{
            value.value("x", fallback.x()),
            value.value("y", fallback.y()),
            value.value("z", fallback.z()),
            value.value("w", fallback.w()),
        };
    }
    throw BridgeError("validation_error", "Expected vec4 as array[4] or object{x,y,z,w}.");
}

inline std::string require_string_field(const json& object, std::string_view field_name) {
    const std::string key(field_name);
    if (!object.contains(key) || !object.at(key).is_string()) {
        throw BridgeError("validation_error", "Missing required string field '" + key + "'.");
    }
    return object.at(key).get<std::string>();
}

inline const json& require_object_field(const json& object, std::string_view field_name) {
    const std::string key(field_name);
    if (!object.contains(key) || !object.at(key).is_object()) {
        throw BridgeError("validation_error", "Missing required object field '" + key + "'.");
    }
    return object.at(key);
}

inline const json& require_array_field(const json& object, std::string_view field_name) {
    const std::string key(field_name);
    if (!object.contains(key) || !object.at(key).is_array()) {
        throw BridgeError("validation_error", "Missing required array field '" + key + "'.");
    }
    return object.at(key);
}

struct SessionState {
    explicit SessionState(std::filesystem::path root_dir, std::string initial_scene_name = "main_scene")
        : resource_root_dir(std::move(root_dir)),
          scene_name(std::move(initial_scene_name)) {
        reset(scene_name);
    }

    std::filesystem::path resource_root_dir{};
    std::string scene_name{"main_scene"};
    std::unique_ptr<resource::ResourceManager> resources{};
    std::unique_ptr<framework::core::World> world{};
    framework::core::Scene* active_scene{nullptr};
    std::optional<framework::integration::CompatibleInfo> compatible_info{};
    std::unique_ptr<framework::integration::PbptOfflineRenderService> offline_render_service{};
    std::optional<std::string> active_job_id{};
    std::uint64_t next_job_number{1};
    std::string import_note{};

    void reset(std::string new_scene_name = "main_scene") {
        if (offline_render_service) {
            offline_render_service->request_cancel();
        }

        scene_name = std::move(new_scene_name);
        compatible_info.reset();
        active_job_id.reset();
        next_job_number = 1;
        import_note.clear();

        resources = std::make_unique<resource::ResourceManager>(resource_root_dir);
        world = std::make_unique<framework::core::World>(*resources);
        active_scene = &world->create_scene(scene_name);
        (void)world->set_active_scene(active_scene->id());
        offline_render_service = std::make_unique<framework::integration::PbptOfflineRenderService>();
    }

    framework::core::Scene& scene() {
        if (active_scene == nullptr) {
            throw BridgeError("internal_error", "Session has no active scene.");
        }
        return *active_scene;
    }
};

class BridgeCore {
public:
    json handle_request(const json& request) {
        const json id = request.contains("id") ? request.at("id") : json(nullptr);

        try {
            if (!request.is_object()) {
                throw BridgeError("invalid_request", "Request must be a JSON object.");
            }

            const std::string method = require_string_field(request, "method");
            const json params = request.contains("params") ? request.at("params") : json::object();
            if (!params.is_object()) {
                throw BridgeError("validation_error", "Request params must be a JSON object.");
            }

            const CommandResult result = dispatch(method, params);
            return json{
                {"id", id},
                {"ok", true},
                {"error_code", ""},
                {"message", result.message},
                {"data", result.data},
            };
        } catch (const BridgeError& e) {
            return json{
                {"id", id},
                {"ok", false},
                {"error_code", e.code()},
                {"message", e.what()},
                {"data", json::object()},
            };
        } catch (const std::exception& e) {
            return json{
                {"id", id},
                {"ok", false},
                {"error_code", "internal_error"},
                {"message", e.what()},
                {"data", json::object()},
            };
        }
    }

private:
    std::unordered_map<std::string, std::unique_ptr<SessionState>> m_sessions{};
    std::uint64_t m_next_session_number{1};

    CommandResult dispatch(const std::string& method, const json& params) {
        if (method == "project_info") {
            return handle_project_info();
        }
        if (method == "session_create") {
            return handle_session_create(params);
        }
        if (method == "session_reset") {
            return handle_session_reset(params);
        }
        if (method == "scene_inspect") {
            return handle_scene_inspect(params);
        }
        if (method == "scene_replace") {
            return handle_scene_replace(params);
        }
        if (method == "scene_import_pbpt") {
            return handle_scene_import_pbpt(params);
        }
        if (method == "scene_export_pbpt") {
            return handle_scene_export_pbpt(params);
        }
        if (method == "offline_render") {
            return handle_offline_render(params);
        }
        throw BridgeError("unknown_method", "Unsupported method '" + method + "'.");
    }

    SessionState& require_session(const json& params) {
        const std::string session_id = require_string_field(params, "session_id");
        const auto it = m_sessions.find(session_id);
        if (it == m_sessions.end() || !it->second) {
            throw BridgeError("session_not_found", "Unknown session_id '" + session_id + "'.");
        }
        return *it->second;
    }

    CommandResult handle_project_info() const {
        const auto default_root = std::filesystem::path(std::string(resource::ResourceManager::kDefaultResourceRootDir));
        return CommandResult{
            .data = json{
                {"helper_version", RTR_MCP_BRIDGE_VERSION},
                {"source_root", std::string(RTR_PROJECT_SOURCE_DIR)},
                {"session_count", m_sessions.size()},
                {"build", json{
                    {"mode",
#if defined(NDEBUG)
                        "release"
#else
                        "debug"
#endif
                    },
                    {"headless_only", true},
                }},
                {"defaults", json{
                    {"resource_root_dir", default_root.string()},
                    {"scene_name", "main_scene"},
                    {"offline_render", json{
                        {"spp", 16},
                        {"film_width", 0},
                        {"film_height", 0},
                    }},
                }},
                {"supported_node_types", json::array({"perspective_camera", "mesh_object", "emissive_mesh_object"})},
            },
            .message = "Project info ready.",
        };
    }

    CommandResult handle_session_create(const json& params) {
        std::filesystem::path resource_root_dir(
            params.value("resource_root_dir", std::string(resource::ResourceManager::kDefaultResourceRootDir))
        );
        const std::string session_id = "session_" + std::to_string(m_next_session_number++);
        auto session = std::make_unique<SessionState>(resource_root_dir.lexically_normal());
        auto* session_ptr = session.get();
        m_sessions.emplace(session_id, std::move(session));

        return CommandResult{
            .data = json{
                {"session_id", session_id},
                {"resource_root_dir", session_ptr->resource_root_dir.string()},
                {"scene_name", session_ptr->scene_name},
            },
            .message = "Session created.",
        };
    }

    CommandResult handle_session_reset(const json& params) {
        SessionState& session = require_session(params);
        session.reset(params.value("scene_name", std::string("main_scene")));
        return CommandResult{
            .data = json{
                {"session_id", require_string_field(params, "session_id")},
                {"scene", summarize_scene(session)},
            },
            .message = "Session reset.",
        };
    }

    CommandResult handle_scene_inspect(const json& params) {
        SessionState& session = require_session(params);
        return CommandResult{
            .data = summarize_scene(session),
            .message = "Scene summary ready.",
        };
    }

    CommandResult handle_scene_replace(const json& params) {
        SessionState& session = require_session(params);
        const json& scene_spec = require_object_field(params, "scene_spec");
        const std::string scene_name = scene_spec.value("scene_name", std::string("main_scene"));

        try {
            session.reset(scene_name);
            const json& nodes = require_array_field(scene_spec, "nodes");
            std::size_t implicit_active_camera_count = 0;

            for (const auto& node_spec : nodes) {
                if (!node_spec.is_object()) {
                    throw BridgeError("validation_error", "scene_spec.nodes entries must be objects.");
                }
                add_scene_node(session, node_spec, implicit_active_camera_count);
            }

            session.scene().scene_graph().update_world_transforms();
        } catch (const BridgeError&) {
            throw;
        } catch (const std::exception& e) {
            throw BridgeError("validation_error", e.what());
        }

        return CommandResult{
            .data = summarize_scene(session),
            .message = "Scene replaced.",
        };
    }

    CommandResult handle_scene_import_pbpt(const json& params) {
        SessionState& session = require_session(params);
        const std::string scene_xml_path = require_string_field(params, "scene_xml_path");
        const std::string mode = params.value("mode", std::string("replace"));

        if (mode != "replace" && mode != "merge") {
            throw BridgeError("validation_error", "scene_import_pbpt mode must be 'replace' or 'merge'.");
        }

        if (mode == "replace") {
            session.reset(std::filesystem::path(scene_xml_path).stem().string().empty()
                              ? std::string("imported_scene")
                              : std::filesystem::path(scene_xml_path).stem().string());
        }

        try {
            auto package = framework::integration::load_scene(scene_xml_path, session.scene(), *session.resources);
            if (mode == "replace") {
                session.compatible_info = std::move(package.compatible_info);
            } else if (!session.compatible_info.has_value()) {
                session.compatible_info = std::move(package.compatible_info);
            } else {
                session.compatible_info.reset();
                session.import_note = "Compatible passthrough metadata cleared after merge import.";
            }

            return CommandResult{
                .data = json{
                    {"scene", summarize_scene(session)},
                    {"import", json{
                        {"mode", mode},
                        {"scene_xml_path", scene_xml_path},
                        {"imported_shape_count", package.result.imported_shape_count},
                        {"imported_light_shape_count", package.result.imported_light_shape_count},
                        {"note", session.import_note},
                    }},
                },
                .message = "PBPT scene imported.",
            };
        } catch (const BridgeError&) {
            throw;
        } catch (const std::exception& e) {
            throw BridgeError("import_failed", e.what());
        }
    }

    CommandResult handle_scene_export_pbpt(const json& params) {
        SessionState& session = require_session(params);
        const std::string scene_xml_path = require_string_field(params, "scene_xml_path");
        const int film_width = params.value("film_width", 0);
        const int film_height = params.value("film_height", 0);
        const int spp = params.value("spp", 16);

        try {
            auto result = framework::integration::build_scene_result(
                session.scene(),
                *session.resources,
                session.compatible_info ? &*session.compatible_info : nullptr,
                film_width,
                film_height,
                spp
            );
            if (std::filesystem::path(scene_xml_path).has_parent_path()) {
                std::filesystem::create_directories(std::filesystem::path(scene_xml_path).parent_path());
            }
            framework::integration::write_scene_result(result, scene_xml_path);
            return CommandResult{
                .data = json{
                    {"scene_xml_path", scene_xml_path},
                    {"shape_count", result.scene.resources.shape_instances.size()},
                    {"spp", result.spp},
                },
                .message = "PBPT scene exported.",
            };
        } catch (const std::exception& e) {
            throw BridgeError("export_failed", e.what());
        }
    }

    CommandResult handle_offline_render(const json& params) {
        SessionState& session = require_session(params);
        const std::string action = params.value("action", std::string("status"));

        if (action == "status") {
            return CommandResult{.data = summarize_offline_render(session), .message = "Offline render status ready."};
        }

        if (action == "cancel") {
            session.offline_render_service->request_cancel();
            return CommandResult{
                .data = summarize_offline_render(session),
                .message = "Offline render cancel requested.",
            };
        }

        if (action != "start") {
            throw BridgeError("validation_error", "offline_render action must be 'start', 'status', or 'cancel'.");
        }

        framework::integration::OfflineRenderConfig config{};
        config.scene_xml_path = require_string_field(params, "scene_xml_path");
        config.output_exr_path = require_string_field(params, "output_exr_path");
        config.spp = params.value("spp", 16);
        config.film_width = params.value("film_width", 0);
        config.film_height = params.value("film_height", 0);
        config.compatible_info = session.compatible_info ? &*session.compatible_info : nullptr;

        const bool started = session.offline_render_service->start(session.scene(), *session.resources, config);
        if (!started) {
            throw BridgeError("render_failed", session.offline_render_service->last_message());
        }

        session.active_job_id = "job_" + std::to_string(session.next_job_number++);
        json status = summarize_offline_render(session);
        status["job_id"] = *session.active_job_id;

        return CommandResult{
            .data = std::move(status),
            .message = "Offline render started.",
        };
    }

    void add_scene_node(SessionState& session, const json& node_spec, std::size_t& implicit_active_camera_count) {
        const std::string type = require_string_field(node_spec, "type");
        const std::string name = node_spec.value("name", type + "_" + std::to_string(session.scene().game_object_count() + 1));
        auto& game_object = session.scene().create_game_object(name);
        apply_transform(game_object, node_spec);

        if (type == "perspective_camera") {
            auto& camera = game_object.add_component<framework::component::PerspectiveCamera>();
            const bool active = node_spec.contains("active") ? node_spec.at("active").get<bool>() : (implicit_active_camera_count == 0);
            camera.set_active(active);
            camera.fov_degrees() = node_spec.value("fov_degrees", camera.fov_degrees());
            camera.near_bound() = node_spec.value("near", camera.near_bound());
            camera.far_bound() = node_spec.value("far", camera.far_bound());
            if (active) {
                ++implicit_active_camera_count;
            }
            return;
        }

        if (type == "mesh_object" || type == "emissive_mesh_object") {
            const std::string mesh_path = require_string_field(node_spec, "mesh_path");
            const auto base_color = parse_vec4(node_spec.value("base_color", json(nullptr)),
                                               pbpt::math::Vec4{1.0f, 1.0f, 1.0f, 1.0f});
            const auto mesh_handle =
                session.resources->create_from_relative_path<resource::MeshResourceKind>(mesh_path);
            (void)game_object.add_component<framework::component::MeshRenderer>(*session.resources, mesh_handle, base_color);

            const bool want_pbpt_mesh = (type == "emissive_mesh_object")
                                            ? true
                                            : node_spec.value("pbpt_mesh", true);
            if (want_pbpt_mesh) {
                (void)game_object.add_component<framework::component::PbptMesh>();
            }

            if (type == "emissive_mesh_object") {
                auto& light = game_object.add_component<framework::component::PbptLight>();
                if (node_spec.contains("radiance_spectrum")) {
                    light.set_radiance_spectrum(parse_spectrum(node_spec.at("radiance_spectrum")));
                }
            }
            return;
        }

        throw BridgeError("validation_error", "Unsupported scene node type '" + type + "'.");
    }

    static framework::component::PbptSpectrum parse_spectrum(const json& value) {
        if (!value.is_array()) {
            throw BridgeError("validation_error", "radiance_spectrum must be an array.");
        }

        framework::component::PbptSpectrum spectrum{};
        spectrum.reserve(value.size());
        for (const auto& entry : value) {
            if (!entry.is_object()) {
                throw BridgeError("validation_error", "radiance_spectrum entries must be objects.");
            }
            spectrum.push_back(framework::component::PbptSpectrumPoint{
                .lambda_nm = entry.value("lambda_nm", 0.0f),
                .value = entry.value("value", 0.0f),
            });
        }
        framework::component::validate_pbpt_spectrum(spectrum, "radiance_spectrum");
        return spectrum;
    }

    static void apply_transform(framework::core::GameObject& game_object, const json& node_spec) {
        const json transform = node_spec.contains("transform") ? node_spec.at("transform") : json::object();
        if (!transform.is_object()) {
            throw BridgeError("validation_error", "transform must be an object.");
        }

        auto node = game_object.node();
        node.set_local_position(parse_vec3(transform.value("position", json(nullptr)), pbpt::math::Vec3{0.0f, 0.0f, 0.0f}));
        node.set_local_rotation(parse_quat(transform.value("rotation_quat", json(nullptr)),
                                           pbpt::math::Quat{1.0f, 0.0f, 0.0f, 0.0f}));
        node.set_local_scale(parse_vec3(transform.value("scale", json(nullptr)), pbpt::math::Vec3{1.0f, 1.0f, 1.0f}));
    }

    json summarize_scene(SessionState& session) const {
        auto& scene = session.scene();
        scene.scene_graph().update_world_transforms();

        json game_objects = json::array();
        std::size_t active_camera_count = 0;
        std::size_t exportable_shape_count = 0;
        std::size_t emissive_shape_count = 0;
        std::vector<std::string> issues{};
        json active_camera = nullptr;

        for (const auto& go_ptr : scene.game_objects()) {
            if (!go_ptr) {
                continue;
            }
            const auto& go = *go_ptr;
            json components = json::array();
            const auto maybe_name = scene.game_object_name(go.id());
            const std::string object_name = maybe_name.has_value() ? std::string(*maybe_name) : "GameObject";

            if (const auto* camera = go.get_component<framework::component::PerspectiveCamera>(); camera != nullptr) {
                components.push_back("PerspectiveCamera");
                if (camera->active()) {
                    ++active_camera_count;
                    active_camera = json{
                        {"game_object_id", go.id()},
                        {"name", object_name},
                        {"fov_degrees", camera->fov_degrees()},
                        {"near", camera->near_bound()},
                        {"far", camera->far_bound()},
                    };
                }
            }

            if (const auto* mesh_renderer = go.get_component<framework::component::MeshRenderer>(); mesh_renderer != nullptr) {
                components.push_back("MeshRenderer");
                if (!session.resources->alive<resource::MeshResourceKind>(mesh_renderer->mesh_handle())) {
                    issues.push_back("Mesh handle is not alive for '" + object_name + "'.");
                }
            }

            const auto* pbpt_mesh = go.get_component<framework::component::PbptMesh>();
            if (pbpt_mesh != nullptr) {
                components.push_back("PbptMesh");
                ++exportable_shape_count;
            }

            const auto* pbpt_light = go.get_component<framework::component::PbptLight>();
            if (pbpt_light != nullptr) {
                components.push_back("PbptLight");
                ++emissive_shape_count;
                if (pbpt_mesh == nullptr) {
                    issues.push_back("PbptLight without PbptMesh on '" + object_name + "'.");
                }
            }

            const auto node = go.node();
            game_objects.push_back(json{
                {"game_object_id", go.id()},
                {"name", object_name},
                {"enabled", go.enabled()},
                {"components", components},
                {"transform", json{
                    {"position", vec3_to_json(node.local_position())},
                    {"rotation_quat", quat_to_json(node.local_rotation())},
                    {"scale", vec3_to_json(node.local_scale())},
                }},
            });
        }

        if (active_camera_count == 0) {
            issues.push_back("Scene requires exactly one active perspective camera.");
        } else if (active_camera_count > 1) {
            issues.push_back("Scene has multiple active perspective cameras.");
        }

        if (exportable_shape_count == 0) {
            issues.push_back("Scene has no exportable PBPT meshes.");
        }
        if (emissive_shape_count == 0) {
            issues.push_back("Scene has no PBPT area emitter.");
        }

        return json{
            {"scene_name", session.scene_name},
            {"resource_root_dir", session.resource_root_dir.string()},
            {"game_object_count", scene.game_object_count()},
            {"game_objects", std::move(game_objects)},
            {"active_camera", active_camera},
            {"compatible_info", json{
                {"enabled", session.compatible_info.has_value()},
                {"passthrough_shape_count",
                 session.compatible_info ? session.compatible_info->passthrough_shape_ids.size() : 0u},
                {"note", session.import_note},
            }},
            {"export_checks", json{
                {"active_camera_count", active_camera_count},
                {"exportable_shape_count", exportable_shape_count},
                {"emissive_shape_count", emissive_shape_count},
                {"can_export_pbpt", active_camera_count == 1 && exportable_shape_count > 0},
                {"can_start_offline_render",
                 active_camera_count == 1 && exportable_shape_count > 0 && emissive_shape_count > 0},
                {"issues", issues},
            }},
        };
    }

    json summarize_offline_render(const SessionState& session) const {
        return json{
            {"job_id", session.active_job_id.value_or("")},
            {"state", framework::integration::to_state_label(session.offline_render_service->state())},
            {"progress_01", session.offline_render_service->progress_01()},
            {"message", session.offline_render_service->last_message()},
            {"is_running", session.offline_render_service->is_running()},
        };
    }
};

} // namespace rtr::mcp
