#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "imgui.h"

#include "rtr/app/app_runtime.hpp"
#include "rtr/editor/core/editor_capture.hpp"
#include "rtr/editor/core/editor_host.hpp"
#include "rtr/editor/panel/hierarchy_panel.hpp"
#include "rtr/editor/panel/inspector_panel.hpp"
#include "rtr/editor/panel/logger_panel.hpp"
#include "rtr/editor/panel/scene_view_panel.hpp"
#include "rtr/editor/panel/stats_panel.hpp"
#include "rtr/editor/render/forward_editor_pipeline.hpp"
#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/framework/integration/pbpt/pbpt_offline_render_service.hpp"
#include "rtr/framework/integration/pbpt/serde/scene_loader.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/input/input_types.hpp"

namespace {
constexpr const char* kCboxSceneRootRel       = "pbpt_scene/cbox";
constexpr const char* kCboxSceneXmlFilename   = "cbox.xml";
constexpr const char* kOutputExrPath          = "output/cbox_offline.exr";
constexpr const char* kOutputSceneXmlFilename = "output/cbox_output.xml";

constexpr std::size_t kPathBufferSize = 1024;

void set_path_buffer(std::array<char, kPathBufferSize>& buffer, const std::string& value) {
    std::fill(buffer.begin(), buffer.end(), '\0');
    if (value.empty()) {
        return;
    }
    std::strncpy(buffer.data(), value.c_str(), buffer.size() - 1);
}

struct ExportResolutionInfo {
    int   window_w{0};
    int   window_h{0};
    float scale_x{1.0f};
    float scale_y{1.0f};
    int   framebuffer_w{0};
    int   framebuffer_h{0};
    int   export_w{0};
    int   export_h{0};
};

ExportResolutionInfo resolve_export_resolution(const rtr::rhi::Window& window, uint32_t fallback_w,
                                               uint32_t fallback_h) {
    ExportResolutionInfo info{};

    const auto [fb_w, fb_h] = window.framebuffer_size();
    info.framebuffer_w      = fb_w;
    info.framebuffer_h      = fb_h;

    GLFWwindow* glfw_window = window.window();
    if (glfw_window != nullptr) {
        glfwGetWindowSize(glfw_window, &info.window_w, &info.window_h);
        glfwGetWindowContentScale(glfw_window, &info.scale_x, &info.scale_y);
    }

    if (info.window_w > 0 && info.window_h > 0) {
        info.export_w = info.window_w;
        info.export_h = info.window_h;
    } else if (fb_w > 0 && fb_h > 0) {
        info.export_w = fb_w;
        info.export_h = fb_h;
    } else {
        info.export_w = static_cast<int>(fallback_w);
        info.export_h = static_cast<int>(fallback_h);
    }

    return info;
}

rtr::framework::component::Camera* find_unique_active_camera(rtr::framework::core::Scene& scene) {
    rtr::framework::component::Camera* active_camera = nullptr;
    for (const auto node_id : scene.scene_graph().active_nodes()) {
        auto* go = scene.find_game_object(node_id);
        if (go == nullptr || !go->enabled()) {
            continue;
        }
        auto* camera = go->get_component<rtr::framework::component::Camera>();
        if (camera == nullptr || !camera->enabled() || !camera->active()) {
            continue;
        }
        if (active_camera != nullptr) {
            return nullptr;
        }
        active_camera = camera;
    }
    return active_camera;
}

class OfflineRenderPanel final : public rtr::editor::IEditorPanel {
private:
    struct UiState {
        std::array<char, kPathBufferSize> scene_xml_path{};
        std::array<char, kPathBufferSize> output_exr_path{};
        std::array<char, kPathBufferSize> output_scene_xml_path{};
        int                               spp{16};
    };

    rtr::framework::integration::PbptOfflineRenderService& m_offline_render_service;
    rtr::app::AppRuntime&                                  m_runtime;
    const rtr::framework::integration::LoadSummary&        m_import_result;
    uint32_t                                               m_scene_width{0};
    uint32_t                                               m_scene_height{0};
    bool                                                   m_visible{true};
    UiState                                                m_ui_state{};

public:
    OfflineRenderPanel(rtr::framework::integration::PbptOfflineRenderService& offline_render_service,
                       rtr::app::AppRuntime& runtime,
                       const rtr::framework::integration::LoadSummary& import_result, uint32_t scene_width,
                       uint32_t scene_height, const std::string& scene_xml_path, const std::string& output_exr_path,
                       const std::string& output_scene_xml_path)
        : m_offline_render_service(offline_render_service),
          m_runtime(runtime),
          m_import_result(import_result),
          m_scene_width(scene_width),
          m_scene_height(scene_height) {
        set_path_buffer(m_ui_state.scene_xml_path, scene_xml_path);
        set_path_buffer(m_ui_state.output_exr_path, output_exr_path);
        set_path_buffer(m_ui_state.output_scene_xml_path, output_scene_xml_path);
    }

    std::string_view id() const override { return "offline_render"; }

    int order() const override { return 250; }

    bool visible() const override { return m_visible; }

    void set_visible(bool visible) override { m_visible = visible; }

    void on_imgui(rtr::editor::EditorContext& /*ctx*/) override {
        if (!m_visible) {
            return;
        }

        const auto state = m_offline_render_service.state();
        ImGui::Begin("Offline Render", &m_visible);
        ImGui::Text("Imported shapes: %zu", m_import_result.imported_shape_count);
        ImGui::Text("Imported lights: %zu", m_import_result.imported_light_shape_count);
        ImGui::Text("Scene film: %u x %u", m_scene_width, m_scene_height);

        ImGui::InputText("Scene XML", m_ui_state.scene_xml_path.data(), m_ui_state.scene_xml_path.size());
        ImGui::InputText("Output EXR", m_ui_state.output_exr_path.data(), m_ui_state.output_exr_path.size());
        ImGui::InputText("Output Scene XML", m_ui_state.output_scene_xml_path.data(),
                         m_ui_state.output_scene_xml_path.size());
        ImGui::InputInt("SPP", &m_ui_state.spp);
        m_ui_state.spp = std::clamp(m_ui_state.spp, 1, 4096);

        const auto export_resolution = resolve_export_resolution(m_runtime.renderer().window(), m_scene_width, m_scene_height);
        ImGui::Text("Window: %d x %d", export_resolution.window_w, export_resolution.window_h);
        ImGui::Text("Scale: %.2f x %.2f", export_resolution.scale_x, export_resolution.scale_y);
        ImGui::Text("Framebuffer: %d x %d", export_resolution.framebuffer_w, export_resolution.framebuffer_h);
        ImGui::Text("Export Film: %d x %d", export_resolution.export_w, export_resolution.export_h);

        const bool can_render = is_render_start_allowed(state);
        const bool can_cancel = state == rtr::framework::integration::OfflineRenderState::Running;

        if (!can_render) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Render")) {
            auto* active_scene = m_runtime.world().active_scene();
            if (active_scene == nullptr) {
                throw std::runtime_error("No active scene to export for offline render.");
            }

            const rtr::framework::integration::OfflineRenderConfig config{
                .scene_xml_path  = std::string(m_ui_state.output_scene_xml_path.data()),
                .output_exr_path = std::string(m_ui_state.output_exr_path.data()),
                .spp             = m_ui_state.spp,
                .film_width      = export_resolution.export_w,
                .film_height     = export_resolution.export_h};
            (void)m_offline_render_service.start(*active_scene, m_runtime.resource_manager(), config);
        }
        if (!can_render) {
            ImGui::EndDisabled();
        }

        ImGui::SameLine();
        if (!can_cancel) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Cancel")) {
            m_offline_render_service.request_cancel();
        }
        if (!can_cancel) {
            ImGui::EndDisabled();
        }

        ImGui::ProgressBar(std::clamp(m_offline_render_service.progress_01(), 0.0f, 1.0f), ImVec2(-1.0f, 0.0f));
        ImGui::Text("State: %s", to_state_label(state));
        ImGui::TextWrapped("Message: %s", m_offline_render_service.last_message().c_str());
        ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
        ImGui::End();
    }
};

}  // namespace

int main() {
    try {
        rtr::app::AppRuntime runtime(rtr::app::AppRuntimeConfig{
            .window_width  = 1280,
            .window_height = 720,
            .window_title  = "RTR Framework Offline CBox",
        });
        rtr::framework::integration::PbptOfflineRenderService offline_render_service{};

        auto& resource_manager = runtime.resource_manager();
        const auto import_xml_path =
            (resource_manager.resource_root_dir() / kCboxSceneRootRel / kCboxSceneXmlFilename).string();

        rtr::framework::integration::LoadOptions import_options{};
        import_options.free_look_input_state = &runtime.input_system().state();

        auto&      scene = runtime.world().create_scene("cbox_scene");
        const auto import_package =
            rtr::framework::integration::load_scene(import_xml_path, scene, resource_manager, import_options);
        const auto& import_result = import_package.result;

        const uint32_t scene_width  = import_result.sensor ? import_result.sensor->film_width : 1280;
        const uint32_t scene_height = import_result.sensor ? import_result.sensor->film_height : 720;

        const auto remove_required_imported_game_object = [&](const char* name) {
            const auto it = import_result.imported_game_object_id_by_name.find(name);
            if (it == import_result.imported_game_object_id_by_name.end()) {
                throw std::runtime_error(std::string("Imported cbox scene does not contain ") + name + ".");
            }
            if (!scene.destroy_game_object(it->second)) {
                throw std::runtime_error(std::string("Failed to destroy imported ") + name + " game object.");
            }
        };

        // // Remove selected cbox primitives from the imported scene.
        // remove_required_imported_game_object("cbox_floor");
        // remove_required_imported_game_object("cbox_redwall");

        if (find_unique_active_camera(scene) == nullptr) {
            throw std::runtime_error("Imported cbox scene has no active camera.");
        }

        auto editor_host = std::make_shared<rtr::editor::EditorHost>(runtime);
        editor_host->register_panel(std::make_unique<rtr::editor::SceneViewPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::HierarchyPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::InspectorPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::StatsPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::LoggerPanel>());
        editor_host->register_panel(std::make_unique<OfflineRenderPanel>(
            offline_render_service, runtime, import_result, scene_width, scene_height,
            (resource_manager.resource_root_dir() / kCboxSceneRootRel / kCboxSceneXmlFilename).string(),
            (resource_manager.resource_root_dir() / kCboxSceneRootRel / kOutputExrPath).string(),
            (resource_manager.resource_root_dir() / kCboxSceneRootRel / kOutputSceneXmlFilename).string()));

        auto editor_pipeline = std::make_unique<rtr::editor::render::ForwardEditorPipeline>(
            runtime.renderer().build_pipeline_runtime(), editor_host);
        rtr::editor::bind_input_capture_to_editor(runtime.input_system(), *editor_pipeline);
        runtime.set_pipeline(std::move(editor_pipeline));

        runtime.set_callbacks(rtr::app::RuntimeCallbacks{
            .on_post_update =
                [editor_host](rtr::app::RuntimeContext& ctx) {
                    editor_host->begin_frame(rtr::editor::EditorFrameData{
                        .frame_serial  = ctx.frame_serial,
                        .delta_seconds = ctx.delta_seconds,
                        .paused        = ctx.paused,
                    });

                    auto* active_scene = ctx.world.active_scene();
                    if (active_scene == nullptr) {
                        throw std::runtime_error("No active scene.");
                    }

                    auto* active_camera = find_unique_active_camera(*active_scene);
                    if (active_camera != nullptr) {
                        const auto [fb_w, fb_h] = ctx.renderer.window().framebuffer_size();
                        if (fb_w > 0 && fb_h > 0) {
                            if (auto* perspective =
                                    dynamic_cast<rtr::framework::component::PerspectiveCamera*>(active_camera);
                                perspective != nullptr) {
                                perspective->aspect_ratio() = static_cast<float>(fb_w) / static_cast<float>(fb_h);
                            }
                        }
                    }
                },
            .on_pre_render =
                [](rtr::app::RuntimeContext& ctx) {
                    if (ctx.input.state().key_down(rtr::system::input::KeyCode::ESCAPE)) {
                        ctx.renderer.window().close();
                    }
                },
        });

        const auto result = runtime.run();
        if (!result.ok) {
            throw std::runtime_error(result.error_message);
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
