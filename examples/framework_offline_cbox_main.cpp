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
#include "pugixml.hpp"

#include "rtr/editor/editor_attach.hpp"
#include "rtr/editor/editor_host.hpp"
#include "rtr/editor/hierarchy_panel.hpp"
#include "rtr/editor/inspector_panel.hpp"
#include "rtr/editor/logger_panel.hpp"
#include "rtr/editor/scene_view_panel.hpp"
#include "rtr/editor/stats_panel.hpp"
#include "rtr/framework/core/camera.hpp"
#include "rtr/framework/core/engine.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view_builder.hpp"
#include "rtr/framework/integration/pbpt/pbpt_offline_render_service.hpp"
#include "rtr/framework/integration/pbpt/pbpt_scene_importer.hpp"
#include "rtr/system/input/input_system.hpp"
#include "rtr/system/input/input_types.hpp"
#include "rtr/system/render/pipeline/forward/forward_pipeline.hpp"
#include "rtr/system/render/renderer.hpp"

namespace {

constexpr uint32_t kMaxFramesInFlight = 2;

constexpr const char* kCboxSceneRootRel =
    "pbpt_scene/cbox";
constexpr const char* kCboxSceneXmlFilename =
    "cbox.xml";
constexpr const char* kOutputExrPath =
    "output/cbox_offline.exr";
constexpr const char* kOutputSceneXmlFilename =
    "output/cbox_output.xml";

constexpr std::size_t kPathBufferSize = 1024;

const char* to_state_label(rtr::framework::integration::OfflineRenderState state) {
    using rtr::framework::integration::OfflineRenderState;
    switch (state) {
        case OfflineRenderState::Idle:
            return "Idle";
        case OfflineRenderState::Running:
            return "Running";
        case OfflineRenderState::Succeeded:
            return "Succeeded";
        case OfflineRenderState::Failed:
            return "Failed";
        case OfflineRenderState::Canceled:
            return "Canceled";
    }
    return "Unknown";
}

void set_path_buffer(std::array<char, kPathBufferSize>& buffer, const std::string& value) {
    std::fill(buffer.begin(), buffer.end(), '\0');
    if (value.empty()) {
        return;
    }
    std::strncpy(buffer.data(), value.c_str(), buffer.size() - 1);
}

bool is_render_start_allowed(rtr::framework::integration::OfflineRenderState state) {
    using rtr::framework::integration::OfflineRenderState;
    return state == OfflineRenderState::Idle ||
        state == OfflineRenderState::Succeeded ||
        state == OfflineRenderState::Failed ||
        state == OfflineRenderState::Canceled;
}

struct ExportResolutionInfo {
    int window_w{0};
    int window_h{0};
    float scale_x{1.0f};
    float scale_y{1.0f};
    int framebuffer_w{0};
    int framebuffer_h{0};
    int export_w{0};
    int export_h{0};
};

ExportResolutionInfo resolve_export_resolution(
    const rtr::rhi::Window& window,
    uint32_t fallback_w,
    uint32_t fallback_h
) {
    ExportResolutionInfo info{};

    const auto [fb_w, fb_h] = window.framebuffer_size();
    info.framebuffer_w = fb_w;
    info.framebuffer_h = fb_h;

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

std::pair<uint32_t, uint32_t> resolve_resolution_from_pbpt_scene_xml(const char* xml_path) {
    pugi::xml_document doc;
    const auto parse_result = doc.load_file(xml_path);
    if (!parse_result) {
        throw std::runtime_error(
            std::string("Failed to load scene XML for resolution: ") + parse_result.description()
        );
    }

    const auto scene_node = doc.child("scene");
    const auto sensor_node = scene_node.child("sensor");
    const auto film_node = sensor_node.child("film");
    if (!scene_node || !sensor_node || !film_node) {
        throw std::runtime_error("scene/sensor/film node is missing in input cbox.xml.");
    }

    int width = -1;
    int height = -1;
    for (const auto& integer_node : film_node.children("integer")) {
        const std::string name = integer_node.attribute("name").value();
        if (name == "width") {
            width = integer_node.attribute("value").as_int(-1);
        } else if (name == "height") {
            height = integer_node.attribute("value").as_int(-1);
        }
    }

    if (width <= 0 || height <= 0) {
        throw std::runtime_error("Input cbox.xml film width/height must be positive.");
    }
    return {
        static_cast<uint32_t>(width),
        static_cast<uint32_t>(height)
    };
}

class OfflineRenderPanel final : public rtr::editor::IEditorPanel {
private:
    struct UiState {
        std::array<char, kPathBufferSize> scene_xml_path{};
        std::array<char, kPathBufferSize> output_exr_path{};
        std::array<char, kPathBufferSize> output_scene_xml_path{};
        int spp{16};
    };

    rtr::framework::integration::PbptOfflineRenderService& m_offline_render_service;
    rtr::framework::core::Engine& m_engine;
    rtr::system::render::Renderer& m_renderer;
    rtr::resource::ResourceManager& m_resource_manager;
    const rtr::framework::integration::PbptImportResult& m_import_result;
    uint32_t m_scene_width{0};
    uint32_t m_scene_height{0};
    bool m_visible{true};
    UiState m_ui_state{};

public:
    OfflineRenderPanel(
        rtr::framework::integration::PbptOfflineRenderService& offline_render_service,
        rtr::framework::core::Engine& engine,
        rtr::system::render::Renderer& renderer,
        rtr::resource::ResourceManager& resource_manager,
        const rtr::framework::integration::PbptImportResult& import_result,
        uint32_t scene_width,
        uint32_t scene_height,
        const std::string& scene_xml_path,
        const std::string& output_exr_path,
        const std::string& output_scene_xml_path
    )
        : m_offline_render_service(offline_render_service),
          m_engine(engine),
          m_renderer(renderer),
          m_resource_manager(resource_manager),
          m_import_result(import_result),
          m_scene_width(scene_width),
          m_scene_height(scene_height) {
        set_path_buffer(m_ui_state.scene_xml_path, scene_xml_path);
        set_path_buffer(m_ui_state.output_exr_path, output_exr_path);
        set_path_buffer(m_ui_state.output_scene_xml_path, output_scene_xml_path);
    }

    std::string_view id() const override {
        return "offline_render";
    }

    int order() const override {
        return 250;
    }

    bool visible() const override {
        return m_visible;
    }

    void set_visible(bool visible) override {
        m_visible = visible;
    }

    void on_imgui(rtr::editor::EditorContext& /*ctx*/) override {
        if (!m_visible) {
            return;
        }

        const auto state = m_offline_render_service.state();
        ImGui::Begin("Offline Render", &m_visible);
        ImGui::Text("Imported shapes: %zu", m_import_result.imported_shape_count);
        ImGui::Text("Imported lights: %zu", m_import_result.imported_light_shape_count);
        ImGui::Text("Scene film: %u x %u", m_scene_width, m_scene_height);

        ImGui::InputText(
            "Scene XML",
            m_ui_state.scene_xml_path.data(),
            m_ui_state.scene_xml_path.size()
        );
        ImGui::InputText(
            "Output EXR",
            m_ui_state.output_exr_path.data(),
            m_ui_state.output_exr_path.size()
        );
        ImGui::InputText(
            "Output Scene XML",
            m_ui_state.output_scene_xml_path.data(),
            m_ui_state.output_scene_xml_path.size()
        );
        ImGui::InputInt("SPP", &m_ui_state.spp);
        m_ui_state.spp = std::clamp(m_ui_state.spp, 1, 4096);

        const auto export_resolution = resolve_export_resolution(
            m_renderer.window(),
            m_scene_width,
            m_scene_height
        );
        ImGui::Text("Window: %d x %d", export_resolution.window_w, export_resolution.window_h);
        ImGui::Text("Scale: %.2f x %.2f", export_resolution.scale_x, export_resolution.scale_y);
        ImGui::Text(
            "Framebuffer: %d x %d",
            export_resolution.framebuffer_w,
            export_resolution.framebuffer_h
        );
        ImGui::Text("Export Film: %d x %d", export_resolution.export_w, export_resolution.export_h);

        const bool can_render = is_render_start_allowed(state);
        const bool can_cancel = state == rtr::framework::integration::OfflineRenderState::Running;

        if (!can_render) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Render")) {
            auto* active_scene = m_engine.world().active_scene();
            if (active_scene == nullptr) {
                throw std::runtime_error("No active scene to export for offline render.");
            }

            const rtr::framework::integration::OfflineRenderConfig config{
                .scene_xml_path = std::string(m_ui_state.output_scene_xml_path.data()),
                .output_exr_path = std::string(m_ui_state.output_exr_path.data()),
                .spp = m_ui_state.spp,
                .film_width = export_resolution.export_w,
                .film_height = export_resolution.export_h
            };
            (void)m_offline_render_service.start(*active_scene, m_resource_manager, config);
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

        ImGui::ProgressBar(
            std::clamp(m_offline_render_service.progress_01(), 0.0f, 1.0f),
            ImVec2(-1.0f, 0.0f)
        );
        ImGui::Text("State: %s", to_state_label(state));
        ImGui::TextWrapped("Message: %s", m_offline_render_service.last_message().c_str());
        ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
        ImGui::End();
    }
};

} // namespace

int main() {
    try {
        rtr::resource::ResourceManager resource_manager(kMaxFramesInFlight);
        rtr::framework::integration::PbptOfflineRenderService offline_render_service{};
        
        const auto import_location = rtr::framework::integration::make_pbpt_scene_location(
            kCboxSceneRootRel,
            kCboxSceneXmlFilename
        );

        const auto [scene_width, scene_height] =
            resolve_resolution_from_pbpt_scene_xml(
                (resource_manager.resource_root_dir() / import_location.scene_root_rel_to_resource_dir / import_location.xml_filename).c_str()
            );

        auto renderer = std::make_unique<rtr::system::render::Renderer>(
            static_cast<int>(scene_width),
            static_cast<int>(scene_height),
            "RTR Framework Offline CBox",
            kMaxFramesInFlight
        );

        auto runtime_pipeline = std::make_unique<rtr::system::render::ForwardPipeline>(
            renderer->build_pipeline_runtime(),
            rtr::system::render::ForwardPipelineConfig{}
        );
        auto* forward_pipeline = runtime_pipeline.get();
        forward_pipeline->set_resource_manager(&resource_manager);

        auto input_system = std::make_unique<rtr::system::input::InputSystem>(&renderer->window());

        rtr::framework::integration::PbptImportOptions import_options{};
        import_options.free_look_input_state = &input_system->state();

        rtr::framework::core::Engine engine(rtr::framework::core::EngineConfig{
            .window_width = scene_width,
            .window_height = scene_height,
            .window_title = "RTR Framework Offline CBox",
            .max_frames_in_flight = kMaxFramesInFlight
        });
        engine.world().set_resource_manager(&resource_manager);

        auto& scene = engine.world().create_scene("cbox_scene");
        const auto import_result = rtr::framework::integration::import_pbpt_scene_xml_to_scene(
            import_location,
            scene,
            resource_manager,
            import_options
        );

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

        if (scene.active_camera() == nullptr) {
            throw std::runtime_error("Imported cbox scene has no active camera.");
        }

        auto editor_host = std::make_shared<rtr::editor::EditorHost>();
        editor_host->bind_runtime(
            &engine.world(),
            &resource_manager,
            renderer.get(),
            input_system.get()
        );
        editor_host->register_panel(std::make_unique<rtr::editor::SceneViewPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::HierarchyPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::InspectorPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::StatsPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::LoggerPanel>());
        editor_host->register_panel(std::make_unique<OfflineRenderPanel>(
            offline_render_service,
            engine,
            *renderer,
            resource_manager,
            import_result,
            scene_width,
            scene_height,
            (resource_manager.resource_root_dir() / import_location.scene_root_rel_to_resource_dir / import_location.xml_filename).string(),
            (resource_manager.resource_root_dir() / import_location.scene_root_rel_to_resource_dir / kOutputExrPath).string(),
            (resource_manager.resource_root_dir() / import_location.scene_root_rel_to_resource_dir / kOutputSceneXmlFilename).string()
        ));
        auto editor_pipeline = rtr::editor::create_editor_pipeline(
            renderer->build_pipeline_runtime(),
            std::move(runtime_pipeline),
            editor_host
        );
        rtr::editor::bind_input_capture_to_editor(*input_system, *editor_pipeline);
        renderer->set_pipeline(std::move(editor_pipeline));

        engine.set_loop_hooks(rtr::framework::core::Engine::LoopHooks{
            .input_begin = [&]() { input_system->begin_frame(); },
            .input_poll = [&]() { renderer->window().poll_events(); },
            .input_end = [&]() { input_system->end_frame(); },
            .render = [&]() {
                static std::uint64_t frame_serial = 0;

                auto* active_scene = engine.world().active_scene();
                if (active_scene == nullptr) {
                    throw std::runtime_error("No active scene.");
                }

                auto* active_camera = active_scene->active_camera();
                if (active_camera == nullptr) {
                    throw std::runtime_error("Active scene has no active camera.");
                }

                const auto [fb_w, fb_h] = renderer->window().framebuffer_size();
                if (fb_w > 0 && fb_h > 0) {
                    active_camera->set_aspect_ratio(
                        static_cast<float>(fb_w) / static_cast<float>(fb_h)
                    );
                }

                editor_host->begin_frame(rtr::editor::EditorFrameData{
                    .frame_serial = frame_serial,
                    .delta_seconds = 0.0,
                    .paused = engine.paused(),
                });

                forward_pipeline->set_scene_view(
                    rtr::system::render::build_forward_scene_view(
                        *active_scene,
                        engine.world().resource_manager()
                    )
                );
                renderer->draw_frame();
                resource_manager.tick(frame_serial++);

                if (input_system->state().key_down(rtr::system::input::KeyCode::ESCAPE)) {
                    renderer->window().close();
                }
            },
            .should_close = [&]() { return renderer->window().is_should_close(); }
        });

        engine.run();
        renderer->device().wait_idle();
        resource_manager.flush_after_wait_idle();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
