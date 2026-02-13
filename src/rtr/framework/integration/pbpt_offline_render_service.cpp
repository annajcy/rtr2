#include "rtr/framework/integration/pbpt_offline_render_service.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <utility>

#include "pbpt/integrator/integrator.hpp"
#include "pbpt/integrator/plugin/integrator/path_integrator.hpp"
#include "pbpt/loader/scene_loader.hpp"

#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/pbpt_scene_export_builder.hpp"

namespace rtr::framework::integration {

PbptOfflineRenderService::PbptOfflineRenderService(RenderBackend backend)
    : m_backend(std::move(backend)) {
    if (!m_backend) {
        m_backend = [](const OfflineRenderConfig& config,
                       const ProgressCallback& on_progress,
                       const CancelQuery& is_cancel_requested) {
            auto pbpt_scene = pbpt::loader::load_scene<float>(config.scene_xml_path);
            pbpt::integrator::PathIntegrator<float, 4> integrator(-1, 0.9f);

            pbpt::integrator::RenderObserver observer{};
            observer.on_progress = on_progress;
            observer.is_cancel_requested = is_cancel_requested;
            try {
                integrator.render(pbpt_scene, config.spp, config.output_exr_path, false, observer);
            } catch (const pbpt::integrator::RenderCanceled& e) {
                throw RenderCanceled(e.what());
            }
        };
    }

    set_message("Idle.");
}

PbptOfflineRenderService::~PbptOfflineRenderService() {
    request_cancel();
    if (m_worker.joinable()) {
        m_worker.join();
    }
}

bool PbptOfflineRenderService::start(const core::Scene& scene, const OfflineRenderConfig& config) {
    std::scoped_lock lifecycle_lock(m_lifecycle_mutex);

    if (is_running()) {
        set_message("Render already running.");
        return false;
    }

    if (config.scene_xml_path.empty()) {
        m_state.store(OfflineRenderState::Failed);
        set_message("scene_xml_path must not be empty.");
        return false;
    }
    if (config.output_exr_path.empty()) {
        m_state.store(OfflineRenderState::Failed);
        set_message("output_exr_path must not be empty.");
        return false;
    }
    if (config.spp < 1) {
        m_state.store(OfflineRenderState::Failed);
        set_message("spp must be >= 1.");
        return false;
    }
    if ((config.film_width > 0 && config.film_height <= 0) ||
        (config.film_height > 0 && config.film_width <= 0)) {
        m_state.store(OfflineRenderState::Failed);
        set_message("film_width and film_height must both be > 0 when overriding film size.");
        return false;
    }

    if (m_worker.joinable()) {
        m_worker.join();
    }

    try {
        if (scene.active_camera() == nullptr) {
            throw std::runtime_error("Offline render requires an active camera.");
        }

        auto record = build_pbpt_scene_record(scene);
        if (!record.sensor.has_value()) {
            throw std::runtime_error("Failed to export PBPT sensor from current active camera.");
        }
        if (record.shapes.empty()) {
            throw std::runtime_error("Current scene has no exportable PBPT shapes.");
        }
        const bool has_emitter = std::any_of(
            record.shapes.begin(),
            record.shapes.end(),
            [](const auto& shape) { return shape.has_area_emitter; }
        );
        if (!has_emitter) {
            throw std::runtime_error("Current scene has no PBPT area emitter; output would be black.");
        }
        if (record.sensor.has_value()) {
            record.sensor->sample_count = config.spp;
            if (config.film_width > 0 && config.film_height > 0) {
                record.sensor->film_width = config.film_width;
                record.sensor->film_height = config.film_height;
            }
        }
        const std::string scene_xml = serialize_pbpt_scene_xml(record);

        std::filesystem::path scene_xml_path(config.scene_xml_path);
        if (scene_xml_path.has_parent_path()) {
            std::filesystem::create_directories(scene_xml_path.parent_path());
        }
        std::filesystem::path output_exr_path(config.output_exr_path);
        if (output_exr_path.has_parent_path()) {
            std::filesystem::create_directories(output_exr_path.parent_path());
        }

        std::ofstream out(scene_xml_path);
        if (!out) {
            throw std::runtime_error(
                "Failed to open scene XML path for writing: " + config.scene_xml_path
            );
        }
        out << scene_xml;
        if (!out.good()) {
            throw std::runtime_error("Failed to write scene XML to: " + config.scene_xml_path);
        }
    } catch (const std::exception& e) {
        m_state.store(OfflineRenderState::Failed);
        set_message(e.what());
        return false;
    }

    m_cancel_requested.store(false);
    m_progress_01.store(0.0f);
    m_state.store(OfflineRenderState::Running);
    set_message("Scene snapshot saved to XML: " + config.scene_xml_path);

    const OfflineRenderConfig worker_config = config;
    m_worker = std::thread([this, worker_config]() {
        run_worker(worker_config);
    });

    return true;
}

void PbptOfflineRenderService::request_cancel() {
    m_cancel_requested.store(true);
    if (is_running()) {
        set_message("Cancel requested.");
    }
}

OfflineRenderState PbptOfflineRenderService::state() const {
    return m_state.load();
}

float PbptOfflineRenderService::progress_01() const {
    return m_progress_01.load();
}

const std::string& PbptOfflineRenderService::last_message() const {
    thread_local std::shared_ptr<const std::string> snapshot;
    std::scoped_lock lock(m_message_mutex);
    snapshot = m_last_message;
    return *snapshot;
}

bool PbptOfflineRenderService::is_running() const {
    return state() == OfflineRenderState::Running;
}

void PbptOfflineRenderService::run_worker(const OfflineRenderConfig& config) {
    try {
        m_backend(
            config,
            [this](float progress) {
                m_progress_01.store(std::clamp(progress, 0.0f, 1.0f));
            },
            [this]() {
                return m_cancel_requested.load();
            }
        );

        m_progress_01.store(1.0f);
        m_state.store(OfflineRenderState::Succeeded);
        set_message("Render succeeded: " + config.output_exr_path);
    } catch (const RenderCanceled&) {
        m_state.store(OfflineRenderState::Canceled);
        set_message("Render canceled.");
    } catch (const std::exception& e) {
        m_state.store(OfflineRenderState::Failed);
        set_message(e.what());
    }
}

void PbptOfflineRenderService::set_message(std::string message) {
    std::scoped_lock lock(m_message_mutex);
    m_last_message = std::make_shared<const std::string>(std::move(message));
}

} // namespace rtr::framework::integration
