#pragma once

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "pbpt/integrator/integrator.hpp"
#include "pbpt/integrator/plugin/integrator/path_integrator.hpp"
#include "pbpt/serde/scene_loader.hpp"

#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/pbpt/pbpt_scene_export_builder.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::core {
class Scene;
}

namespace rtr::framework::integration {
using OfflineRenderWorkerThread = std::thread;

enum class OfflineRenderState {
    Idle,
    Running,
    Succeeded,
    Failed,
    Canceled
};

inline const char* to_state_label(rtr::framework::integration::OfflineRenderState state) {
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

inline bool is_render_start_allowed(rtr::framework::integration::OfflineRenderState state) {
    using rtr::framework::integration::OfflineRenderState;
    return state == OfflineRenderState::Idle ||
        state == OfflineRenderState::Succeeded ||
        state == OfflineRenderState::Failed ||
        state == OfflineRenderState::Canceled;
}

struct OfflineRenderConfig {
    std::string scene_xml_path{};
    std::string output_exr_path{};
    int spp{16};
    // Optional override. <= 0 means using exporter defaults.
    int film_width{0};
    int film_height{0};
};

class PbptOfflineRenderService {
public:
    class RenderCanceled : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

    using ProgressCallback = std::function<void(float)>;
    using CancelQuery = std::function<bool()>;
    using RenderBackend = std::function<void(
        const OfflineRenderConfig&,
        const ProgressCallback&,
        const CancelQuery&
    )>;

    static std::shared_ptr<spdlog::logger> offline_service_logger() {
        return rtr::utils::get_logger("framework.integration.pbpt.offline_service");
    }

private:
    std::atomic<OfflineRenderState> m_state{OfflineRenderState::Idle};
    std::atomic<float> m_progress_01{0.0f};
    std::atomic<bool> m_cancel_requested{false};

    mutable std::mutex m_message_mutex{};
    std::shared_ptr<const std::string> m_last_message{};

    mutable std::mutex m_lifecycle_mutex{};
    OfflineRenderWorkerThread m_worker{};

    RenderBackend m_backend{};

public:
    explicit PbptOfflineRenderService(RenderBackend backend = RenderBackend{}): m_backend(std::move(backend)) {
        if (!m_backend) {
            m_backend = [](const OfflineRenderConfig& config,
                        const ProgressCallback& on_progress,
                        const CancelQuery& is_cancel_requested) {
                auto pbpt_scene_result = pbpt::serde::load_scene<float>(config.scene_xml_path);
                auto& pbpt_scene = pbpt_scene_result.scene;
                pbpt::integrator::PathIntegrator<float, 4> integrator(-1, 0.9f);

                pbpt::integrator::RenderObserver observer{};
                observer.on_progress = on_progress;
                observer.is_cancel_requested = is_cancel_requested;
                try {
                    integrator.render(pbpt_scene, config.output_exr_path, false, observer, config.spp);
                } catch (const pbpt::integrator::RenderCanceled& e) {
                    throw RenderCanceled(e.what());
                }
            };
        }

        set_message("Idle.");
        offline_service_logger()->info("PbptOfflineRenderService initialized.");
    }

    PbptOfflineRenderService(const PbptOfflineRenderService&) = delete;
    PbptOfflineRenderService& operator=(const PbptOfflineRenderService&) = delete;

    ~PbptOfflineRenderService() {
        request_cancel();
        if (m_worker.joinable()) {
            m_worker.join();
        }
    }

    bool start(
        const core::Scene& scene,
        resource::ResourceManager& resources,
        const OfflineRenderConfig& config
    ) {
        auto log = offline_service_logger();
        std::scoped_lock lifecycle_lock(m_lifecycle_mutex);
        log->info(
            "Offline render start requested (scene_xml='{}', output_exr='{}', spp={}, film_override={}x{}).",
            config.scene_xml_path,
            config.output_exr_path,
            config.spp,
            config.film_width,
            config.film_height
        );

        if (is_running()) {
            set_message("Render already running.");
            log->warn("Offline render start rejected: render is already running.");
            return false;
        }

        if (config.scene_xml_path.empty()) {
            m_state.store(OfflineRenderState::Failed);
            set_message("scene_xml_path must not be empty.");
            log->error("Offline render start failed: scene_xml_path is empty.");
            return false;
        }
        if (config.output_exr_path.empty()) {
            m_state.store(OfflineRenderState::Failed);
            set_message("output_exr_path must not be empty.");
            log->error("Offline render start failed: output_exr_path is empty.");
            return false;
        }
        if (config.spp < 1) {
            m_state.store(OfflineRenderState::Failed);
            set_message("spp must be >= 1.");
            log->error("Offline render start failed: spp={} is invalid.", config.spp);
            return false;
        }
        if ((config.film_width > 0 && config.film_height <= 0) ||
            (config.film_height > 0 && config.film_width <= 0)) {
            m_state.store(OfflineRenderState::Failed);
            set_message("film_width and film_height must both be > 0 when overriding film size.");
            log->error(
                "Offline render start failed: invalid film override width={} height={}.",
                config.film_width,
                config.film_height
            );
            return false;
        }

        if (m_worker.joinable()) {
            m_worker.join();
        }

        try {
            if (scene.active_camera() == nullptr) {
                throw std::runtime_error("Offline render requires an active camera.");
            }

            auto record = build_pbpt_scene_record(scene, resources);
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
            const std::string scene_xml = serialize_pbpt_scene_xml(
                record,
                resources,
                config.scene_xml_path
            );

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
                log->error("Failed to open scene XML path for writing: '{}'.", config.scene_xml_path);
                throw std::runtime_error(
                    "Failed to open scene XML path for writing: " + config.scene_xml_path
                );
            }
            out << scene_xml;
            if (!out.good()) {
                log->error("Failed to write scene XML to '{}'.", config.scene_xml_path);
                throw std::runtime_error("Failed to write scene XML to: " + config.scene_xml_path);
            }
        } catch (const std::exception& e) {
            m_state.store(OfflineRenderState::Failed);
            set_message(e.what());
            log->error("Offline render scene snapshot/export failed: {}", e.what());
            return false;
        }

        m_cancel_requested.store(false);
        m_progress_01.store(0.0f);
        m_state.store(OfflineRenderState::Running);
        set_message("Scene snapshot saved to XML: " + config.scene_xml_path);
        log->info("Scene snapshot exported to '{}'.", config.scene_xml_path);

        const OfflineRenderConfig worker_config = config;
        m_worker = std::thread([this, worker_config]() {
            run_worker(worker_config);
        });
        log->info("Offline render worker thread launched.");

        return true;
    }
    void request_cancel() {
        m_cancel_requested.store(true);
        if (is_running()) {
            set_message("Cancel requested.");
            offline_service_logger()->warn("Offline render cancel requested.");
        } else {
            offline_service_logger()->info("Offline render cancel requested while not running.");
        }
    }

    OfflineRenderState state() const { return m_state.load(); }
    float progress_01() const { return m_progress_01.load(); }
    const std::string& last_message() const { return *m_last_message; }
    bool is_running() const { return m_state.load() == OfflineRenderState::Running; }

private:
    void run_worker(const OfflineRenderConfig& config) {
        auto log = offline_service_logger();
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
            log->info("Offline render succeeded (output_exr='{}').", config.output_exr_path);
        } catch (const RenderCanceled&) {
            m_state.store(OfflineRenderState::Canceled);
            set_message("Render canceled.");
            log->warn("Offline render canceled.");
        } catch (const std::exception& e) {
            m_state.store(OfflineRenderState::Failed);
            set_message(e.what());
            log->error("Offline render worker failed: {}", e.what());
        }
    }

    void set_message(std::string message) {
        std::scoped_lock lock(m_message_mutex);
        m_last_message = std::make_shared<const std::string>(std::move(message));
    }
};

} // namespace rtr::framework::integration
