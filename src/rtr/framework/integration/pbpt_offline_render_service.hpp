#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

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

    explicit PbptOfflineRenderService(RenderBackend backend = RenderBackend{});
    PbptOfflineRenderService(const PbptOfflineRenderService&) = delete;
    PbptOfflineRenderService& operator=(const PbptOfflineRenderService&) = delete;
    ~PbptOfflineRenderService();

    bool start(const core::Scene& scene, const OfflineRenderConfig& config);
    void request_cancel();
    OfflineRenderState state() const;
    float progress_01() const;
    const std::string& last_message() const;
    bool is_running() const;

private:
    void run_worker(const OfflineRenderConfig& config);
    void set_message(std::string message);

private:
    std::atomic<OfflineRenderState> m_state{OfflineRenderState::Idle};
    std::atomic<float> m_progress_01{0.0f};
    std::atomic<bool> m_cancel_requested{false};

    mutable std::mutex m_message_mutex{};
    std::shared_ptr<const std::string> m_last_message{};

    mutable std::mutex m_lifecycle_mutex{};
    OfflineRenderWorkerThread m_worker{};

    RenderBackend m_backend{};
};

} // namespace rtr::framework::integration
