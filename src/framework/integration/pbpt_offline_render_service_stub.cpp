#include "framework/integration/pbpt_offline_render_service.hpp"

#include <utility>

#include "framework/core/scene.hpp"

namespace rtr::framework::integration {

PbptOfflineRenderService::PbptOfflineRenderService(RenderBackend backend)
    : m_backend(std::move(backend)) {
    set_message("PBPT runtime is disabled in this package build.");
}

PbptOfflineRenderService::~PbptOfflineRenderService() {
    request_cancel();
    if (m_worker.joinable()) {
        m_worker.join();
    }
}

bool PbptOfflineRenderService::start(const core::Scene&, const OfflineRenderConfig&) {
    m_state.store(OfflineRenderState::Failed);
    set_message("PBPT runtime is disabled in this package build.");
    return false;
}

void PbptOfflineRenderService::request_cancel() {
    m_cancel_requested.store(true);
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

void PbptOfflineRenderService::run_worker(const OfflineRenderConfig&) {
    m_state.store(OfflineRenderState::Failed);
    set_message("PBPT runtime is disabled in this package build.");
}

void PbptOfflineRenderService::set_message(std::string message) {
    std::scoped_lock lock(m_message_mutex);
    m_last_message = std::make_shared<const std::string>(std::move(message));
}

} // namespace rtr::framework::integration
