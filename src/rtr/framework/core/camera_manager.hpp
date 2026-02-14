#pragma once

#include <algorithm>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "rtr/framework/core/camera.hpp"
#include "rtr/framework/core/scene_graph.hpp"
#include "rtr/framework/core/types.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::core {

class CameraManager {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("framework.core.camera_manager");
    }

    SceneGraph* m_scene_graph{nullptr};
    std::unordered_map<GameObjectId, std::unique_ptr<CameraBase>> m_cameras{};
    std::vector<GameObjectId> m_camera_order{};
    GameObjectId m_active_owner_id{core::kInvalidGameObjectId};

    void ensure_valid_owner(GameObjectId owner_id) const {
        if (m_scene_graph == nullptr) {
            logger()->error("CameraManager ensure_valid_owner failed: SceneGraph is null.");
            throw std::runtime_error("CameraManager is not bound to a SceneGraph.");
        }
        if (owner_id == core::kInvalidGameObjectId || !m_scene_graph->has_node(owner_id)) {
            logger()->error("CameraManager ensure_valid_owner failed: invalid owner id {}.", owner_id);
            throw std::runtime_error("Camera owner id is invalid or does not exist in scene graph.");
        }
    }

    template <typename TCamera>
    TCamera& create_camera_internal(GameObjectId owner_id) {
        ensure_valid_owner(owner_id);
        if (has_camera(owner_id)) {
            logger()->warn(
                "create_camera rejected: owner {} already has a camera.",
                owner_id
            );
            throw std::runtime_error("GameObject already has a camera.");
        }

        auto camera = std::make_unique<TCamera>(owner_id, m_scene_graph);
        TCamera* ptr = camera.get();
        m_cameras.emplace(owner_id, std::move(camera));
        m_camera_order.emplace_back(owner_id);

        if (m_active_owner_id == core::kInvalidGameObjectId) {
            m_active_owner_id = owner_id;
        }

        logger()->info(
            "Camera created (owner_id={}, camera_count={}, active_owner_id={})",
            owner_id,
            m_cameras.size(),
            m_active_owner_id
        );

        return *ptr;
    }

public:
    explicit CameraManager(SceneGraph* scene_graph = nullptr)
        : m_scene_graph(scene_graph) {}

    CameraManager(const CameraManager&) = delete;
    CameraManager& operator=(const CameraManager&) = delete;
    CameraManager(CameraManager&&) noexcept = default;
    CameraManager& operator=(CameraManager&&) noexcept = default;

    void bind_scene_graph(SceneGraph* scene_graph) {
        m_scene_graph = scene_graph;
        logger()->info("CameraManager bound SceneGraph (bound={}).", m_scene_graph != nullptr);
        for (auto& [owner_id, camera] : m_cameras) {
            if (camera != nullptr) {
                camera->bind(owner_id, m_scene_graph);
            }
        }
    }

    PerspectiveCamera& create_perspective_camera(GameObjectId owner_id) {
        return create_camera_internal<PerspectiveCamera>(owner_id);
    }

    OrthographicCamera& create_orthographic_camera(GameObjectId owner_id) {
        return create_camera_internal<OrthographicCamera>(owner_id);
    }

    bool destroy_camera(GameObjectId owner_id) {
        const auto it = m_cameras.find(owner_id);
        if (it == m_cameras.end()) {
            logger()->warn("destroy_camera ignored: owner {} has no camera.", owner_id);
            return false;
        }

        const auto order_it = std::find(m_camera_order.begin(), m_camera_order.end(), owner_id);
        if (order_it == m_camera_order.end()) {
            logger()->error("Camera order/storage mismatch while destroying owner {}.", owner_id);
            throw std::runtime_error("Camera order and storage are out of sync.");
        }

        const std::size_t removed_index = static_cast<std::size_t>(order_it - m_camera_order.begin());
        const bool was_active = (owner_id == m_active_owner_id);

        m_cameras.erase(it);
        m_camera_order.erase(order_it);

        if (m_camera_order.empty()) {
            m_active_owner_id = core::kInvalidGameObjectId;
            return true;
        }

        if (was_active) {
            const std::size_t next_index = removed_index % m_camera_order.size();
            m_active_owner_id = m_camera_order[next_index];
        }

        logger()->info(
            "Camera destroyed (owner_id={}, camera_count={}, active_owner_id={})",
            owner_id,
            m_cameras.size(),
            m_active_owner_id
        );

        return true;
    }

    CameraBase* camera(GameObjectId owner_id) {
        const auto it = m_cameras.find(owner_id);
        return (it == m_cameras.end()) ? nullptr : it->second.get();
    }

    const CameraBase* camera(GameObjectId owner_id) const {
        const auto it = m_cameras.find(owner_id);
        return (it == m_cameras.end()) ? nullptr : it->second.get();
    }

    bool has_camera(GameObjectId owner_id) const {
        return m_cameras.contains(owner_id);
    }

    std::size_t camera_count() const {
        return m_cameras.size();
    }

    CameraBase* active_camera() {
        if (m_active_owner_id == core::kInvalidGameObjectId) {
            return nullptr;
        }
        return camera(m_active_owner_id);
    }

    const CameraBase* active_camera() const {
        if (m_active_owner_id == core::kInvalidGameObjectId) {
            return nullptr;
        }
        return camera(m_active_owner_id);
    }

    GameObjectId active_camera_owner_id() const {
        return m_active_owner_id;
    }

    bool set_active_camera(GameObjectId owner_id) {
        if (!has_camera(owner_id)) {
            logger()->warn("set_active_camera failed: owner {} has no camera.", owner_id);
            return false;
        }
        m_active_owner_id = owner_id;
        logger()->info("Active camera owner changed to {}.", owner_id);
        return true;
    }

    void on_game_objects_destroyed(const std::vector<GameObjectId>& ids) {
        for (const auto id : ids) {
            (void)destroy_camera(id);
        }
    }
};

} // namespace rtr::framework::core
