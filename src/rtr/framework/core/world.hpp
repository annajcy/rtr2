#pragma once

#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/framework/core/types.hpp"
#include "rtr/resource/resource_fwd.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::core {

class World {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("framework.core.world");
    }

    SceneId m_next_scene_id{1};
    SceneId m_active_scene_id{core::kInvalidSceneId};
    std::vector<std::unique_ptr<Scene>> m_scenes{};
    std::unordered_map<SceneId, Scene*> m_scene_by_id{};
    std::unordered_map<SceneId, std::string> m_scene_name_by_id{};
    std::unordered_map<std::string, SceneId> m_scene_id_by_name{};
    resource::ResourceManager& m_resource_manager;

    bool is_name_occupied(std::string_view name, SceneId ignore_id = core::kInvalidSceneId) const {
        const auto it = m_scene_id_by_name.find(std::string(name));
        if (it == m_scene_id_by_name.end()) {
            return false;
        }
        return it->second != ignore_id;
    }

    std::string make_unique_scene_name(std::string_view requested_name,
                                       SceneId ignore_id = core::kInvalidSceneId) const {
        std::string base_name = requested_name.empty() ? "Scene" : std::string(requested_name);
        if (!is_name_occupied(base_name, ignore_id)) {
            return base_name;
        }
        for (std::size_t suffix = 1;; ++suffix) {
            std::string candidate = base_name + "_" + std::to_string(suffix);
            if (!is_name_occupied(candidate, ignore_id)) {
                return candidate;
            }
        }
    }

public:
    explicit World(resource::ResourceManager& resource_manager)
        : m_resource_manager(resource_manager) {}

    World(const World&) = delete;
    World& operator=(const World&) = delete;
    World(World&&) = delete;
    World& operator=(World&&) = delete;

    Scene& create_scene(std::string name = "Scene") {
        std::string unique_name = make_unique_scene_name(name);
        auto log = logger();
        auto scene = std::make_unique<Scene>(m_next_scene_id++);
        Scene* ptr = scene.get();
        const SceneId id = ptr->id();
        m_scenes.emplace_back(std::move(scene));
        m_scene_by_id.emplace(id, ptr);
        m_scene_name_by_id.emplace(id, unique_name);
        m_scene_id_by_name.emplace(unique_name, id);
        if (m_active_scene_id == core::kInvalidSceneId) {
            m_active_scene_id = id;
        }
        log->info(
            "Scene created (scene_id={}, name='{}', scene_count={}, active_scene_id={})",
            id,
            unique_name,
            m_scenes.size(),
            m_active_scene_id
        );
        return *ptr;
    }

    Scene* find_scene(SceneId id) {
        const auto it = m_scene_by_id.find(id);
        return (it == m_scene_by_id.end()) ? nullptr : it->second;
    }

    const Scene* find_scene(SceneId id) const {
        const auto it = m_scene_by_id.find(id);
        return (it == m_scene_by_id.end()) ? nullptr : it->second;
    }

    Scene* find_scene(std::string_view name) {
        const auto id_it = m_scene_id_by_name.find(std::string(name));
        if (id_it == m_scene_id_by_name.end()) {
            return nullptr;
        }
        return find_scene(id_it->second);
    }

    const Scene* find_scene(std::string_view name) const {
        const auto id_it = m_scene_id_by_name.find(std::string(name));
        if (id_it == m_scene_id_by_name.end()) {
            return nullptr;
        }
        return find_scene(id_it->second);
    }

    bool has_scene(SceneId id) const {
        return find_scene(id) != nullptr;
    }

    bool has_scene(std::string_view name) const {
        return find_scene(name) != nullptr;
    }

    std::optional<std::string_view> scene_name(SceneId id) const noexcept {
        const auto it = m_scene_name_by_id.find(id);
        if (it == m_scene_name_by_id.end()) {
            return std::nullopt;
        }
        return std::string_view{it->second};
    }

    bool rename_scene(SceneId id, std::string new_name) {
        if (find_scene(id) == nullptr) {
            logger()->warn("rename_scene ignored: scene {} does not exist.", id);
            return false;
        }
        const auto name_it = m_scene_name_by_id.find(id);
        if (name_it == m_scene_name_by_id.end()) {
            logger()->error("rename_scene failed: missing name entry for scene {}.", id);
            return false;
        }
        const std::string old_name = name_it->second;
        const std::string unique_name = make_unique_scene_name(new_name, id);
        if (old_name == unique_name) {
            return true;
        }

        m_scene_id_by_name.erase(old_name);
        name_it->second = unique_name;
        m_scene_id_by_name[unique_name] = id;
        logger()->info("Scene renamed (scene_id={}, old_name='{}', new_name='{}').", id, old_name, unique_name);
        return true;
    }

    bool set_active_scene(SceneId id) {
        if (find_scene(id) == nullptr) {
            logger()->warn("set_active_scene failed: scene {} does not exist.", id);
            return false;
        }
        m_active_scene_id = id;
        logger()->info("Active scene changed to {} ('{}').", id, scene_name(id).value_or(""));
        return true;
    }

    bool set_active_scene(std::string_view name) {
        const Scene* scene = find_scene(name);
        if (scene == nullptr) {
            logger()->warn("set_active_scene failed: scene '{}' does not exist.", name);
            return false;
        }
        return set_active_scene(scene->id());
    }

    bool destroy_scene(SceneId id) {
        if (find_scene(id) == nullptr) {
            logger()->warn("destroy_scene ignored: scene {} does not exist.", id);
            return false;
        }
        if (id == m_active_scene_id) {
            logger()->warn("destroy_scene ignored: scene {} is currently active.", id);
            return false;
        }

        const auto it = std::find_if(m_scenes.begin(), m_scenes.end(), [id](const std::unique_ptr<Scene>& scene) {
            return scene && scene->id() == id;
        });
        if (it == m_scenes.end()) {
            logger()->error("destroy_scene failed: scene {} missing in owned scene list.", id);
            return false;
        }

        std::string removed_name{};
        if (const auto name_it = m_scene_name_by_id.find(id); name_it != m_scene_name_by_id.end()) {
            removed_name = name_it->second;
            m_scene_id_by_name.erase(removed_name);
            m_scene_name_by_id.erase(name_it);
        }
        m_scene_by_id.erase(id);
        m_scenes.erase(it);

        logger()->info("Scene destroyed (scene_id={}, name='{}', scene_count={}, active_scene_id={})", id, removed_name,
                       m_scenes.size(), m_active_scene_id);
        return true;
    }

    SceneId active_scene_id() const {
        return m_active_scene_id;
    }

    Scene* active_scene() {
        return find_scene(m_active_scene_id);
    }

    const Scene* active_scene() const {
        return find_scene(m_active_scene_id);
    }

    const std::vector<std::unique_ptr<Scene>>& scenes() const {
        return m_scenes;
    }

    std::size_t scene_count() const {
        return m_scenes.size();
    }

    resource::ResourceManager& resource_manager() {
        return m_resource_manager;
    }

    const resource::ResourceManager& resource_manager() const {
        return m_resource_manager;
    }

    void fixed_tick(const FixedTickContext& ctx) {
        Scene* scene = active_scene();
        if (scene != nullptr) {
            scene->fixed_tick(ctx);
        }
    }

    void tick(const FrameTickContext& ctx) {
        Scene* scene = active_scene();
        if (scene != nullptr) {
            scene->tick(ctx);
        }
    }

    void late_tick(const FrameTickContext& ctx) {
        Scene* scene = active_scene();
        if (scene != nullptr) {
            scene->late_tick(ctx);
        }
    }
};

} // namespace rtr::framework::core
