#pragma once

#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "rtr/system/render/render_pipeline.hpp"

namespace rtr::system::render {

template <typename TTargets>
class SceneTargetController {
private:
    RenderPipeline& m_owner;
    std::string m_debug_name;
    std::optional<TTargets> m_targets{};

    vk::Extent2D m_scene_extent{};
    vk::Extent2D m_requested_scene_extent{};
    bool m_scene_extent_dirty{false};
    utils::SubscriptionToken m_scene_viewport_resize_subscription{};

    static bool is_valid_extent(const vk::Extent2D& extent) {
        return extent.width > 0 && extent.height > 0;
    }

    void on_scene_viewport_resize(const SceneViewportResizeEvent& event) {
        if (!is_valid_extent(vk::Extent2D{event.width, event.height})) {
            return;
        }
        if (m_requested_scene_extent.width == event.width && m_requested_scene_extent.height == event.height) {
            return;
        }
        m_requested_scene_extent = vk::Extent2D{event.width, event.height};
        m_scene_extent_dirty = true;
    }

public:
    SceneTargetController(RenderPipeline& owner, std::string debug_name)
        : m_owner(owner),
          m_debug_name(std::move(debug_name)) {
        m_scene_viewport_resize_subscription = m_owner.subscribe_event<SceneViewportResizeEvent>(
            [this](const SceneViewportResizeEvent& event) { on_scene_viewport_resize(event); }
        );
    }

    void on_swapchain_extent_changed() {
        m_scene_extent_dirty = true;
    }

    void request_recreate() {
        m_scene_extent_dirty = true;
    }

    template <class CreateFn, class PostFn>
    TTargets& ensure(vk::Extent2D fallback_extent, CreateFn&& create_fn, PostFn&& post_fn) {
        if (!is_valid_extent(fallback_extent)) {
            throw std::runtime_error(m_debug_name + " fallback extent is invalid.");
        }

        vk::Extent2D desired = fallback_extent;
        if (is_valid_extent(m_requested_scene_extent)) {
            desired = m_requested_scene_extent;
        }

        const bool need_recreate =
            m_scene_extent_dirty ||
            !m_targets.has_value() ||
            m_scene_extent.width != desired.width ||
            m_scene_extent.height != desired.height;

        if (!need_recreate) {
            return *m_targets;
        }

        m_owner.wait_for_scene_target_rebuild();
        m_scene_extent = desired;
        m_targets = std::invoke(std::forward<CreateFn>(create_fn), desired);
        std::invoke(std::forward<PostFn>(post_fn), *m_targets);
        m_scene_extent_dirty = false;
        return *m_targets;
    }

    TTargets& require_targets() {
        if (!m_targets.has_value()) {
            throw std::runtime_error(m_debug_name + " scene targets are not initialized.");
        }
        return *m_targets;
    }

    const TTargets& require_targets() const {
        if (!m_targets.has_value()) {
            throw std::runtime_error(m_debug_name + " scene targets are not initialized.");
        }
        return *m_targets;
    }

    vk::Extent2D scene_extent() const {
        return m_scene_extent;
    }
};

}  // namespace rtr::system::render
