#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "rtr/system/render/render_pipeline.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::system::render {

template <typename TTargets>
class SceneTargetController {
private:
    struct RetiredTargets {
        TTargets targets;
        std::uint32_t pending_frame_mask{0};
        std::uint64_t generation{0};
    };

    RenderPipeline& m_owner;
    std::string m_debug_name;
    std::optional<TTargets> m_targets{};
    std::vector<RetiredTargets> m_retired_targets{};

    vk::Extent2D m_scene_extent{};
    vk::Extent2D m_requested_scene_extent{};
    bool m_scene_extent_dirty{false};
    std::uint64_t m_active_generation{0};
    bool m_recreated_this_frame{false};
    utils::SubscriptionToken m_scene_viewport_resize_subscription{};

    static bool is_valid_extent(const vk::Extent2D& extent) {
        return extent.width > 0 && extent.height > 0;
    }

    static std::uint32_t all_frames_mask() {
        static_assert(rhi::kFramesInFlight > 0, "kFramesInFlight must be greater than 0.");
        static_assert(rhi::kFramesInFlight <= 32, "SceneTargetController frame mask supports up to 32 frames in flight.");
        if constexpr (rhi::kFramesInFlight == 32) {
            return 0xFFFF'FFFFu;
        } else {
            return (1u << rhi::kFramesInFlight) - 1u;
        }
    }

    void validate_frame_index(std::uint32_t frame_index) const {
        if (frame_index >= rhi::kFramesInFlight) {
            throw std::out_of_range(m_debug_name + " frame index out of range.");
        }
    }

    void collect_retired_targets(std::uint32_t frame_index) {
        const std::uint32_t frame_bit = (1u << frame_index);
        std::size_t released_count = 0;
        for (auto& retired : m_retired_targets) {
            retired.pending_frame_mask &= ~frame_bit;
            if (retired.pending_frame_mask == 0) {
                ++released_count;
            }
        }
        m_retired_targets.erase(
            std::remove_if(
                m_retired_targets.begin(),
                m_retired_targets.end(),
                [](const RetiredTargets& retired) { return retired.pending_frame_mask == 0; }
            ),
            m_retired_targets.end()
        );

        if (released_count > 0) {
            utils::get_logger("system.render.scene_target_controller")
                ->debug("[{}] collect_retired: released={}, remaining={}",
                         m_debug_name, released_count, m_retired_targets.size());
        }
    }

    void enqueue_retired_targets(std::uint32_t frame_index, TTargets&& targets) {
        std::uint32_t pending_frame_mask = all_frames_mask();
        pending_frame_mask &= ~(1u << frame_index);
        if (pending_frame_mask == 0) {
            utils::get_logger("system.render.scene_target_controller")
                ->debug("[{}] enqueue_retired: dropped immediately (all frame slots already safe).", m_debug_name);
            return;
        }

        m_retired_targets.push_back(RetiredTargets{
            .targets = std::move(targets),
            .pending_frame_mask = pending_frame_mask,
            .generation = m_active_generation
        });
        utils::get_logger("system.render.scene_target_controller")
            ->debug("[{}] enqueue_retired: pending_mask=0x{:x}, retired_count={}",
                     m_debug_name, pending_frame_mask, m_retired_targets.size());
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
    TTargets& ensure(std::uint32_t frame_index, vk::Extent2D fallback_extent, CreateFn&& create_fn, PostFn&& post_fn) {
        validate_frame_index(frame_index);
        if (!is_valid_extent(fallback_extent)) {
            throw std::runtime_error(m_debug_name + " fallback extent is invalid.");
        }
        collect_retired_targets(frame_index);
        m_recreated_this_frame = false;

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

        auto new_targets = std::invoke(std::forward<CreateFn>(create_fn), desired);
        if (m_targets.has_value()) {
            enqueue_retired_targets(frame_index, std::move(*m_targets));
        }

        m_scene_extent = desired;
        m_targets = std::move(new_targets);
        ++m_active_generation;
        m_recreated_this_frame = true;
        std::invoke(std::forward<PostFn>(post_fn), *m_targets);
        m_scene_extent_dirty = false;
        utils::get_logger("system.render.scene_target_controller")
            ->debug("[{}] recreate: generation={}, extent=({}, {}), retired_count={}",
                     m_debug_name, m_active_generation, m_scene_extent.width, m_scene_extent.height, m_retired_targets.size());
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

    std::uint64_t active_generation() const {
        return m_active_generation;
    }

    bool recreated_this_frame() const {
        return m_recreated_this_frame;
    }
};

}  // namespace rtr::system::render
