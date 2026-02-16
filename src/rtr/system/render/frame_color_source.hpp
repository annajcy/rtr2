#pragma once

#include "vulkan/vulkan.hpp"

#include <cstdint>

namespace rtr::system::render {

struct FrameColorSourceView {
    vk::ImageView image_view{};
    vk::ImageLayout image_layout{vk::ImageLayout::eUndefined};
    vk::Extent2D extent{};

    bool valid() const {
        return image_view != vk::ImageView{} &&
            image_layout != vk::ImageLayout::eUndefined &&
            extent.width > 0 &&
            extent.height > 0;
    }
};

class IFrameColorSource {
public:
    virtual ~IFrameColorSource() = default;
    virtual FrameColorSourceView frame_color_source_view(uint32_t frame_index) const = 0;
};

class ISceneViewportSink {
public:
    virtual ~ISceneViewportSink() = default;
    virtual void set_scene_viewport_extent(vk::Extent2D extent) = 0;
};

} // namespace rtr::system::render
