#pragma once

#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "rtr/rhi/texture.hpp"
#include "vulkan/vulkan.hpp"

namespace rtr::system::render {

struct TrackedImage {
    rhi::Image& image;
    vk::ImageLayout& layout;
};

struct FrameTrackedImage {
    rhi::Image image;
    vk::ImageLayout layout{vk::ImageLayout::eUndefined};

    explicit FrameTrackedImage(rhi::Image&& image_in, vk::ImageLayout layout_in = vk::ImageLayout::eUndefined)
        : image(std::move(image_in)),
          layout(layout_in) {}

    TrackedImage view() {
        return TrackedImage{
            .image = image,
            .layout = layout
        };
    }
};

inline void require_valid_tracked_image(const TrackedImage& tracked, std::string_view message) {
    if (tracked.image.width() == 0 || tracked.image.height() == 0) {
        throw std::runtime_error(std::string(message));
    }
}

}  // namespace rtr::system::render
