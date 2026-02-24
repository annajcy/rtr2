#pragma once

#include <string_view>
#include <stdexcept>

#include "rtr/system/render/frame_context.hpp"
#include "vulkan/vulkan.hpp"

namespace rtr::system::render {

template <typename TResource>
class RenderPass {
public:
    virtual ~RenderPass() = default;

    void execute(FrameContext& ctx, const TResource& resources) {
        validate(resources);
        do_execute(ctx, resources);
    }

protected:
    virtual void validate(const TResource&) const {}
    virtual void do_execute(FrameContext& ctx, const TResource& resources) = 0;

    static void require(bool condition, std::string_view message) {
        if (!condition) {
            throw std::runtime_error(std::string(message));
        }
    }

    static void require_valid_extent(const vk::Extent2D& extent, std::string_view message) {
        require(extent.width > 0 && extent.height > 0, message);
    }
};

} // namespace rtr::system::render
