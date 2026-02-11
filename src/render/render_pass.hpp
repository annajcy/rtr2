#pragma once


#include <string_view>
#include <vector>

#include "render/frame_context.hpp"

namespace rtr::render {

enum class ResourceAccess {
    eRead,
    eWrite,
    eReadWrite
};

struct ResourceDependency {
    std::string resource_name;
    ResourceAccess access{ResourceAccess::eRead};
};

class IRenderPass {
public:
    virtual ~IRenderPass() = default;
    virtual std::string_view name() const = 0;
    virtual const std::vector<ResourceDependency>& dependencies() const = 0;
    virtual void execute(render::FrameContext& ctx) = 0;
};



} // namespace rtr::render
