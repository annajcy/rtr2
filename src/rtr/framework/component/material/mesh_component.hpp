#pragma once

#include <pbpt/math/math.h>

#include <vector>

#include "rtr/framework/component/component.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view.hpp"

namespace rtr::framework::component {

class MeshComponent : public Component {
public:
    MeshComponent(core::GameObject& owner, pbpt::math::Vec4 base_color)
        : Component(owner), m_base_color(base_color) {}

    virtual ~MeshComponent() = default;

    const pbpt::math::Vec4& base_color() const {
        return m_base_color;
    }

    void set_base_color(const pbpt::math::Vec4& base_color) {
        m_base_color = base_color;
    }

    virtual std::vector<pbpt::math::Vec3> local_vertices() const = 0;
    
    virtual bool has_valid_mesh() const = 0;
    
    virtual system::render::MeshView mesh_view(rhi::Device& device) = 0;

private:
    pbpt::math::Vec4 m_base_color{1.0f, 1.0f, 1.0f, 1.0f};
};

}  // namespace rtr::framework::component
