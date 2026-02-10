#pragma once

#include <cstdint>
#include <string>

#include "render/resource_registries.hpp"
#include "rhi/command.hpp"
#include "rhi/device.hpp"
#include "rhi/texture.hpp"

namespace rtr::render {

class FrameContext {
private:
    rhi::Device* m_device{};
    rhi::CommandBuffer* m_cmd{};
    ResourceRegistries* m_registries{};
    const vk::raii::ImageView* m_swapchain_image_view{};
    const vk::Image* m_swapchain_image{};
    vk::Extent2D m_render_extent{};
    uint32_t m_frame_index{0};

public:
    FrameContext(
        rhi::Device* device,
        rhi::CommandBuffer* cmd,
        ResourceRegistries* registries,
        const vk::raii::ImageView& swapchain_image_view,
        const vk::Image& swapchain_image,
        const vk::Extent2D& render_extent,
        uint32_t frame_index
    )
        : m_device(device),
          m_cmd(cmd),
          m_registries(registries),
          m_swapchain_image_view(&swapchain_image_view),
          m_swapchain_image(&swapchain_image),
          m_render_extent(render_extent),
          m_frame_index(frame_index) {}

    const rhi::CommandBuffer& cmd() const { return *m_cmd; }
    rhi::CommandBuffer& cmd() { return *m_cmd; }

    const vk::raii::ImageView& swapchain_image_view() const { return *m_swapchain_image_view; }
    const vk::Image& swapchain_image() const { return *m_swapchain_image; }
    uint32_t frame_index() const { return m_frame_index; }
    const rhi::Image& depth_image() const {
        return m_registries->registry<rhi::Image>().get_perframe_resource(
            m_frame_index,
            kBuiltinDepthImageResourceName
        );
    }
    // Only inject attachments during binding stage; avoid mutating resources inside pass execution.
    void set_perframe_image(const std::string& name, rhi::Image& image) {
        m_registries->registry<rhi::Image>().set_frame_resource(m_frame_index, name, image);
    }
    const vk::Extent2D& render_extent() const { return m_render_extent; }

    rhi::Buffer& get_perframe_buffer(const std::string& name) {
        return m_registries->registry<rhi::Buffer>().get_perframe_resource(m_frame_index, name);
    }

    rhi::Buffer& get_global_buffer(const std::string& name) {
        return m_registries->registry<rhi::Buffer>().get_global_resource(name);
    }

    vk::raii::DescriptorSet& get_perframe_descriptor_set(const std::string& name) {
        return m_registries->registry<vk::raii::DescriptorSet>().get_perframe_resource(m_frame_index, name);
    }

    vk::raii::DescriptorSet& get_global_descriptor_set(const std::string& name) {
        return m_registries->registry<vk::raii::DescriptorSet>().get_global_resource(name);
    }

    bool has_perframe_buffer(const std::string& name) const {
        return m_registries->registry<rhi::Buffer>().has_perframe_resource(m_frame_index, name);
    }

    bool has_global_buffer(const std::string& name) const {
        return m_registries->registry<rhi::Buffer>().has_global_resource(name);
    }

    bool has_perframe_descriptor_set(const std::string& name) const {
        return m_registries->registry<vk::raii::DescriptorSet>().has_perframe_resource(m_frame_index, name);
    }

    bool has_global_descriptor_set(const std::string& name) const {
        return m_registries->registry<vk::raii::DescriptorSet>().has_global_resource(name);
    }
};

} // namespace rtr::render
