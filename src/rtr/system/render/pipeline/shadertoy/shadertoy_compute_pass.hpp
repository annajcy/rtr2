#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string_view>
#include <vector>

#include "rtr/rhi/buffer.hpp"
#include "rtr/rhi/descriptor.hpp"
#include "rtr/rhi/texture.hpp"
#include "rtr/system/render/frame_context.hpp"
#include "rtr/system/render/render_pass.hpp"
#include "vulkan/vulkan.hpp"

namespace rtr::system::render {

struct ShaderToyUniformBufferObject {
    alignas(16) std::array<float, 4> i_resolution{};
    alignas(16) std::array<float, 4> i_time{};
};

class ComputePass final : public IRenderPass {
public:
    struct RenderPassResources {
        rhi::Buffer*             uniform_buffer{};
        rhi::Image*              offscreen_image{};
        vk::ImageLayout*         offscreen_layout{};
        vk::raii::DescriptorSet* compute_set{};
    };

private:
    vk::raii::PipelineLayout*             m_pipeline_layout{};
    vk::raii::Pipeline*                   m_compute_pipeline{};
    RenderPassResources                   m_render_pass_resources{};
    std::chrono::steady_clock::time_point m_start_time = std::chrono::steady_clock::now();
    std::vector<ResourceDependency>       m_dependencies{{"shadertoy.uniform", ResourceAccess::eRead},
                                                         {"shadertoy.compute", ResourceAccess::eRead},
                                                         {"shadertoy.offscreen", ResourceAccess::eReadWrite}};

public:
    ComputePass(vk::raii::PipelineLayout* pipeline_layout, vk::raii::Pipeline* compute_pipeline)
        : m_pipeline_layout(pipeline_layout), m_compute_pipeline(compute_pipeline) {}

    std::string_view name() const override { return "shadertoy.compute"; }

    const std::vector<ResourceDependency>& dependencies() const override { return m_dependencies; }

    void bind_render_pass_resources(const RenderPassResources& resources) {
        if (resources.uniform_buffer == nullptr || resources.offscreen_image == nullptr ||
            resources.offscreen_layout == nullptr || resources.compute_set == nullptr) {
            throw std::runtime_error("ShaderToyComputePass frame resources are incomplete.");
        }
        m_render_pass_resources = resources;
    }

    void execute(render::FrameContext& ctx) override {
        if (m_render_pass_resources.uniform_buffer == nullptr || m_render_pass_resources.offscreen_image == nullptr ||
            m_render_pass_resources.offscreen_layout == nullptr || m_render_pass_resources.compute_set == nullptr) {
            throw std::runtime_error("ShaderToyComputePass frame resources are not bound.");
        }

        const vk::Extent2D offscreen_extent{m_render_pass_resources.offscreen_image->width(),
                                            m_render_pass_resources.offscreen_image->height()};
        update_uniform_buffer(*m_render_pass_resources.uniform_buffer, offscreen_extent);

        auto&                 cmd        = ctx.cmd().command_buffer();
        rhi::Image&           offscreen  = *m_render_pass_resources.offscreen_image;
        const vk::ImageLayout old_layout = *m_render_pass_resources.offscreen_layout;

        vk::PipelineStageFlags2 src_stage  = vk::PipelineStageFlagBits2::eTopOfPipe;
        vk::AccessFlags2        src_access = vk::AccessFlagBits2::eNone;
        if (old_layout == vk::ImageLayout::eShaderReadOnlyOptimal) {
            src_stage  = vk::PipelineStageFlagBits2::eFragmentShader;
            src_access = vk::AccessFlagBits2::eShaderSampledRead;
        } else if (old_layout == vk::ImageLayout::eGeneral) {
            src_stage  = vk::PipelineStageFlagBits2::eComputeShader;
            src_access = vk::AccessFlagBits2::eShaderStorageWrite;
        }

        vk::ImageMemoryBarrier2 to_general{};
        to_general.srcStageMask                    = src_stage;
        to_general.dstStageMask                    = vk::PipelineStageFlagBits2::eComputeShader;
        to_general.srcAccessMask                   = src_access;
        to_general.dstAccessMask                   = vk::AccessFlagBits2::eShaderStorageWrite;
        to_general.oldLayout                       = old_layout;
        to_general.newLayout                       = vk::ImageLayout::eGeneral;
        to_general.image                           = *offscreen.image();
        to_general.subresourceRange.aspectMask     = vk::ImageAspectFlagBits::eColor;
        to_general.subresourceRange.baseMipLevel   = 0;
        to_general.subresourceRange.levelCount     = 1;
        to_general.subresourceRange.baseArrayLayer = 0;
        to_general.subresourceRange.layerCount     = 1;

        vk::DependencyInfo to_general_dep{};
        to_general_dep.imageMemoryBarrierCount = 1;
        to_general_dep.pImageMemoryBarriers    = &to_general;
        cmd.pipelineBarrier2(to_general_dep);

        *m_render_pass_resources.offscreen_layout = vk::ImageLayout::eGeneral;

        cmd.bindPipeline(vk::PipelineBindPoint::eCompute, **m_compute_pipeline);
        cmd.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **m_pipeline_layout, 0,
                               **m_render_pass_resources.compute_set, {});

        const vk::Extent2D extent{m_render_pass_resources.offscreen_image->width(),
                                  m_render_pass_resources.offscreen_image->height()};
        const uint32_t     group_count_x = (extent.width + 7) / 8;
        const uint32_t     group_count_y = (extent.height + 7) / 8;
        cmd.dispatch(group_count_x, group_count_y, 1);
    }

private:
    void update_uniform_buffer(rhi::Buffer& uniform_buffer, const vk::Extent2D& extent) {
        const auto  now     = std::chrono::steady_clock::now();
        const float seconds = std::chrono::duration<float>(now - m_start_time).count();

        ShaderToyUniformBufferObject ubo{};
        ubo.i_resolution = {static_cast<float>(extent.width), static_cast<float>(extent.height), 1.0f, 0.0f};
        ubo.i_time       = {seconds, 0.0f, 0.0f, 0.0f};

        std::memcpy(uniform_buffer.mapped_data(), &ubo, sizeof(ubo));
    }
};

}  // namespace rtr::system::render
