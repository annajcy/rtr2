#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "rtr/system/render/renderer.hpp"
#include "rtr/rhi/buffer.hpp"
#include "rtr/rhi/descriptor.hpp"
#include "rtr/rhi/shader_module.hpp"

namespace {

class ComputeBufferAddKernel {
private:
    rtr::rhi::Device* m_device{};
    vk::DeviceSize m_buffer_bytes{};
    uint32_t m_dispatch_x{1};

    std::unique_ptr<rtr::rhi::DescriptorSetLayout> m_descriptor_set_layout{};
    std::unique_ptr<rtr::rhi::DescriptorPool> m_descriptor_pool{};
    vk::raii::DescriptorSet m_descriptor_set{nullptr};

    vk::raii::PipelineLayout m_pipeline_layout{nullptr};
    std::unique_ptr<rtr::rhi::ShaderModule> m_shader_module{};
    vk::raii::Pipeline m_compute_pipeline{nullptr};

public:
    ComputeBufferAddKernel(
        rtr::rhi::Device* device,
        const std::string& shader_path,
        uint32_t dispatch_x,
        vk::DeviceSize buffer_bytes
    )
        : m_device(device),
          m_buffer_bytes(buffer_bytes),
          m_dispatch_x(dispatch_x) {
        if (m_device == nullptr) {
            throw std::runtime_error("ComputeBufferAddKernel requires a valid device.");
        }
        create_descriptor_resources();
        create_pipeline(shader_path);
    }

    void bind_buffers(const rtr::rhi::Buffer& lhs, const rtr::rhi::Buffer& rhs, const rtr::rhi::Buffer& out) const {
        rtr::rhi::DescriptorWriter writer;
        writer
            .write_buffer(0, *lhs.buffer(), 0, m_buffer_bytes, vk::DescriptorType::eStorageBuffer)
            .write_buffer(1, *rhs.buffer(), 0, m_buffer_bytes, vk::DescriptorType::eStorageBuffer)
            .write_buffer(2, *out.buffer(), 0, m_buffer_bytes, vk::DescriptorType::eStorageBuffer);
        writer.update(m_device, *m_descriptor_set);
    }

    void record(rtr::rhi::CommandBuffer& cb) const {
        auto& cmd = cb.command_buffer();
        cmd.bindPipeline(vk::PipelineBindPoint::eCompute, *m_compute_pipeline);
        cmd.bindDescriptorSets(
            vk::PipelineBindPoint::eCompute,
            *m_pipeline_layout,
            0,
            *m_descriptor_set,
            {}
        );
        cmd.dispatch(m_dispatch_x, 1, 1);

        vk::MemoryBarrier2 memory_barrier{};
        memory_barrier.srcStageMask = vk::PipelineStageFlagBits2::eComputeShader;
        memory_barrier.srcAccessMask = vk::AccessFlagBits2::eShaderStorageWrite;
        memory_barrier.dstStageMask = vk::PipelineStageFlagBits2::eHost;
        memory_barrier.dstAccessMask = vk::AccessFlagBits2::eHostRead;

        vk::DependencyInfo dep_info{};
        dep_info.memoryBarrierCount = 1;
        dep_info.pMemoryBarriers = &memory_barrier;
        cmd.pipelineBarrier2(dep_info);
    }

private:
    void create_descriptor_resources() {
        rtr::rhi::DescriptorSetLayout::Builder layout_builder;
        layout_builder
            .add_binding(0, vk::DescriptorType::eStorageBuffer, vk::ShaderStageFlagBits::eCompute)
            .add_binding(1, vk::DescriptorType::eStorageBuffer, vk::ShaderStageFlagBits::eCompute)
            .add_binding(2, vk::DescriptorType::eStorageBuffer, vk::ShaderStageFlagBits::eCompute);
        m_descriptor_set_layout = std::make_unique<rtr::rhi::DescriptorSetLayout>(
            layout_builder.build(m_device)
        );

        rtr::rhi::DescriptorPool::Builder pool_builder;
        pool_builder
            .add_layout(*m_descriptor_set_layout, 1)
            .set_flags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
        m_descriptor_pool = std::make_unique<rtr::rhi::DescriptorPool>(pool_builder.build(m_device));
        m_descriptor_set = m_descriptor_pool->allocate(*m_descriptor_set_layout);
    }

    void create_pipeline(const std::string& shader_path) {
        const vk::DescriptorSetLayout set_layout = *m_descriptor_set_layout->layout();
        vk::PipelineLayoutCreateInfo pipeline_layout_info{};
        pipeline_layout_info.setLayoutCount = 1;
        pipeline_layout_info.pSetLayouts = &set_layout;
        m_pipeline_layout = vk::raii::PipelineLayout{m_device->device(), pipeline_layout_info};

        m_shader_module = std::make_unique<rtr::rhi::ShaderModule>(
            rtr::rhi::ShaderModule::from_file(
                m_device,
                shader_path,
                vk::ShaderStageFlagBits::eCompute
            )
        );

        vk::ComputePipelineCreateInfo compute_pipeline_info{};
        compute_pipeline_info.stage = m_shader_module->stage_create_info();
        compute_pipeline_info.layout = *m_pipeline_layout;
        m_compute_pipeline = vk::raii::Pipeline{
            m_device->device(),
            nullptr,
            compute_pipeline_info
        };
    }
};

} // namespace

int main() {
    constexpr uint32_t kElementCount = 256;
    constexpr vk::DeviceSize kBufferBytes = sizeof(float) * kElementCount;
    constexpr int kWidth = 800;
    constexpr int kHeight = 600;
    constexpr uint32_t kMaxFramesInFlight = 2;
    const std::string kShaderPath =
        "/Users/jinceyang/Desktop/codebase/graphics/rtr2/build/Debug/shaders/compiled/hello_world_comp.spv";

    try {
        auto renderer = std::make_unique<rtr::system::render::Renderer>(
            kWidth,
            kHeight,
            "RTR Compute Buffer Add",
            kMaxFramesInFlight
        );

        auto buffer0 = std::make_unique<rtr::rhi::Buffer>(
            rtr::rhi::Buffer::create_host_visible_buffer(
                &renderer->device(),
                kBufferBytes,
                vk::BufferUsageFlagBits::eStorageBuffer
            )
        );
        auto buffer1 = std::make_unique<rtr::rhi::Buffer>(
            rtr::rhi::Buffer::create_host_visible_buffer(
                &renderer->device(),
                kBufferBytes,
                vk::BufferUsageFlagBits::eStorageBuffer
            )
        );
        auto result = std::make_unique<rtr::rhi::Buffer>(
            rtr::rhi::Buffer::create_host_visible_buffer(
                &renderer->device(),
                kBufferBytes,
                vk::BufferUsageFlagBits::eStorageBuffer
            )
        );

        buffer0->map();
        buffer1->map();
        result->map();

        {
            auto* buffer0_data = static_cast<float*>(buffer0->mapped_data());
            auto* buffer1_data = static_cast<float*>(buffer1->mapped_data());
            auto* result_data = static_cast<float*>(result->mapped_data());

            for (uint32_t i = 0; i < kElementCount; ++i) {
                buffer0_data[i] = static_cast<float>(i);
                buffer1_data[i] = static_cast<float>(100 + i);
                result_data[i] = 0.0f;
            }

            ComputeBufferAddKernel kernel(
                &renderer->device(),
                kShaderPath,
                kElementCount,
                kBufferBytes
            );
            kernel.bind_buffers(*buffer0, *buffer1, *result);

            renderer->compute([&](rtr::rhi::CommandBuffer& cb) {
                kernel.record(cb);
            });

            bool all_pass = true;
            for (uint32_t i = 0; i < kElementCount; ++i) {
                const float expected = buffer0_data[i] + buffer1_data[i];
                const float actual = result_data[i];
                const bool pass = std::memcmp(&expected, &actual, sizeof(float)) == 0;
                all_pass = all_pass && pass;
                std::cout << "result[" << i << "] = " << actual
                        << " (expected " << expected << ")"
                        << (pass ? " [OK]" : " [FAIL]") << '\n';
            }

            std::cout << (all_pass ? "PASS" : "FAIL") << std::endl;
        }

        result->unmap();
        buffer1->unmap();
        buffer0->unmap();
        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
