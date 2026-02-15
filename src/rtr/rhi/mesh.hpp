#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <utility>


#include "buffer.hpp"
#include "command.hpp"
#include "device.hpp"
#include "rtr/utils/log.hpp"
#include "rtr/utils/obj_types.hpp"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"

namespace rtr::rhi {

class Mesh {
public:
    using Vertex = rtr::utils::ObjVertex;

    static void copy_buffer(
        Device* device,
        vk::Buffer src,
        vk::Buffer dst,
        vk::DeviceSize size
    ) {
        auto logger = utils::get_logger("rhi.mesh");
        logger->debug("GPU buffer copy start (bytes={})", static_cast<std::uint64_t>(size));
        CommandPool command_pool(device, vk::CommandPoolCreateFlagBits::eTransient);
        auto cmd = command_pool.create_command_buffer();

        cmd.record_and_submit([&](CommandBuffer& cb) {
            vk::BufferCopy buffer_copy{};
            buffer_copy.srcOffset = 0;
            buffer_copy.dstOffset = 0;
            buffer_copy.size = size;
            cb.command_buffer().copyBuffer(src, dst, buffer_copy);
        });

        device->queue().waitIdle();
        logger->debug("GPU buffer copy finished (bytes={})", static_cast<std::uint64_t>(size));
    }

    static Buffer create_device_local_with_data(
        Device* device,
        const void* data,
        vk::DeviceSize size,
        vk::BufferUsageFlags usage = {}
    ) {
        auto buffer = Buffer::create_device_local_buffer(
            device,
            size,
            usage | vk::BufferUsageFlagBits::eTransferDst
        );

        auto staging_buffer = Buffer::create_host_visible_buffer(
            device,
            size,
            vk::BufferUsageFlagBits::eTransferSrc
        );

        staging_buffer.map();
        std::memcpy(staging_buffer.mapped_data(), data, size);
        staging_buffer.unmap();

        copy_buffer(device, *staging_buffer.buffer(), *buffer.buffer(), size);

        return buffer;
    }

    static Mesh from_cpu_data(Device* device, const utils::ObjMeshData& cpu_data) {
        auto logger = utils::get_logger("rhi.mesh");
        if (cpu_data.vertices.empty() || cpu_data.indices.empty()) {
            logger->error("Mesh upload failed: CPU mesh data is empty.");
            throw std::runtime_error("Mesh CPU data is empty and cannot create GPU buffers.");
        }
        logger->debug(
            "Uploading mesh to GPU (vertices={}, indices={}, vertex_bytes={}, index_bytes={})",
            cpu_data.vertices.size(),
            cpu_data.indices.size(),
            sizeof(Vertex) * cpu_data.vertices.size(),
            sizeof(uint32_t) * cpu_data.indices.size()
        );

        auto vertex_buffer = std::make_unique<Buffer>(
            Mesh::create_device_local_with_data(
                device,
                cpu_data.vertices.data(),
                sizeof(Vertex) * cpu_data.vertices.size(),
                vk::BufferUsageFlagBits::eVertexBuffer
            )
        );

        auto index_buffer = std::make_unique<Buffer>(
            Mesh::create_device_local_with_data(
                device,
                cpu_data.indices.data(),
                sizeof(uint32_t) * cpu_data.indices.size(),
                vk::BufferUsageFlagBits::eIndexBuffer
            )
        );

        logger->debug("Mesh GPU upload completed.");

        return Mesh(
            device,
            static_cast<uint32_t>(cpu_data.vertices.size()),
            static_cast<uint32_t>(cpu_data.indices.size()),
            std::move(vertex_buffer),
            std::move(index_buffer)
        );
    }

    static vk::VertexInputBindingDescription binding_description() {
        vk::VertexInputBindingDescription desc{};
        desc.binding = 0;
        desc.stride = sizeof(Vertex);
        desc.inputRate = vk::VertexInputRate::eVertex;
        return desc;
    }

    static std::array<vk::VertexInputAttributeDescription, 3> attribute_descriptions() {
        std::array<vk::VertexInputAttributeDescription, 3> attributes{};

        attributes[0].binding = 0;
        attributes[0].location = 0;
        attributes[0].format = vk::Format::eR32G32B32Sfloat;
        attributes[0].offset = offsetof(Vertex, position);

        attributes[1].binding = 0;
        attributes[1].location = 1;
        attributes[1].format = vk::Format::eR32G32Sfloat;
        attributes[1].offset = offsetof(Vertex, uv);

        attributes[2].binding = 0;
        attributes[2].location = 2;
        attributes[2].format = vk::Format::eR32G32B32Sfloat;
        attributes[2].offset = offsetof(Vertex, normal);

        return attributes;
    }

    struct VertexInputState {
        std::array<vk::VertexInputBindingDescription, 1> bindings;
        std::array<vk::VertexInputAttributeDescription, 3> attributes;
    };

    static VertexInputState vertex_input_state() {
        VertexInputState state{};
        state.bindings[0] = binding_description();
        state.attributes = attribute_descriptions();
        return state;
    }

private:
    Device* m_device{};
    uint32_t m_vertex_count{0};
    uint32_t m_index_count{0};
    std::unique_ptr<Buffer> m_vertex_buffer{};
    std::unique_ptr<Buffer> m_index_buffer{};

public:
    Mesh(
        Device* device,
        uint32_t vertex_count,
        uint32_t index_count,
        std::unique_ptr<Buffer> vertex_buffer,
        std::unique_ptr<Buffer> index_buffer
    )
        : m_device(device),
          m_vertex_count(vertex_count),
          m_index_count(index_count),
          m_vertex_buffer(std::move(vertex_buffer)),
          m_index_buffer(std::move(index_buffer)) {}

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&&) noexcept = default;
    Mesh& operator=(Mesh&&) noexcept = default;
    ~Mesh() = default;

    vk::Buffer vertex_buffer() const { return *m_vertex_buffer->buffer(); }
    vk::Buffer index_buffer() const { return *m_index_buffer->buffer(); }
    uint32_t index_count() const { return m_index_count; }
    uint32_t vertex_count() const { return m_vertex_count; }
};

} // namespace rtr::rhi
