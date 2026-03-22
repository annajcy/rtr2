#pragma once

#include <cstdint>
#include <cstring>
#include <span>
#include <stdexcept>
#include <utility>

#include "rtr/rhi/buffer.hpp"
#include "rtr/rhi/device.hpp"
#include "rtr/rhi/mesh.hpp"
#include "rtr/utils/log.hpp"
#include "rtr/utils/obj_types.hpp"

namespace rtr::rhi {

class DynamicMesh {
private:
    std::reference_wrapper<Device> m_device;
    uint32_t m_vertex_count{0};
    uint32_t m_index_count{0};
    Buffer m_vertex_buffer;
    Buffer m_index_buffer;
    
public:
    using Vertex = utils::ObjVertex;

    DynamicMesh(Device& device, const utils::ObjMeshData& initial_data)
        : m_device(device),
          m_vertex_count(static_cast<uint32_t>(initial_data.vertices.size())),
          m_index_count(static_cast<uint32_t>(initial_data.indices.size())),
          m_vertex_buffer(Buffer::create_host_visible_buffer(
              device,
              sizeof(Vertex) * initial_data.vertices.size(),
              vk::BufferUsageFlagBits::eVertexBuffer
          )),
          m_index_buffer(Mesh::create_device_local_with_data(
              device,
              initial_data.indices.data(),
              sizeof(uint32_t) * initial_data.indices.size(),
              vk::BufferUsageFlagBits::eIndexBuffer
          )) {
        
        auto logger = utils::get_logger("rhi.dynamic_mesh");
        if (m_vertex_count == 0 || m_index_count == 0) {
            logger->error("DynamicMesh initialization failed: empty data.");
            throw std::invalid_argument("DynamicMesh requires non-empty initial data.");
        }

        m_vertex_buffer.map();
        update_vertices(initial_data.vertices);
        logger->debug("DynamicMesh initialized (vertices={}, indices={})", m_vertex_count, m_index_count);
    }

    ~DynamicMesh() {
        if (m_vertex_buffer.is_mapped()) {
            m_vertex_buffer.unmap();
        }
    }

    DynamicMesh(const DynamicMesh&) = delete;
    DynamicMesh& operator=(const DynamicMesh&) = delete;
    DynamicMesh(DynamicMesh&&) noexcept = default;
    DynamicMesh& operator=(DynamicMesh&&) noexcept = default;

    void update_vertices(std::span<const Vertex> new_vertices) {
        if (new_vertices.size() != m_vertex_count) {
            throw std::invalid_argument("DynamicMesh update_vertices: size mismatch.");
        }
        std::memcpy(m_vertex_buffer.mapped_data(), new_vertices.data(), new_vertices.size() * sizeof(Vertex));
    }

    vk::Buffer vertex_buffer() const { return *m_vertex_buffer.buffer(); }
    vk::Buffer index_buffer() const { return *m_index_buffer.buffer(); }
    uint32_t vertex_count() const { return m_vertex_count; }
    uint32_t index_count() const { return m_index_count; }
};

} // namespace rtr::rhi
