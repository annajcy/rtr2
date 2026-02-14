#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rtr/resource/resource_types.hpp"
#include "rtr/rhi/device.hpp"
#include "rtr/rhi/texture.hpp"
#include "rtr/system/render/mesh.hpp"
#include "rtr/utils/image_loader.hpp"
#include "rtr/utils/obj_loader.hpp"

namespace rtr::resource {

class ResourceManager {
private:
    struct MeshRecord {
        std::string path{};
        bool alive{true};
        bool cpu_loaded{false};
        CpuMeshData cpu{};
        std::unique_ptr<system::render::Mesh> gpu{};
    };

    struct TextureRecord {
        std::string path{};
        bool alive{true};
        bool cpu_loaded{false};
        CpuTextureData cpu{};
        std::unique_ptr<rhi::Image> gpu{};
    };

    struct RetiredMeshGpu {
        std::uint64_t retire_after_frame{0};
        std::unique_ptr<system::render::Mesh> mesh{};
    };

    struct RetiredTextureGpu {
        std::uint64_t retire_after_frame{0};
        std::unique_ptr<rhi::Image> image{};
    };

    std::uint64_t m_next_mesh_id{1};
    std::uint64_t m_next_texture_id{1};
    std::uint64_t m_current_frame_serial{0};
    std::uint32_t m_frames_in_flight{2};

    std::unordered_map<MeshHandle, MeshRecord> m_mesh_records{};
    std::unordered_map<TextureHandle, TextureRecord> m_texture_records{};

    std::unordered_map<std::string, MeshHandle> m_mesh_by_path{};
    std::unordered_map<std::string, TextureHandle> m_texture_by_path{};

    std::vector<RetiredMeshGpu> m_retired_meshes{};
    std::vector<RetiredTextureGpu> m_retired_textures{};

public:
    explicit ResourceManager(std::uint32_t frames_in_flight = 2)
        : m_frames_in_flight(std::max<std::uint32_t>(frames_in_flight, 1)) {}

    void set_frames_in_flight(std::uint32_t frames_in_flight) {
        m_frames_in_flight = std::max<std::uint32_t>(frames_in_flight, 1);
    }

    MeshHandle load_mesh(const std::string& path) {
        const std::string normalized = normalize_path(path);
        if (const auto it = m_mesh_by_path.find(normalized); it != m_mesh_by_path.end()) {
            return it->second;
        }

        const MeshHandle handle{m_next_mesh_id++};
        m_mesh_records.emplace(handle, MeshRecord{
            .path = normalized,
            .alive = true,
            .cpu_loaded = false,
            .cpu = {},
            .gpu = nullptr
        });
        m_mesh_by_path.emplace(normalized, handle);
        return handle;
    }

    TextureHandle load_texture(const std::string& path, bool use_srgb = true) {
        const std::string normalized = normalize_path(path);
        if (const auto it = m_texture_by_path.find(normalized); it != m_texture_by_path.end()) {
            return it->second;
        }

        const TextureHandle handle{m_next_texture_id++};
        TextureRecord record{};
        record.path = normalized;
        record.alive = true;
        record.cpu_loaded = false;
        record.cpu.use_srgb = use_srgb;

        m_texture_records.emplace(handle, std::move(record));
        m_texture_by_path.emplace(normalized, handle);
        return handle;
    }

    void unload_mesh(MeshHandle handle) {
        auto it = m_mesh_records.find(handle);
        if (it == m_mesh_records.end() || !it->second.alive) {
            return;
        }

        MeshRecord& record = it->second;
        record.alive = false;
        record.cpu_loaded = false;
        record.cpu = {};
        m_mesh_by_path.erase(record.path);
        retire_mesh_gpu(record);
    }

    void unload_texture(TextureHandle handle) {
        auto it = m_texture_records.find(handle);
        if (it == m_texture_records.end() || !it->second.alive) {
            return;
        }

        TextureRecord& record = it->second;
        record.alive = false;
        record.cpu_loaded = false;
        const bool use_srgb = record.cpu.use_srgb;
        record.cpu = {};
        record.cpu.use_srgb = use_srgb;
        m_texture_by_path.erase(record.path);
        retire_texture_gpu(record);
    }

    const CpuMeshData& mesh_cpu(MeshHandle handle) {
        auto& record = require_mesh_record(handle);
        ensure_mesh_cpu_loaded(record);
        return record.cpu;
    }

    const CpuTextureData& texture_cpu(TextureHandle handle) {
        auto& record = require_texture_record(handle);
        ensure_texture_cpu_loaded(record);
        return record.cpu;
    }

    bool mesh_alive(MeshHandle handle) const {
        const auto it = m_mesh_records.find(handle);
        return it != m_mesh_records.end() && it->second.alive;
    }

    bool texture_alive(TextureHandle handle) const {
        const auto it = m_texture_records.find(handle);
        return it != m_texture_records.end() && it->second.alive;
    }

    system::render::Mesh& require_mesh_rhi(MeshHandle handle, rhi::Device* device) {
        if (device == nullptr) {
            throw std::invalid_argument("ResourceManager require_mesh_rhi requires non-null device.");
        }

        auto& record = require_mesh_record(handle);
        ensure_mesh_cpu_loaded(record);

        if (record.gpu) {
            return *record.gpu;
        }
        record.gpu = std::make_unique<system::render::Mesh>(
            system::render::Mesh::from_cpu_data(device, record.cpu)
        );
        return *record.gpu;
    }

    rhi::Image& require_texture_rhi(TextureHandle handle, rhi::Device* device) {
        if (device == nullptr) {
            throw std::invalid_argument("ResourceManager require_texture_rhi requires non-null device.");
        }

        auto& record = require_texture_record(handle);
        ensure_texture_cpu_loaded(record);

        if (record.gpu) {
            return *record.gpu;
        }
        const auto& cpu = record.cpu;
        record.gpu = std::make_unique<rhi::Image>(
            rhi::Image::create_image_from_rgba8(
                device,
                cpu.width,
                cpu.height,
                cpu.pixels.data(),
                cpu.pixels.size(),
                cpu.use_srgb,
                true
            )
        );
        return *record.gpu;
    }

    void tick(std::uint64_t frame_serial) {
        m_current_frame_serial = frame_serial;

        std::erase_if(m_retired_meshes, [frame_serial](const RetiredMeshGpu& retired) {
            return retired.retire_after_frame <= frame_serial;
        });

        std::erase_if(m_retired_textures, [frame_serial](const RetiredTextureGpu& retired) {
            return retired.retire_after_frame <= frame_serial;
        });
    }

    void flush_after_wait_idle() {
        m_retired_meshes.clear();
        m_retired_textures.clear();
    }

private:
    static std::string normalize_path(const std::string& path) {
        if (path.empty()) {
            throw std::invalid_argument("Resource path must not be empty.");
        }

        return std::filesystem::path(path).lexically_normal().string();
    }

    std::uint64_t retire_after_frame() const {
        return m_current_frame_serial + static_cast<std::uint64_t>(m_frames_in_flight);
    }

    static CpuMeshData load_mesh_cpu_data(const std::string& path) {
        const auto loaded = utils::load_obj(path);
        if (loaded.vertices.empty() || loaded.indices.empty()) {
            throw std::runtime_error("Mesh resource is empty: " + path);
        }

        CpuMeshData cpu{};
        cpu.vertices = loaded.vertices;
        cpu.indices = loaded.indices;
        return cpu;
    }

    static CpuTextureData load_texture_cpu_data(const std::string& path, bool use_srgb) {
        const utils::ImageLoader loader(path, true, 4);

        CpuTextureData cpu{};
        cpu.width = static_cast<std::uint32_t>(loader.width());
        cpu.height = static_cast<std::uint32_t>(loader.height());
        cpu.channels = static_cast<std::uint32_t>(loader.channels());
        cpu.use_srgb = use_srgb;
        cpu.pixels.resize(loader.data_size());
        std::memcpy(cpu.pixels.data(), loader.data(), loader.data_size());
        return cpu;
    }

    void ensure_mesh_cpu_loaded(MeshRecord& record) {
        if (record.cpu_loaded) {
            return;
        }

        record.cpu = load_mesh_cpu_data(record.path);
        record.cpu_loaded = true;
    }

    void ensure_texture_cpu_loaded(TextureRecord& record) {
        if (record.cpu_loaded) {
            return;
        }

        record.cpu = load_texture_cpu_data(record.path, record.cpu.use_srgb);
        record.cpu_loaded = true;
    }

    void retire_mesh_gpu(MeshRecord& record) {
        if (!record.gpu) {
            return;
        }

        m_retired_meshes.emplace_back(RetiredMeshGpu{
            .retire_after_frame = retire_after_frame(),
            .mesh = std::move(record.gpu)
        });
    }

    void retire_texture_gpu(TextureRecord& record) {
        if (!record.gpu) {
            return;
        }

        m_retired_textures.emplace_back(RetiredTextureGpu{
            .retire_after_frame = retire_after_frame(),
            .image = std::move(record.gpu)
        });
    }

    MeshRecord& require_mesh_record(MeshHandle handle) {
        const auto it = m_mesh_records.find(handle);
        if (it == m_mesh_records.end() || !it->second.alive) {
            throw std::runtime_error("Mesh handle is invalid or unloaded.");
        }
        return it->second;
    }

    TextureRecord& require_texture_record(TextureHandle handle) {
        const auto it = m_texture_records.find(handle);
        if (it == m_texture_records.end() || !it->second.alive) {
            throw std::runtime_error("Texture handle is invalid or unloaded.");
        }
        return it->second;
    }

};

} // namespace rtr::resource
