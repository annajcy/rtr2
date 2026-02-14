#pragma once

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rtr/resource/resource_types.hpp"
#include "rtr/rhi/device.hpp"
#include "rtr/rhi/texture.hpp"
#include "rtr/system/render/mesh.hpp"
#include "rtr/utils/image_io.hpp"
#include "rtr/utils/obj_io.hpp"

namespace rtr::resource {

class ResourceManager {
public:
    static constexpr std::string_view kDefaultResourceRootDir = "./assets/";

private:
    struct MeshRecord {
        utils::ObjMeshData cpu{};
        std::unique_ptr<system::render::Mesh> gpu{};
    };

    struct TextureRecord {
        utils::ImageData cpu{};
        bool use_srgb{true};
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
    std::filesystem::path m_resource_root_dir{std::string(kDefaultResourceRootDir)};

    std::unordered_map<MeshHandle, MeshRecord> m_mesh_records{};
    std::unordered_map<TextureHandle, TextureRecord> m_texture_records{};

    std::vector<RetiredMeshGpu> m_retired_meshes{};
    std::vector<RetiredTextureGpu> m_retired_textures{};

public:
    explicit ResourceManager(
        std::uint32_t frames_in_flight = 2,
        std::filesystem::path resource_root_dir = std::filesystem::path(std::string(kDefaultResourceRootDir))
    )
        : m_frames_in_flight(std::max<std::uint32_t>(frames_in_flight, 1)),
          m_resource_root_dir(std::move(resource_root_dir)) {}

    void set_frames_in_flight(std::uint32_t frames_in_flight) {
        m_frames_in_flight = std::max<std::uint32_t>(frames_in_flight, 1);
    }

    const std::filesystem::path& resource_root_dir() const {
        return m_resource_root_dir;
    }

    void set_resource_root_dir(std::filesystem::path resource_root_dir) {
        m_resource_root_dir = std::move(resource_root_dir);
    }

    MeshHandle create_mesh(utils::ObjMeshData cpu) {
        validate_mesh_data(cpu);

        const MeshHandle handle{m_next_mesh_id++};
        m_mesh_records.emplace(handle, MeshRecord{
            .cpu = std::move(cpu),
            .gpu = nullptr
        });
        return handle;
    }

    TextureHandle create_texture(utils::ImageData cpu, bool use_srgb = true) {
        validate_texture_data(cpu);
        cpu = normalize_to_rgba8(std::move(cpu));

        const TextureHandle handle{m_next_texture_id++};
        m_texture_records.emplace(handle, TextureRecord{
            .cpu = std::move(cpu),
            .use_srgb = use_srgb,
            .gpu = nullptr
        });
        return handle;
    }

    MeshHandle create_mesh_from_obj_relative_path(const std::string& rel_path) {
        const auto abs_path = resolve_resource_path(rel_path);
        return create_mesh(utils::load_obj_from_path(abs_path.string()));
    }

    TextureHandle create_texture_from_relative_path(
        const std::string& rel_path,
        bool use_srgb = true
    ) {
        const auto abs_path = resolve_resource_path(rel_path);
        return create_texture(utils::load_image_from_path(abs_path.string(), true, 4), use_srgb);
    }

    void unload_mesh(MeshHandle handle) {
        auto it = m_mesh_records.find(handle);
        if (it == m_mesh_records.end()) {
            return;
        }

        MeshRecord& record = it->second;
        retire_mesh_gpu(record);
        m_mesh_records.erase(it);
    }

    void unload_texture(TextureHandle handle) {
        auto it = m_texture_records.find(handle);
        if (it == m_texture_records.end()) {
            return;
        }

        TextureRecord& record = it->second;
        retire_texture_gpu(record);
        m_texture_records.erase(it);
    }

    const utils::ObjMeshData& mesh_cpu(MeshHandle handle) {
        auto& record = require_mesh_record(handle);
        ensure_mesh_cpu_loaded(record);
        return record.cpu;
    }

    const utils::ImageData& texture_cpu(TextureHandle handle) {
        auto& record = require_texture_record(handle);
        ensure_texture_cpu_loaded(record);
        return record.cpu;
    }

    bool mesh_alive(MeshHandle handle) const {
        const auto it = m_mesh_records.find(handle);
        return it != m_mesh_records.end();
    }

    bool texture_alive(TextureHandle handle) const {
        const auto it = m_texture_records.find(handle);
        return it != m_texture_records.end();
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
                record.use_srgb,
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

        // Release all live GPU-side caches while the VkDevice is guaranteed idle.
        // CPU-side resource records remain so assets can be re-uploaded on demand.
        for (auto& [_, mesh_record] : m_mesh_records) {
            mesh_record.gpu.reset();
        }
        for (auto& [_, texture_record] : m_texture_records) {
            texture_record.gpu.reset();
        }
    }

private:
    static void validate_mesh_data(const utils::ObjMeshData& mesh) {
        if (mesh.vertices.empty() || mesh.indices.empty()) {
            throw std::invalid_argument("ObjMeshData must not be empty.");
        }
    }

    static void validate_texture_data(const utils::ImageData& image) {
        if (image.width == 0 || image.height == 0) {
            throw std::invalid_argument("ImageData width/height must be positive.");
        }
        if (image.channels == 0 || image.channels > 4) {
            throw std::invalid_argument("ImageData channels must be in [1, 4].");
        }
        const std::size_t expected_size =
            static_cast<std::size_t>(image.width) *
            static_cast<std::size_t>(image.height) *
            static_cast<std::size_t>(image.channels);
        if (image.pixels.size() < expected_size) {
            throw std::invalid_argument("ImageData pixels size is insufficient.");
        }
    }

    static utils::ImageData normalize_to_rgba8(utils::ImageData image) {
        if (image.channels == 4) {
            return image;
        }

        const std::size_t pixel_count =
            static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);

        utils::ImageData rgba{};
        rgba.width = image.width;
        rgba.height = image.height;
        rgba.channels = 4;
        rgba.pixels.resize(pixel_count * 4u);

        for (std::size_t px = 0; px < pixel_count; ++px) {
            const std::size_t src = px * image.channels;
            const std::size_t dst = px * 4u;

            if (image.channels == 1u) {
                const std::uint8_t v = image.pixels[src];
                rgba.pixels[dst + 0u] = v;
                rgba.pixels[dst + 1u] = v;
                rgba.pixels[dst + 2u] = v;
                rgba.pixels[dst + 3u] = 255u;
            } else if (image.channels == 2u) {
                rgba.pixels[dst + 0u] = image.pixels[src + 0u];
                rgba.pixels[dst + 1u] = image.pixels[src + 1u];
                rgba.pixels[dst + 2u] = 0u;
                rgba.pixels[dst + 3u] = 255u;
            } else {
                rgba.pixels[dst + 0u] = image.pixels[src + 0u];
                rgba.pixels[dst + 1u] = image.pixels[src + 1u];
                rgba.pixels[dst + 2u] = image.pixels[src + 2u];
                rgba.pixels[dst + 3u] = 255u;
            }
        }

        return rgba;
    }

    std::filesystem::path resolve_resource_path(const std::string& rel_path) const {
        if (rel_path.empty()) {
            throw std::invalid_argument("Relative resource path must not be empty.");
        }

        const std::filesystem::path path(rel_path);
        if (path.is_absolute()) {
            throw std::invalid_argument("Relative resource path API does not accept absolute path.");
        }

        return (m_resource_root_dir / path).lexically_normal();
    }

    std::uint64_t retire_after_frame() const {
        return m_current_frame_serial + static_cast<std::uint64_t>(m_frames_in_flight);
    }

    void ensure_mesh_cpu_loaded(const MeshRecord& record) const {
        if (record.cpu.vertices.empty() || record.cpu.indices.empty()) {
            throw std::runtime_error("Mesh CPU data is missing for live handle.");
        }
    }

    void ensure_texture_cpu_loaded(const TextureRecord& record) const {
        if (record.cpu.pixels.empty()) {
            throw std::runtime_error("Texture CPU data is missing for live handle.");
        }
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
        if (it == m_mesh_records.end()) {
            throw std::runtime_error("Mesh handle is invalid or unloaded.");
        }
        return it->second;
    }

    TextureRecord& require_texture_record(TextureHandle handle) {
        const auto it = m_texture_records.find(handle);
        if (it == m_texture_records.end()) {
            throw std::runtime_error("Texture handle is invalid or unloaded.");
        }
        return it->second;
    }
};

} // namespace rtr::resource
