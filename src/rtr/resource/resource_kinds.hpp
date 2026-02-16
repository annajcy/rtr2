#pragma once

#include <algorithm>
#include <concepts>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

#include "rtr/rhi/device.hpp"
#include "rtr/rhi/mesh.hpp"
#include "rtr/rhi/texture.hpp"
#include "rtr/utils/image_io.hpp"
#include "rtr/utils/obj_io.hpp"

namespace rtr::resource {

struct MeshResourceKind {
    using cpu_type = utils::ObjMeshData;
    using gpu_type = rhi::Mesh;
    using options_type = std::monostate;

    static void validate_cpu(const cpu_type& mesh) {
        if (mesh.vertices.empty() || mesh.indices.empty()) {
            throw std::invalid_argument("ObjMeshData must not be empty.");
        }
    }

    static cpu_type normalize_cpu(cpu_type mesh, const options_type&) {
        return mesh;
    }

    static cpu_type load_from_path(const std::filesystem::path& abs_path, const options_type&) {
        return utils::load_obj_from_path(abs_path.string());
    }

    static void save_to_path(const cpu_type& mesh, const std::filesystem::path& abs_path) {
        utils::write_obj_to_path(mesh, abs_path.string());
    }

    static gpu_type upload_to_gpu(rhi::Device* device, const cpu_type& mesh, const options_type&) {
        return rhi::Mesh::from_cpu_data(device, mesh);
    }
};

struct TextureCreateOptions {
    bool use_srgb{true};
};

struct TextureResourceKind {
    using cpu_type = utils::ImageData;
    using gpu_type = rhi::Image;
    using options_type = TextureCreateOptions;

    static void validate_cpu(const cpu_type& image) {
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

    static cpu_type normalize_cpu(cpu_type image, const options_type&) {
        if (image.channels == 4u) {
            return image;
        }

        const std::size_t pixel_count =
            static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);

        cpu_type rgba{};
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

    static cpu_type load_from_path(const std::filesystem::path& abs_path, const options_type&) {
        return utils::load_image_from_path(abs_path.string(), true, 4);
    }

    static void save_to_path(const cpu_type& image, const std::filesystem::path& abs_path) {
        utils::write_image_to_path(image, abs_path.string());
    }

    static gpu_type upload_to_gpu(rhi::Device* device, const cpu_type& image, const options_type& options) {
        return rhi::Image::from_rgba8(
            device,
            image.width,
            image.height,
            image.pixels.data(),
            image.pixels.size(),
            options.use_srgb,
            true
        );
    }
};

template <class Kind>
concept ResourceKind = requires(
    typename Kind::cpu_type cpu,
    const typename Kind::cpu_type const_cpu,
    const typename Kind::options_type options,
    const std::filesystem::path& path,
    rhi::Device* device
) {
    typename Kind::cpu_type;
    typename Kind::gpu_type;
    typename Kind::options_type;

    { Kind::validate_cpu(const_cpu) } -> std::same_as<void>;
    { Kind::normalize_cpu(std::move(cpu), options) } -> std::same_as<typename Kind::cpu_type>;
    { Kind::load_from_path(path, options) } -> std::same_as<typename Kind::cpu_type>;
    { Kind::save_to_path(const_cpu, path) } -> std::same_as<void>;
    { Kind::upload_to_gpu(device, const_cpu, options) } -> std::same_as<typename Kind::gpu_type>;
};

} // namespace rtr::resource
