#pragma once

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "stb_image.h"
#include "stb_image_write.h"

namespace rtr::utils {

struct ImageData {
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::uint32_t channels{0};
    std::vector<std::uint8_t> pixels{};
};

namespace image_io_detail {

inline void ensure_parent_directory(const std::filesystem::path& path) {
    if (!path.has_parent_path()) {
        return;
    }
    std::filesystem::create_directories(path.parent_path());
}

inline std::string lowercase_extension(std::filesystem::path path) {
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return ext;
}

inline void flip_image_rows(ImageData& image) {
    if (image.height <= 1 || image.channels == 0 || image.pixels.empty()) {
        return;
    }

    const std::size_t row_size =
        static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.channels);
    std::vector<std::uint8_t> tmp(row_size);

    for (std::uint32_t y = 0; y < image.height / 2u; ++y) {
        auto* top = image.pixels.data() + static_cast<std::size_t>(y) * row_size;
        auto* bottom = image.pixels.data() + static_cast<std::size_t>(image.height - 1u - y) * row_size;
        std::memcpy(tmp.data(), top, row_size);
        std::memcpy(top, bottom, row_size);
        std::memcpy(bottom, tmp.data(), row_size);
    }
}

} // namespace image_io_detail

inline ImageData load_image_from_path(
    const std::string& path,
    bool flip_y = true,
    int desired_channels = 4
) {
    if (path.empty()) {
        throw std::invalid_argument("Image path must not be empty.");
    }
    if (desired_channels < 0 || desired_channels > 4) {
        throw std::invalid_argument("desired_channels must be in [0, 4].");
    }

    int width = 0;
    int height = 0;
    int original_channels = 0;
    stbi_uc* raw = stbi_load(path.c_str(), &width, &height, &original_channels, desired_channels);
    if (raw == nullptr) {
        throw std::runtime_error("Failed to load image: " + path);
    }

    const int channels = (desired_channels != 0) ? desired_channels : original_channels;
    if (width <= 0 || height <= 0 || channels <= 0) {
        stbi_image_free(raw);
        throw std::runtime_error("Loaded image has invalid dimensions/channels: " + path);
    }

    const std::size_t data_size = static_cast<std::size_t>(width) *
                                  static_cast<std::size_t>(height) *
                                  static_cast<std::size_t>(channels);

    ImageData image{};
    image.width = static_cast<std::uint32_t>(width);
    image.height = static_cast<std::uint32_t>(height);
    image.channels = static_cast<std::uint32_t>(channels);
    image.pixels.resize(data_size);
    std::memcpy(image.pixels.data(), raw, data_size);
    stbi_image_free(raw);

    if (flip_y) {
        image_io_detail::flip_image_rows(image);
    }

    return image;
}

inline void write_image_to_path(const ImageData& image, const std::string& path) {
    if (path.empty()) {
        throw std::invalid_argument("Image output path must not be empty.");
    }
    if (image.width == 0 || image.height == 0) {
        throw std::invalid_argument("ImageData width/height must be positive.");
    }
    if (image.channels == 0 || image.channels > 4) {
        throw std::invalid_argument("ImageData channels must be in [1, 4].");
    }

    const std::size_t expected_size = static_cast<std::size_t>(image.width) *
                                      static_cast<std::size_t>(image.height) *
                                      static_cast<std::size_t>(image.channels);
    if (image.pixels.size() < expected_size) {
        throw std::invalid_argument("ImageData pixels size is smaller than width*height*channels.");
    }

    const std::filesystem::path out_path(path);
    image_io_detail::ensure_parent_directory(out_path);
    const std::string ext = image_io_detail::lowercase_extension(out_path);

    if (ext == ".png") {
        const int ret = stbi_write_png(
            path.c_str(),
            static_cast<int>(image.width),
            static_cast<int>(image.height),
            static_cast<int>(image.channels),
            image.pixels.data(),
            static_cast<int>(image.width * image.channels)
        );
        if (ret == 0) {
            throw std::runtime_error("Failed to write PNG image: " + path);
        }
        return;
    }

    if (ext == ".ppm") {
        std::ofstream out(out_path, std::ios::binary);
        if (!out.is_open()) {
            throw std::runtime_error("Failed to open PPM output path: " + path);
        }

        out << "P6\n" << image.width << ' ' << image.height << "\n255\n";
        const std::size_t pixel_count = static_cast<std::size_t>(image.width) *
                                        static_cast<std::size_t>(image.height);
        for (std::size_t px = 0; px < pixel_count; ++px) {
            const std::size_t base = px * image.channels;
            std::uint8_t rgb[3]{0, 0, 0};
            if (image.channels == 1u) {
                rgb[0] = image.pixels[base];
                rgb[1] = image.pixels[base];
                rgb[2] = image.pixels[base];
            } else if (image.channels == 2u) {
                rgb[0] = image.pixels[base + 0u];
                rgb[1] = image.pixels[base + 1u];
                rgb[2] = 0u;
            } else {
                rgb[0] = image.pixels[base + 0u];
                rgb[1] = image.pixels[base + 1u];
                rgb[2] = image.pixels[base + 2u];
            }
            out.write(reinterpret_cast<const char*>(rgb), sizeof(rgb));
        }

        if (!out.good()) {
            throw std::runtime_error("Failed to write PPM image: " + path);
        }
        return;
    }

    throw std::invalid_argument("Unsupported image output extension: " + ext);
}

} // namespace rtr::utils
