#pragma once

#include "core/command.hpp"
#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vulkan_structs.hpp"
#include <cstdint>
#include <memory>
#include <string>
#include <sys/types.h>

#include "device.hpp"
#include "buffer.hpp"
#include "utils/image_loader.hpp"

namespace rtr::core {

class Image {
private:
    Device* m_device{nullptr};
    vk::raii::Image m_image{nullptr};
    vk::raii::DeviceMemory m_image_memory{nullptr};

    uint32_t m_width{0};
    uint32_t m_height{0};
    vk::Format m_format{vk::Format::eR8G8B8A8Unorm};

public:
    Image(Device* device,
          uint32_t width,
          uint32_t height,
          vk::Format format = vk::Format::eR8G8B8A8Srgb,
          vk::ImageTiling tiling = vk::ImageTiling::eOptimal,
          vk::ImageUsageFlags usage = vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eSampled,
          vk::MemoryPropertyFlags properties = vk::MemoryPropertyFlagBits::eDeviceLocal)
        : m_device(device), m_width(width), m_height(height), m_format(format) {

        auto resources_opt = make_image_with_memory(
            device->device(),
            device->physical_device(),
            width, height, format, tiling, usage, properties
        );

        if (!resources_opt.has_value()) {
            throw std::runtime_error("Failed to create image textures!");
        }

        auto [img, mem] = std::move(resources_opt.value());
        m_image = std::move(img);
        m_image_memory = std::move(mem);
      
    }

    Image(const Image&) = delete;
    Image& operator=(const Image&) = delete;

    Image(Image&& other) noexcept 
        : m_device(other.m_device),
          m_image(std::move(other.m_image)),
          m_image_memory(std::move(other.m_image_memory)),
          m_width(other.m_width),
          m_height(other.m_height),
          m_format(other.m_format)
    {
        other.m_width = 0;
        other.m_height = 0;
    }

    // 移动赋值
    Image& operator=(Image&& other) noexcept {
        if (this != &other) {
            m_device = other.m_device;
            m_image = std::move(other.m_image);
            m_image_memory = std::move(other.m_image_memory);
            m_width = other.m_width;
            m_height = other.m_height;
            m_format = other.m_format;

            other.m_width = 0;
            other.m_height = 0;
        }
        return *this;
    }

    const vk::raii::Image& image() const { return m_image; }
    const vk::raii::DeviceMemory& memory() const { return m_image_memory; }
    uint32_t width() const { return m_width; }
    uint32_t height() const { return m_height; }
    vk::Format format() const { return m_format; }
};

class Texture { 
public:
    static void copy_buffer_to_image(
        vk::CommandBuffer cmd,
        vk::Buffer src,
        vk::Image image,
        uint32_t width,
        uint32_t height
    ) {
        
        vk::BufferImageCopy region{};
        region.bufferOffset = 0;
        region.imageSubresource = { vk::ImageAspectFlagBits::eColor, 0, 0, 1 };
        region.imageExtent.width = width;
        region.imageExtent.height = height;
        region.imageExtent.depth = 1;

        cmd.copyBufferToImage(src, image, vk::ImageLayout::eTransferDstOptimal, region);
    }

    static void transition_image_layout(
        vk::CommandBuffer cmd,
        vk::Image image,
        vk::ImageLayout old_layout,
        vk::ImageLayout new_layout
    ) {
        vk::ImageMemoryBarrier barrier{};
        barrier.oldLayout = old_layout;
        barrier.newLayout = new_layout;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = image;
        barrier.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
        barrier.subresourceRange.baseMipLevel = 0;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.baseArrayLayer = 0;
        barrier.subresourceRange.layerCount = 1;

        vk::PipelineStageFlags source_stage;
        vk::PipelineStageFlags destination_stage;

        if (old_layout == vk::ImageLayout::eUndefined && new_layout == vk::ImageLayout::eTransferDstOptimal) {
            barrier.srcAccessMask = {};
            barrier.dstAccessMask = vk::AccessFlagBits::eTransferWrite;

            source_stage = vk::PipelineStageFlagBits::eTopOfPipe;
            destination_stage = vk::PipelineStageFlagBits::eTransfer;
        } else if (old_layout == vk::ImageLayout::eTransferDstOptimal && new_layout == vk::ImageLayout::eShaderReadOnlyOptimal) {
            barrier.srcAccessMask = vk::AccessFlagBits::eTransferWrite;
            barrier.dstAccessMask = vk::AccessFlagBits::eShaderRead;

            source_stage = vk::PipelineStageFlagBits::eTransfer;
            destination_stage = vk::PipelineStageFlagBits::eFragmentShader;
        } else {
            throw std::invalid_argument("Unsupported layout transition!");
        }

        cmd.pipelineBarrier(
            source_stage,
            destination_stage,
            {},
            nullptr,
            nullptr,
            barrier
        );
    }

private:
    Device* m_device{nullptr};
    std::unique_ptr<Image> m_image;
    
public:
    Texture(Device* device, const std::string& file_path) : m_device(device) {
        auto image_loader = rtr::utils::ImageLoader(file_path, true, 4);

        auto stage_buffer = std::make_unique<Buffer>(
            Buffer::create_host_visible_buffer(
                device,
                image_loader.data_size(),
                vk::BufferUsageFlagBits::eTransferSrc
            )
        );

        // 将图像数据复制到 staging buffer
        stage_buffer->map();
        std::memcpy(stage_buffer->mapped_data(), image_loader.data(), image_loader.data_size());
        stage_buffer->unmap();

        m_image = std::make_unique<Image>(
            device,
            static_cast<uint32_t>(image_loader.width()),
            static_cast<uint32_t>(image_loader.height()),
            vk::Format::eR8G8B8A8Srgb,
            vk::ImageTiling::eOptimal,
            vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eSampled,
            vk::MemoryPropertyFlagBits::eDeviceLocal
        );

        upload(*m_image, *stage_buffer);
    }

private:
    void upload(Image& m_image, Buffer& m_stage_buffer) {
        CommandPool command_pool(m_device, vk::CommandPoolCreateFlagBits::eTransient);
        auto cmd = command_pool.create_command_buffer();
        // Transition image layout and copy buffer to image
        cmd.record_and_submit([&](CommandBuffer& cb){

            transition_image_layout(
                *cb.command_buffer(),
                *m_image.image(), 
                vk::ImageLayout::eUndefined, 
                vk::ImageLayout::eTransferDstOptimal
            );

            copy_buffer_to_image(
                *cb.command_buffer(),
                *m_stage_buffer.buffer(),
                *m_image.image(),
                m_image.width(),
                m_image.height()
            );

            transition_image_layout(
                *cb.command_buffer(),
                *m_image.image(), 
                vk::ImageLayout::eTransferDstOptimal, 
                vk::ImageLayout::eShaderReadOnlyOptimal
            );
            
        });
    }
};


}