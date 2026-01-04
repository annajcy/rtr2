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

public:
    struct LayoutTransitionConfig {
        vk::ImageLayout old_layout{vk::ImageLayout::eUndefined};
        vk::ImageLayout new_layout{vk::ImageLayout::eUndefined};
        vk::PipelineStageFlags src_stage{};
        vk::PipelineStageFlags dst_stage{};
        vk::AccessFlags src_access{};
        vk::AccessFlags dst_access{};
        vk::ImageAspectFlags aspect_mask{vk::ImageAspectFlagBits::eColor};
    };

    static void copy_buffer_to_image(
        vk::CommandBuffer cmd,
        vk::Buffer src,
        vk::Image image,
        uint32_t width,
        uint32_t height,
        vk::ImageAspectFlags aspect_mask
    ) {
        
        vk::BufferImageCopy region{};
        region.bufferOffset = 0;
        region.imageSubresource = { aspect_mask, 0, 0, 1 };
        region.imageExtent.width = width;
        region.imageExtent.height = height;
        region.imageExtent.depth = 1;

        cmd.copyBufferToImage(src, image, vk::ImageLayout::eTransferDstOptimal, region);
    }

    static void transition_image_layout(
        vk::CommandBuffer cmd,
        vk::Image image,
        const LayoutTransitionConfig& config
    ) {
        vk::ImageMemoryBarrier barrier{};
        barrier.oldLayout = config.old_layout;
        barrier.newLayout = config.new_layout;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = image;
        barrier.subresourceRange.aspectMask = config.aspect_mask;
        barrier.subresourceRange.baseMipLevel = 0;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.baseArrayLayer = 0;
        barrier.subresourceRange.layerCount = 1;
        barrier.srcAccessMask = config.src_access;
        barrier.dstAccessMask = config.dst_access;

        cmd.pipelineBarrier(
            config.src_stage,
            config.dst_stage,
            {},
            nullptr,
            nullptr,
            barrier
        );
    }

    static LayoutTransitionConfig make_default_transition(
        vk::ImageLayout old_layout,
        vk::ImageLayout new_layout,
        vk::ImageAspectFlags aspect_mask
    ) {
        LayoutTransitionConfig config{};
        config.old_layout = old_layout;
        config.new_layout = new_layout;
        config.aspect_mask = aspect_mask;

        if (old_layout == vk::ImageLayout::eUndefined &&
            new_layout == vk::ImageLayout::eTransferDstOptimal) {
            config.src_access = {};
            config.dst_access = vk::AccessFlagBits::eTransferWrite;
            config.src_stage = vk::PipelineStageFlagBits::eTopOfPipe;
            config.dst_stage = vk::PipelineStageFlagBits::eTransfer;
        } else if (old_layout == vk::ImageLayout::eTransferDstOptimal &&
                   new_layout == vk::ImageLayout::eShaderReadOnlyOptimal) {
            config.src_access = vk::AccessFlagBits::eTransferWrite;
            config.dst_access = vk::AccessFlagBits::eShaderRead;
            config.src_stage = vk::PipelineStageFlagBits::eTransfer;
            config.dst_stage = vk::PipelineStageFlagBits::eFragmentShader;
        } else if (old_layout == vk::ImageLayout::eUndefined &&
                   new_layout == vk::ImageLayout::eDepthAttachmentOptimal) {
            config.src_access = {};
            config.dst_access = vk::AccessFlagBits::eDepthStencilAttachmentWrite | vk::AccessFlagBits::eDepthStencilAttachmentRead;
            config.src_stage = vk::PipelineStageFlagBits::eTopOfPipe;
            config.dst_stage = vk::PipelineStageFlagBits::eEarlyFragmentTests;
        } else if (old_layout == vk::ImageLayout::eTransferDstOptimal &&
                   new_layout == vk::ImageLayout::eDepthAttachmentOptimal) {
            config.src_access = vk::AccessFlagBits::eTransferWrite;
            config.dst_access = vk::AccessFlagBits::eDepthStencilAttachmentWrite | vk::AccessFlagBits::eDepthStencilAttachmentRead;
            config.src_stage = vk::PipelineStageFlagBits::eTransfer;
            config.dst_stage = vk::PipelineStageFlagBits::eEarlyFragmentTests;
        } else {
            throw std::invalid_argument("Unsupported layout transition!");
        }

        return config;
    }

    static Image create_depth_image(
        Device* device,
        uint32_t width,
        uint32_t height,
        vk::Format format
    ) {
        Image image(
            device,
            width,
            height,
            format,
            vk::ImageTiling::eOptimal,
            vk::ImageUsageFlagBits::eDepthStencilAttachment,
            vk::MemoryPropertyFlagBits::eDeviceLocal,
            vk::ImageAspectFlagBits::eDepth
        );
        return image;
    }

    static Image create_image_from_file(
        Device* device,
        const std::string& file_path,
        bool use_srgb = true, // 默认为 true，即颜色贴图
        bool flip_y = true
    ) {
        auto image_loader = rtr::utils::ImageLoader(file_path, flip_y, 4);

        auto stage_buffer = Buffer::create_host_visible_buffer(
            device,
            image_loader.data_size(),
            vk::BufferUsageFlagBits::eTransferSrc
        );
        

        // 将图像数据复制到 staging buffer
        stage_buffer.map();
        std::memcpy(stage_buffer.mapped_data(), image_loader.data(), image_loader.data_size());
        stage_buffer.unmap();

        vk::Format format = use_srgb ? vk::Format::eR8G8B8A8Srgb : vk::Format::eR8G8B8A8Unorm;

        auto image = Image(
            device,
            static_cast<uint32_t>(image_loader.width()),
            static_cast<uint32_t>(image_loader.height()),
            format,
            vk::ImageTiling::eOptimal,
            vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eSampled,
            vk::MemoryPropertyFlagBits::eDeviceLocal,
            vk::ImageAspectFlagBits::eColor
        );

        image.upload(stage_buffer, vk::ImageLayout::eShaderReadOnlyOptimal);
        return image;
    }

private:
    Device* m_device{nullptr};
    vk::raii::Image m_image{nullptr};
    vk::raii::ImageView m_image_view{nullptr};
    vk::raii::DeviceMemory m_image_memory{nullptr};

    uint32_t m_width{0};
    uint32_t m_height{0};
    vk::Format m_format{vk::Format::eR8G8B8A8Unorm};
    vk::ImageAspectFlags m_aspect_mask{vk::ImageAspectFlagBits::eColor};
    vk::ImageUsageFlags m_usage{};
    vk::ImageLayout m_current_layout{vk::ImageLayout::eUndefined};

public:
    Image(Device* device,
          uint32_t width,
          uint32_t height,
          vk::Format format,
          vk::ImageTiling tiling,
          vk::ImageUsageFlags usage,
          vk::MemoryPropertyFlags properties,
          vk::ImageAspectFlags aspect_mask)
        : m_device(device), m_width(width), m_height(height), m_format(format), m_aspect_mask(aspect_mask), m_usage(usage) {

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

        vk::ImageViewCreateInfo view_info{};
        view_info.image = *m_image;
        view_info.viewType = vk::ImageViewType::e2D;
        view_info.format = format;
        view_info.subresourceRange.aspectMask = m_aspect_mask;
        view_info.subresourceRange.baseMipLevel = 0;
        view_info.subresourceRange.levelCount = 1;
        view_info.subresourceRange.baseArrayLayer = 0;
        view_info.subresourceRange.layerCount = 1;

        m_image_view = vk::raii::ImageView(m_device->device(), view_info);
    }

    Image(const Image&) = delete;
    Image& operator=(const Image&) = delete;

    Image(Image&& other) noexcept 
        : m_device(other.m_device),
          m_image(std::move(other.m_image)),
          m_image_view(std::move(other.m_image_view)),
          m_image_memory(std::move(other.m_image_memory)),
          m_width(other.m_width),
          m_height(other.m_height),
          m_format(other.m_format),
          m_aspect_mask(other.m_aspect_mask),
          m_usage(other.m_usage),
          m_current_layout(other.m_current_layout)
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
            m_image_view = std::move(other.m_image_view);
            m_width = other.m_width;
            m_height = other.m_height;
            m_format = other.m_format;
            m_aspect_mask = other.m_aspect_mask;
            m_usage = other.m_usage;
            m_current_layout = other.m_current_layout;

            other.m_width = 0;
            other.m_height = 0;
        }
        return *this;
    }

    const vk::raii::Image& image() const { return m_image; }
    const vk::raii::ImageView& image_view() const { return m_image_view; }
    const vk::raii::DeviceMemory& memory() const { return m_image_memory; }
    uint32_t width() const { return m_width; }
    uint32_t height() const { return m_height; }
    vk::Format format() const { return m_format; }
    vk::ImageAspectFlags aspect_mask() const { return m_aspect_mask; }
    vk::ImageUsageFlags usage() const { return m_usage; }
    vk::ImageLayout layout() const { return m_current_layout; }

    void apply_transition(vk::CommandBuffer cmd, const LayoutTransitionConfig& config) {
        transition_image_layout(cmd, *m_image, config);
        m_current_layout = config.new_layout;
    }

private:
    void upload(Buffer& stage_buffer, vk::ImageLayout final_layout) {
        CommandPool command_pool(m_device, vk::CommandPoolCreateFlagBits::eTransient);
        auto cmd = command_pool.create_command_buffer();
        vk::raii::Fence upload_fence(m_device->device(), vk::FenceCreateInfo{});
        CommandBuffer::SubmitInfo submit_info{};
        submit_info.fence = *upload_fence;
        // Transition image layout and copy buffer to image
        cmd.record_and_submit([&](CommandBuffer& cb){

            auto to_transfer = make_default_transition(
                m_current_layout,
                vk::ImageLayout::eTransferDstOptimal,
                m_aspect_mask
            );
            apply_transition(*cb.command_buffer(), to_transfer);

            copy_buffer_to_image(
                *cb.command_buffer(),
                *stage_buffer.buffer(),
                *m_image,
                m_width,
                m_height,
                m_aspect_mask
            );

            auto to_shader = make_default_transition(
                m_current_layout,
                final_layout,
                m_aspect_mask
            );
            apply_transition(*cb.command_buffer(), to_shader);
        }, submit_info);

        
        auto result = m_device->device().waitForFences(*upload_fence, VK_TRUE, UINT64_MAX);
        if (result != vk::Result::eSuccess) {
            throw std::runtime_error("Failed to wait for image upload fence!");
        }

    }
};

class Sampler {
private:
    Device* m_device{nullptr};
    vk::raii::Sampler m_sampler{nullptr};

public:
    // =========================================================================
    // 静态工厂方法：创建一个标准的、高质量的采样器
    // 特性：线性过滤 (Linear), 重复平铺 (Repeat), 开启最大各向异性过滤 (Anisotropy)
    // =========================================================================
    static Sampler create_default(Device* device) {
        // 1. 获取物理设备属性，用于查询支持的最大各向异性过滤级别
        vk::PhysicalDeviceProperties properties = device->physical_device().getProperties();

        vk::SamplerCreateInfo info{};
        
        // --- 过滤设置 (Filtering) ---
        // Mag: 纹理被放大时 (离相机很近) -> 线性插值 (平滑)
        // Min: 纹理被缩小时 (离相机很远) -> 线性插值 (平滑)
        info.magFilter = vk::Filter::eLinear;
        info.minFilter = vk::Filter::eLinear;

        // --- 寻址模式 (Addressing) ---
        // U, V, W 对应 X, Y, Z 轴。
        // eRepeat: 超过 0~1 范围时重复纹理 (适合地板、墙壁)
        info.addressModeU = vk::SamplerAddressMode::eRepeat;
        info.addressModeV = vk::SamplerAddressMode::eRepeat;
        info.addressModeW = vk::SamplerAddressMode::eRepeat;

        // --- 各向异性过滤 (Anisotropy) ---
        // 解决倾斜观察时的模糊问题
        info.anisotropyEnable = VK_TRUE; 
        // 使用显卡支持的最大级别 (通常是 16.0f)
        info.maxAnisotropy = properties.limits.maxSamplerAnisotropy;

        // --- 其他设置 ---
        // 边框颜色 (当使用 ClampToBorder 时有效，这里没用但填上个默认值)
        info.borderColor = vk::BorderColor::eIntOpaqueBlack;
        
        // 坐标系归一化: True 表示用 [0, 1] 访问，False 表示用像素坐标 [0, width]
        // 几乎总是用 True
        info.unnormalizedCoordinates = VK_FALSE;

        // 比较操作 (用于阴影贴图 PCF，这里暂时不用)
        info.compareEnable = VK_FALSE;
        info.compareOp = vk::CompareOp::eAlways;

        // --- Mipmap 设置 ---
        // 暂时还没生成 Mipmap，但设置线性模式是安全的标准做法
        info.mipmapMode = vk::SamplerMipmapMode::eLinear;
        info.mipLodBias = 0.0f;
        info.minLod = 0.0f;
        info.maxLod = 0.0f;

        return Sampler(device, info);
    }

    // 工厂方法：创建一个像素风采样器 (Nearest, Clamp)
    static Sampler create_pixel_art_style(Device* device) {
        vk::SamplerCreateInfo info{};
        info.magFilter = vk::Filter::eNearest; // 马赛克效果
        info.minFilter = vk::Filter::eNearest;
        info.addressModeU = vk::SamplerAddressMode::eClampToEdge; // 不重复
        info.addressModeV = vk::SamplerAddressMode::eClampToEdge;
        info.addressModeW = vk::SamplerAddressMode::eClampToEdge;
        info.anisotropyEnable = VK_FALSE; // 像素风通常不需要各向异性
        info.unnormalizedCoordinates = VK_FALSE;
        info.mipmapMode = vk::SamplerMipmapMode::eNearest;
        
        return Sampler(device, info);
    }

public:
    // 通用构造函数
    Sampler(Device* device, const vk::SamplerCreateInfo& create_info) 
        : m_device(device) {
        m_sampler = vk::raii::Sampler(device->device(), create_info);
    }

    // Rule of Five: 禁用拷贝，允许移动
    Sampler(const Sampler&) = delete;
    Sampler& operator=(const Sampler&) = delete;

    Sampler(Sampler&& other) noexcept 
        : m_device(other.m_device), m_sampler(std::move(other.m_sampler)) {}

    Sampler& operator=(Sampler&& other) noexcept {
        if (this != &other) {
            m_device = other.m_device;
            m_sampler = std::move(other.m_sampler);
        }
        return *this;
    }

    // Getters
    const vk::raii::Sampler& sampler() const { return m_sampler; }
};

}
