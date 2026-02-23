#pragma once

#include "command.hpp"
#include "vulkan/vulkan.hpp"
#include "vulkan/vulkan_enums.hpp"
#include "vulkan/vulkan_handles.hpp"
#include "vulkan/vulkan_raii.hpp"
#include "vulkan/vulkan_structs.hpp"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <sys/types.h>
#include <algorithm>
#include <cmath>
#include "device.hpp"
#include "buffer.hpp"

#include <functional>
namespace rtr::rhi {

namespace detail {

template<vk::ImageLayout From, vk::ImageLayout To>
struct TransitionTraits {
    static_assert(From != From, "Unsupported layout transition! Check template arguments.");
};

template<>
struct TransitionTraits<vk::ImageLayout::eUndefined, vk::ImageLayout::eTransferDstOptimal> {
    static constexpr vk::AccessFlags src_access{};
    static constexpr vk::AccessFlags dst_access{vk::AccessFlagBits::eTransferWrite};
    static constexpr vk::PipelineStageFlags src_stage{vk::PipelineStageFlagBits::eTopOfPipe};
    static constexpr vk::PipelineStageFlags dst_stage{vk::PipelineStageFlagBits::eTransfer};
};

template<>
struct TransitionTraits<vk::ImageLayout::eTransferDstOptimal, vk::ImageLayout::eShaderReadOnlyOptimal> {
    static constexpr vk::AccessFlags src_access{vk::AccessFlagBits::eTransferWrite};
    static constexpr vk::AccessFlags dst_access{vk::AccessFlagBits::eShaderRead};
    static constexpr vk::PipelineStageFlags src_stage{vk::PipelineStageFlagBits::eTransfer};
    static constexpr vk::PipelineStageFlags dst_stage{vk::PipelineStageFlagBits::eFragmentShader};
};

template<>
struct TransitionTraits<vk::ImageLayout::eTransferDstOptimal, vk::ImageLayout::eTransferSrcOptimal> {
    static constexpr vk::AccessFlags src_access{vk::AccessFlagBits::eTransferWrite};
    static constexpr vk::AccessFlags dst_access{vk::AccessFlagBits::eTransferRead};
    static constexpr vk::PipelineStageFlags src_stage{vk::PipelineStageFlagBits::eTransfer};
    static constexpr vk::PipelineStageFlags dst_stage{vk::PipelineStageFlagBits::eTransfer};
};

template<>
struct TransitionTraits<vk::ImageLayout::eTransferSrcOptimal, vk::ImageLayout::eShaderReadOnlyOptimal> {
    static constexpr vk::AccessFlags src_access{vk::AccessFlagBits::eTransferRead};
    static constexpr vk::AccessFlags dst_access{vk::AccessFlagBits::eShaderRead};
    static constexpr vk::PipelineStageFlags src_stage{vk::PipelineStageFlagBits::eTransfer};
    static constexpr vk::PipelineStageFlags dst_stage{vk::PipelineStageFlagBits::eFragmentShader};
};

template<>
struct TransitionTraits<vk::ImageLayout::eUndefined, vk::ImageLayout::eDepthAttachmentOptimal> {
    static constexpr vk::AccessFlags src_access{};
    static constexpr vk::AccessFlags dst_access{vk::AccessFlagBits::eDepthStencilAttachmentWrite | vk::AccessFlagBits::eDepthStencilAttachmentRead};
    static constexpr vk::PipelineStageFlags src_stage{vk::PipelineStageFlagBits::eTopOfPipe};
    static constexpr vk::PipelineStageFlags dst_stage{vk::PipelineStageFlagBits::eEarlyFragmentTests};
};

template<>
struct TransitionTraits<vk::ImageLayout::eTransferDstOptimal, vk::ImageLayout::eDepthAttachmentOptimal> {
    static constexpr vk::AccessFlags src_access{vk::AccessFlagBits::eTransferWrite};
    static constexpr vk::AccessFlags dst_access{vk::AccessFlagBits::eDepthStencilAttachmentWrite | vk::AccessFlagBits::eDepthStencilAttachmentRead};
    static constexpr vk::PipelineStageFlags src_stage{vk::PipelineStageFlagBits::eTransfer};
    static constexpr vk::PipelineStageFlags dst_stage{vk::PipelineStageFlagBits::eEarlyFragmentTests};
};

} // namespace detail

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

    template<vk::ImageLayout From, vk::ImageLayout To>
    using TransitionTraits = detail::TransitionTraits<From, To>;

    template<vk::ImageLayout From, vk::ImageLayout To>
    static LayoutTransitionConfig get_transition_config(vk::ImageAspectFlags aspect_mask) {
        using Traits = TransitionTraits<From, To>;
        LayoutTransitionConfig config{};
        config.old_layout = From;
        config.new_layout = To;
        config.aspect_mask = aspect_mask;
        config.src_access = Traits::src_access;
        config.dst_access = Traits::dst_access;
        config.src_stage = Traits::src_stage;
        config.dst_stage = Traits::dst_stage;
        return config;
    }

    static void copy_buffer_to_image(
        vk::CommandBuffer cmd, vk::Buffer src, vk::Image image,
        uint32_t width, uint32_t height,
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
        vk::CommandBuffer cmd, vk::Image image,
        const LayoutTransitionConfig& config,
        uint32_t base_mip_level = 0, uint32_t mip_levels = 1
    ) {
        vk::ImageMemoryBarrier barrier{};
        barrier.oldLayout = config.old_layout;
        barrier.newLayout = config.new_layout;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = image;
        barrier.subresourceRange.aspectMask = config.aspect_mask;
        barrier.subresourceRange.baseMipLevel = base_mip_level;
        barrier.subresourceRange.levelCount = mip_levels;
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

    static LayoutTransitionConfig make_transition_config(
        vk::ImageLayout old_layout,
        vk::ImageLayout new_layout,
        vk::ImageAspectFlags aspect_mask
    ) {
        if (old_layout == vk::ImageLayout::eUndefined &&
            new_layout == vk::ImageLayout::eTransferDstOptimal) {
            return get_transition_config<
                vk::ImageLayout::eUndefined,
                vk::ImageLayout::eTransferDstOptimal
            >(aspect_mask);
        } else if (old_layout == vk::ImageLayout::eTransferDstOptimal &&
                   new_layout == vk::ImageLayout::eShaderReadOnlyOptimal) {
            return get_transition_config<
                vk::ImageLayout::eTransferDstOptimal,
                vk::ImageLayout::eShaderReadOnlyOptimal
            >(aspect_mask);
        } else if (old_layout == vk::ImageLayout::eTransferDstOptimal &&
                   new_layout == vk::ImageLayout::eTransferSrcOptimal) {
            return get_transition_config<
                vk::ImageLayout::eTransferDstOptimal,
                vk::ImageLayout::eTransferSrcOptimal
            >(aspect_mask);
        } else if (old_layout == vk::ImageLayout::eTransferSrcOptimal &&
                   new_layout == vk::ImageLayout::eShaderReadOnlyOptimal) {
            return get_transition_config<
                vk::ImageLayout::eTransferSrcOptimal,
                vk::ImageLayout::eShaderReadOnlyOptimal
            >(aspect_mask);
        } else if (old_layout == vk::ImageLayout::eUndefined &&
                   new_layout == vk::ImageLayout::eDepthAttachmentOptimal) {
            return get_transition_config<
                vk::ImageLayout::eUndefined,
                vk::ImageLayout::eDepthAttachmentOptimal
            >(aspect_mask);
        } else if (old_layout == vk::ImageLayout::eTransferDstOptimal &&
                   new_layout == vk::ImageLayout::eDepthAttachmentOptimal) {
            return get_transition_config<
                vk::ImageLayout::eTransferDstOptimal,
                vk::ImageLayout::eDepthAttachmentOptimal
            >(aspect_mask);
        }

        throw std::invalid_argument("Unsupported layout transition!");
    }

    static Image create_depth_image(
        Device& device,
        uint32_t width, uint32_t height,
        vk::Format format
    ) {
        Image image(
            device,
            width, height,
            format,
            vk::ImageTiling::eOptimal,
            vk::ImageUsageFlagBits::eDepthStencilAttachment,
            vk::MemoryPropertyFlagBits::eDeviceLocal,
            vk::ImageAspectFlagBits::eDepth,
            false
        );
        return image;
    }

    static Image from_rgba8(
        Device& device,
        uint32_t width,
        uint32_t height,
        const uint8_t* rgba_data,
        std::size_t data_size,
        bool use_srgb = true,
        bool generate_mipmaps = true
    ) {
        if (width == 0 || height == 0) {
            throw std::invalid_argument("Image::create_image_from_rgba8 requires non-zero extent.");
        }
        if (rgba_data == nullptr || data_size == 0) {
            throw std::invalid_argument("Image::create_image_from_rgba8 requires non-empty pixel data.");
        }

        const std::size_t min_expected = static_cast<std::size_t>(width) *
                                         static_cast<std::size_t>(height) *
                                         4u;
        if (data_size < min_expected) {
            throw std::invalid_argument("Image::create_image_from_rgba8 pixel data size is smaller than width*height*4.");
        }

        auto stage_buffer = Buffer::create_host_visible_buffer(
            device,
            data_size,
            vk::BufferUsageFlagBits::eTransferSrc
        );

        stage_buffer.map();
        std::memcpy(stage_buffer.mapped_data(), rgba_data, data_size);
        stage_buffer.unmap();

        vk::Format format = use_srgb ? vk::Format::eR8G8B8A8Srgb : vk::Format::eR8G8B8A8Unorm;

        auto usage = vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eSampled;
        if (generate_mipmaps) {
            usage |= vk::ImageUsageFlagBits::eTransferSrc;
        }

        auto image = Image(
            device,
            width,
            height,
            format,
            vk::ImageTiling::eOptimal,
            usage,
            vk::MemoryPropertyFlagBits::eDeviceLocal,
            vk::ImageAspectFlagBits::eColor,
            generate_mipmaps
        );

        image.upload(stage_buffer, vk::ImageLayout::eShaderReadOnlyOptimal);
        return image;
    }

private:
    std::reference_wrapper<Device> m_device;
    vk::raii::Image m_image{nullptr};
    vk::raii::ImageView m_image_view{nullptr};
    vk::raii::DeviceMemory m_image_memory{nullptr};

    uint32_t m_width{0};
    uint32_t m_height{0};
    vk::Format m_format{vk::Format::eR8G8B8A8Unorm};
    vk::ImageAspectFlags m_aspect_mask{vk::ImageAspectFlagBits::eColor};
    vk::ImageUsageFlags m_usage{};
    vk::ImageLayout m_current_layout{vk::ImageLayout::eUndefined};
    uint32_t m_mip_levels{1};

public:
    Image(
        Device& device,
        uint32_t width, uint32_t height,
        vk::Format format, vk::ImageTiling tiling,
        vk::ImageUsageFlags usage, vk::MemoryPropertyFlags properties, vk::ImageAspectFlags aspect_mask,
        bool generate_mipmaps
    ) : m_device(device), m_width(width), m_height(height), m_format(format), m_aspect_mask(aspect_mask), m_usage(usage) {

        if (generate_mipmaps && (usage & vk::ImageUsageFlagBits::eSampled)) {
            m_mip_levels = static_cast<uint32_t>(std::floor(std::log2(std::max(width, height)))) + 1;
        } else {
            m_mip_levels = 1;
        }

        auto resources_opt = make_image_with_memory(
            device.device(),
            device.physical_device(),
            width, height, 
            m_mip_levels,
            format, tiling, usage, properties
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
        view_info.subresourceRange.levelCount = m_mip_levels;
        view_info.subresourceRange.baseArrayLayer = 0;
        view_info.subresourceRange.layerCount = 1;

        m_image_view = vk::raii::ImageView(m_device.get().device(), view_info);
    }

    Image(const Image&) = delete;
    Image& operator=(const Image&) = delete;

    Image(Image&& other) noexcept 
    : m_device(other.m_device.get()),
        m_image(std::move(other.m_image)),
        m_image_view(std::move(other.m_image_view)),
        m_image_memory(std::move(other.m_image_memory)),
        m_width(other.m_width),
        m_height(other.m_height),
        m_format(other.m_format),
        m_aspect_mask(other.m_aspect_mask),
        m_usage(other.m_usage),
        m_current_layout(other.m_current_layout),
        m_mip_levels(other.m_mip_levels)
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
            m_mip_levels = other.m_mip_levels;

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

    uint32_t mip_levels() const { return m_mip_levels; }

    void apply_transition(vk::CommandBuffer cmd, const LayoutTransitionConfig& config, int base_mip_level = 0, int mip_levels = 1) {
        transition_image_layout(cmd, *m_image, config, base_mip_level, mip_levels);
        m_current_layout = config.new_layout;
    }

private:
    void generate_mipmaps(CommandBuffer& cb) {
        auto cmd = *cb.command_buffer();
        auto format_properties = m_device.get().physical_device().getFormatProperties(m_format);
        if (!(format_properties.optimalTilingFeatures & vk::FormatFeatureFlagBits::eSampledImageFilterLinear)) {
            throw std::runtime_error("Texture image format does not support linear blitting!");
        }

        int32_t mip_width = static_cast<int32_t>(m_width);
        int32_t mip_height = static_cast<int32_t>(m_height);

        for (int i = 1; i < m_mip_levels; i++) {
            auto to_src = get_transition_config<
                vk::ImageLayout::eTransferDstOptimal,
                vk::ImageLayout::eTransferSrcOptimal
            >(m_aspect_mask);
            apply_transition(cmd, to_src, i - 1, 1);

            vk::ImageBlit blit{};
            blit.srcOffsets[0] = vk::Offset3D{0, 0, 0};
            blit.srcOffsets[1] = vk::Offset3D{mip_width, mip_height, 1};
            blit.srcSubresource.aspectMask = m_aspect_mask;
            blit.srcSubresource.mipLevel = i - 1;
            blit.srcSubresource.baseArrayLayer = 0;
            blit.srcSubresource.layerCount = 1;
            blit.dstOffsets[0] = vk::Offset3D{0, 0, 0};
            blit.dstOffsets[1] = vk::Offset3D{
                mip_width > 1 ? mip_width / 2 : 1,
                mip_height > 1 ? mip_height / 2 : 1,
                1
            };
            blit.dstSubresource.aspectMask = m_aspect_mask;
            blit.dstSubresource.mipLevel = i;
            blit.dstSubresource.baseArrayLayer = 0;
            blit.dstSubresource.layerCount = 1;

            cmd.blitImage(
                *m_image, vk::ImageLayout::eTransferSrcOptimal,
                *m_image, vk::ImageLayout::eTransferDstOptimal,
                blit,
                vk::Filter::eLinear
            );

            auto to_shader = get_transition_config<
                vk::ImageLayout::eTransferSrcOptimal,
                vk::ImageLayout::eShaderReadOnlyOptimal
            >(m_aspect_mask);
            apply_transition(cmd, to_shader, i - 1, 1);

            if (mip_width > 1) mip_width /= 2;
            if (mip_height > 1) mip_height /= 2;
        }

        auto last_to_shader = get_transition_config<
            vk::ImageLayout::eTransferDstOptimal,
            vk::ImageLayout::eShaderReadOnlyOptimal
        >(m_aspect_mask);
        apply_transition(cmd, last_to_shader, m_mip_levels - 1, 1);
        m_current_layout = vk::ImageLayout::eShaderReadOnlyOptimal;
    }

    void upload(Buffer& stage_buffer, vk::ImageLayout final_layout) {
        CommandPool command_pool(m_device.get(), vk::CommandPoolCreateFlagBits::eTransient);
        auto cmd = command_pool.create_command_buffer();
        vk::raii::Fence upload_fence(m_device.get().device(), vk::FenceCreateInfo{});
        CommandBuffer::SubmitInfo submit_info{};
        submit_info.fence = *upload_fence;
        // Transition image layout and copy buffer to image
        cmd.record_and_submit([&](CommandBuffer& cb){

            LayoutTransitionConfig to_transfer{};
            if (m_current_layout == vk::ImageLayout::eUndefined) {
                to_transfer = get_transition_config<
                    vk::ImageLayout::eUndefined,
                    vk::ImageLayout::eTransferDstOptimal
                >(m_aspect_mask);
            } else {
                to_transfer = make_transition_config(
                    m_current_layout,
                    vk::ImageLayout::eTransferDstOptimal,
                    m_aspect_mask
                );
            }
            // Transition all mip levels so subsequent blits have a valid dst layout
            apply_transition(*cb.command_buffer(), to_transfer, 0, m_mip_levels);

            copy_buffer_to_image(
                *cb.command_buffer(),
                *stage_buffer.buffer(),
                *m_image,
                m_width,
                m_height,
                m_aspect_mask
            );

            if (m_mip_levels > 1) {
                generate_mipmaps(cb);
            }  else {
                LayoutTransitionConfig to_shader{};
                if (final_layout == vk::ImageLayout::eShaderReadOnlyOptimal) {
                    to_shader = get_transition_config<
                        vk::ImageLayout::eTransferDstOptimal,
                        vk::ImageLayout::eShaderReadOnlyOptimal
                    >(m_aspect_mask);
                } else {
                    to_shader = make_transition_config(
                        vk::ImageLayout::eTransferDstOptimal,
                        final_layout,
                        m_aspect_mask
                    );
                }
                apply_transition(*cb.command_buffer(), to_shader, 0, m_mip_levels);
            }
        }, submit_info);

        
        auto result = m_device.get().device().waitForFences(*upload_fence, VK_TRUE, UINT64_MAX);
        if (result != vk::Result::eSuccess) {
            throw std::runtime_error("Failed to wait for image upload fence!");
        }

    }
};

class Sampler {
private:
    std::reference_wrapper<Device> m_device;
    vk::raii::Sampler m_sampler{nullptr};

public:
    // =========================================================================
    // 静态工厂方法：创建一个标准的、高质量的采样器
    // 特性：线性过滤 (Linear), 重复平铺 (Repeat), 开启最大各向异性过滤 (Anisotropy)
    // =========================================================================
    static Sampler create_default(Device& device, uint32_t mip_levels = 1) {
        // 1. 获取物理设备属性，用于查询支持的最大各向异性过滤级别
        vk::PhysicalDeviceProperties properties = device.physical_device().getProperties();

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
        info.mipmapMode = vk::SamplerMipmapMode::eLinear;
        info.mipLodBias = 0.0f;
        info.minLod = 0.0f;
        info.maxLod = static_cast<float>(mip_levels - 1);

        return Sampler(device, info);
    }

    // 工厂方法：创建一个像素风采样器 (Nearest, Clamp)
    static Sampler create_pixel_art_style(Device& device) {
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
    Sampler(Device& device, const vk::SamplerCreateInfo& create_info) 
        : m_device(device) {
        m_sampler = vk::raii::Sampler(device.device(), create_info);
    }

    // Rule of Five: 禁用拷贝，允许移动
    Sampler(const Sampler&) = delete;
    Sampler& operator=(const Sampler&) = delete;

    Sampler(Sampler&& other) noexcept 
        : m_device(other.m_device.get()), m_sampler(std::move(other.m_sampler)) {}

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

} // namespace rtr::rhi
