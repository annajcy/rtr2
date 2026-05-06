#pragma once

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <utility>
#include <vector>

#include "rtr/rhi/buffer.hpp"
#include "rtr/rhi/command.hpp"
#include "rtr/rhi/device.hpp"
#include "rtr/utils/image_io.hpp"

namespace rtr::rhi {

struct ReadbackImageDesc {
    vk::Image image{};
    vk::Format format{vk::Format::eUndefined};
    vk::Extent2D extent{};
    vk::ImageLayout current_layout{vk::ImageLayout::eUndefined};
};

struct ReadbackResult {
    std::vector<std::uint8_t> pixels_rgba8;
    uint32_t width{0};
    uint32_t height{0};
};

namespace detail {

struct PendingImageReadback {
    Buffer staging_buffer;
    vk::Format format{vk::Format::eUndefined};
    vk::Extent2D extent{};

    PendingImageReadback(Buffer&& staging_buffer_in, vk::Format format_in, vk::Extent2D extent_in)
        : staging_buffer(std::move(staging_buffer_in)),
          format(format_in),
          extent(extent_in) {}
};

inline std::uint32_t bytes_per_pixel(vk::Format format) {
    switch (format) {
    case vk::Format::eR8G8B8A8Unorm:
    case vk::Format::eR8G8B8A8Srgb:
    case vk::Format::eB8G8R8A8Unorm:
    case vk::Format::eB8G8R8A8Srgb:
        return 4;
    default:
        throw std::runtime_error("Unsupported readback image format.");
    }
}

inline std::pair<vk::PipelineStageFlags2, vk::AccessFlags2> layout_stage_access(vk::ImageLayout layout) {
    switch (layout) {
    case vk::ImageLayout::eUndefined:
        return {vk::PipelineStageFlagBits2::eTopOfPipe, vk::AccessFlagBits2::eNone};
    case vk::ImageLayout::eColorAttachmentOptimal:
        return {vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eColorAttachmentWrite};
    case vk::ImageLayout::eShaderReadOnlyOptimal:
        return {vk::PipelineStageFlagBits2::eFragmentShader, vk::AccessFlagBits2::eShaderRead};
    case vk::ImageLayout::eGeneral:
        return {
            vk::PipelineStageFlagBits2::eAllCommands,
            vk::AccessFlagBits2::eShaderStorageRead | vk::AccessFlagBits2::eShaderStorageWrite
        };
    case vk::ImageLayout::eTransferDstOptimal:
        return {vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferWrite};
    case vk::ImageLayout::eTransferSrcOptimal:
        return {vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferRead};
    default:
        return {vk::PipelineStageFlagBits2::eAllCommands, vk::AccessFlagBits2::eMemoryRead | vk::AccessFlagBits2::eMemoryWrite};
    }
}

inline PendingImageReadback record_readback_copy(
    Device& device,
    CommandBuffer& command_buffer,
    const ReadbackImageDesc& desc
) {
    if (desc.image == vk::Image{}) {
        throw std::invalid_argument("ReadbackImageDesc.image must be valid.");
    }
    if (desc.extent.width == 0 || desc.extent.height == 0) {
        throw std::invalid_argument("ReadbackImageDesc.extent must be non-zero.");
    }

    const auto bytes_per_px = bytes_per_pixel(desc.format);
    const vk::DeviceSize total_size =
        static_cast<vk::DeviceSize>(desc.extent.width) *
        static_cast<vk::DeviceSize>(desc.extent.height) *
        static_cast<vk::DeviceSize>(bytes_per_px);

    PendingImageReadback pending{
        Buffer::create_host_visible_buffer(
            device,
            total_size,
            vk::BufferUsageFlagBits::eTransferDst
        ),
        desc.format,
        desc.extent
    };

    auto& cmd = command_buffer.command_buffer();
    const auto [src_stage, src_access] = layout_stage_access(desc.current_layout);
    if (desc.current_layout != vk::ImageLayout::eTransferSrcOptimal) {
        vk::ImageMemoryBarrier2 to_transfer{};
        to_transfer.srcStageMask = src_stage;
        to_transfer.dstStageMask = vk::PipelineStageFlagBits2::eTransfer;
        to_transfer.srcAccessMask = src_access;
        to_transfer.dstAccessMask = vk::AccessFlagBits2::eTransferRead;
        to_transfer.oldLayout = desc.current_layout;
        to_transfer.newLayout = vk::ImageLayout::eTransferSrcOptimal;
        to_transfer.image = desc.image;
        to_transfer.subresourceRange = {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};

        vk::DependencyInfo dependency{};
        dependency.imageMemoryBarrierCount = 1;
        dependency.pImageMemoryBarriers = &to_transfer;
        cmd.pipelineBarrier2(dependency);
    }

    vk::BufferImageCopy copy_region{};
    copy_region.bufferOffset = 0;
    copy_region.bufferRowLength = 0;
    copy_region.bufferImageHeight = 0;
    copy_region.imageSubresource = {vk::ImageAspectFlagBits::eColor, 0, 0, 1};
    copy_region.imageOffset = vk::Offset3D{0, 0, 0};
    copy_region.imageExtent = vk::Extent3D{desc.extent.width, desc.extent.height, 1};
    cmd.copyImageToBuffer(
        desc.image,
        vk::ImageLayout::eTransferSrcOptimal,
        *pending.staging_buffer.buffer(),
        copy_region
    );

    if (desc.current_layout != vk::ImageLayout::eTransferSrcOptimal) {
        const auto [dst_stage, dst_access] = layout_stage_access(desc.current_layout);
        vk::ImageMemoryBarrier2 restore{};
        restore.srcStageMask = vk::PipelineStageFlagBits2::eTransfer;
        restore.dstStageMask = dst_stage;
        restore.srcAccessMask = vk::AccessFlagBits2::eTransferRead;
        restore.dstAccessMask = dst_access;
        restore.oldLayout = vk::ImageLayout::eTransferSrcOptimal;
        restore.newLayout = desc.current_layout;
        restore.image = desc.image;
        restore.subresourceRange = {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};

        vk::DependencyInfo dependency{};
        dependency.imageMemoryBarrierCount = 1;
        dependency.pImageMemoryBarriers = &restore;
        cmd.pipelineBarrier2(dependency);
    }

    return pending;
}

inline ReadbackResult finalize_readback_copy(PendingImageReadback&& pending) {
    pending.staging_buffer.map();
    auto* mapped = static_cast<const std::uint8_t*>(pending.staging_buffer.mapped_data());
    const auto pixel_count =
        static_cast<std::size_t>(pending.extent.width) * static_cast<std::size_t>(pending.extent.height);

    ReadbackResult result{};
    result.width = pending.extent.width;
    result.height = pending.extent.height;
    result.pixels_rgba8.resize(pixel_count * 4u);

    switch (pending.format) {
    case vk::Format::eR8G8B8A8Unorm:
    case vk::Format::eR8G8B8A8Srgb:
        std::copy(
            mapped,
            mapped + result.pixels_rgba8.size(),
            result.pixels_rgba8.begin()
        );
        break;
    case vk::Format::eB8G8R8A8Unorm:
    case vk::Format::eB8G8R8A8Srgb:
        for (std::size_t i = 0; i < pixel_count; ++i) {
            const std::size_t src = i * 4u;
            result.pixels_rgba8[src + 0u] = mapped[src + 2u];
            result.pixels_rgba8[src + 1u] = mapped[src + 1u];
            result.pixels_rgba8[src + 2u] = mapped[src + 0u];
            result.pixels_rgba8[src + 3u] = mapped[src + 3u];
        }
        break;
    default:
        pending.staging_buffer.unmap();
        throw std::runtime_error("Unsupported readback image format.");
    }

    pending.staging_buffer.unmap();
    return result;
}

}  // namespace detail

inline ReadbackResult readback_image(
    Device& device,
    CommandBuffer& command_buffer,
    const ReadbackImageDesc& desc
) {
    auto pending = detail::record_readback_copy(device, command_buffer, desc);
    command_buffer.submit();
    device.device().waitIdle();
    return detail::finalize_readback_copy(std::move(pending));
}

inline void save_png(
    const std::filesystem::path& output_path,
    const ReadbackResult& image
) {
    utils::ImageData png{};
    png.width = image.width;
    png.height = image.height;
    png.channels = 4;
    png.pixels = image.pixels_rgba8;
    utils::write_image_to_path(png, output_path.string());
}

}  // namespace rtr::rhi
