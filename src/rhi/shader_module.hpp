#pragma once

#include <string>
#include <vector>

#include "rhi/device.hpp"
#include "utils/file_loder.hpp"
#include "vulkan/vulkan_raii.hpp"

namespace rtr::rhi {

class ShaderModule {
private:
    Device* m_device{};
    vk::raii::ShaderModule m_module{nullptr};
    vk::ShaderStageFlagBits m_stage{};
    std::string m_entry_point{"main"};

public:
    static ShaderModule from_file(
        Device* device,
        const std::string& filepath,
        vk::ShaderStageFlagBits stage,
        const std::string& entry_point = "main"
    ) {
        auto code = rtr::utils::read_file(filepath);
        return ShaderModule(device, code, stage, entry_point);
    }

    ShaderModule(
        Device* device,
        const std::vector<char>& code,
        vk::ShaderStageFlagBits stage,
        const std::string& entry_point = "main"
    )
        : m_device(device), m_stage(stage), m_entry_point(entry_point) {
        vk::ShaderModuleCreateInfo create_info{};
        create_info.codeSize = code.size();
        create_info.pCode = reinterpret_cast<const uint32_t*>(code.data());

        m_module = vk::raii::ShaderModule(device->device(), create_info);
    }

    vk::PipelineShaderStageCreateInfo stage_create_info() const {
        vk::PipelineShaderStageCreateInfo stage_info{};
        stage_info.stage = m_stage;
        stage_info.module = *m_module;
        stage_info.pName = m_entry_point.c_str();
        return stage_info;
    }

    const vk::raii::ShaderModule& module() const { return m_module; }
    const std::string& entry_point() const { return m_entry_point; }
    vk::ShaderStageFlagBits stage() const { return m_stage; }
};

} // namespace rtr::rhi
