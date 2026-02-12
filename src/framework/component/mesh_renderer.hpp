#pragma once

#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "framework/component/component.hpp"

namespace rtr::framework::component {

class MeshRenderer final : public Component {
public:
    static constexpr std::string_view kDefaultAlbedoCheckerboardPath =
        "assets/textures/default_checkerboard_512.png";

private:
    std::string m_mesh_path{};
    std::string m_albedo_texture_path{};

    static std::string normalize_albedo_path(std::string albedo_texture_path) {
        if (albedo_texture_path.empty()) {
            return std::string{kDefaultAlbedoCheckerboardPath};
        }
        return albedo_texture_path;
    }

public:
    explicit MeshRenderer(
        std::string mesh_path,
        std::string albedo_texture_path = ""
    )
        : m_mesh_path(std::move(mesh_path)),
          m_albedo_texture_path(normalize_albedo_path(std::move(albedo_texture_path))) {
        if (m_mesh_path.empty()) {
            throw std::invalid_argument("MeshRenderer mesh_path must not be empty.");
        }
    }

    const std::string& mesh_path() const {
        return m_mesh_path;
    }

    void set_mesh_path(std::string mesh_path) {
        if (mesh_path.empty()) {
            throw std::invalid_argument("MeshRenderer mesh_path must not be empty.");
        }
        m_mesh_path = std::move(mesh_path);
    }

    const std::string& albedo_texture_path() const {
        return m_albedo_texture_path;
    }

    void set_albedo_texture_path(std::string albedo_texture_path) {
        m_albedo_texture_path = normalize_albedo_path(std::move(albedo_texture_path));
    }

    void reset_albedo_to_default() {
        m_albedo_texture_path = std::string{kDefaultAlbedoCheckerboardPath};
    }
};

} // namespace rtr::framework::component
