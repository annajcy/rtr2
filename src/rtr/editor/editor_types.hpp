#pragma once

#include <cstdint>
#include <functional>

#include "rtr/framework/core/types.hpp"

namespace rtr::editor {

struct EditorSelection {
    framework::core::SceneId scene_id{framework::core::kInvalidSceneId};
    framework::core::GameObjectId game_object_id{framework::core::kInvalidGameObjectId};

    bool has_game_object() const {
        return scene_id != framework::core::kInvalidSceneId &&
            game_object_id != framework::core::kInvalidGameObjectId;
    }

    void clear() {
        scene_id = framework::core::kInvalidSceneId;
        game_object_id = framework::core::kInvalidGameObjectId;
    }
};

struct EditorFrameData {
    std::uint64_t frame_serial{0};
    double delta_seconds{0.0};
    bool paused{false};
};

struct EditorServices {
    std::function<void(framework::core::GameObjectId)> request_focus_game_object{};
};

} // namespace rtr::editor

