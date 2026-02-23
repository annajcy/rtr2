#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include "imgui.h"

#include "rtr/editor/core/editor_panel.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/scene_graph.hpp"
#include "rtr/framework/core/types.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::editor {

class HierarchyPanel final : public IEditorPanel {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("editor.panel.hierarchy");
    }

    bool m_visible{true};
    int m_order{100};

    static const framework::core::SceneGraph::NodeSnapshot* find_node_snapshot(
        const std::unordered_map<framework::core::GameObjectId, const framework::core::SceneGraph::NodeSnapshot*>& node_map,
        framework::core::GameObjectId id
    ) {
        const auto it = node_map.find(id);
        if (it == node_map.end()) {
            return nullptr;
        }
        return it->second;
    }

    static const char* find_name_or_default(
        const std::unordered_map<framework::core::GameObjectId, std::string>& names,
        framework::core::GameObjectId id
    ) {
        const auto it = names.find(id);
        if (it == names.end()) {
            return "GameObject";
        }
        return it->second.c_str();
    }

    void draw_node_recursive(
        EditorContext& ctx,
        framework::core::SceneId scene_id,
        framework::core::GameObjectId node_id,
        const std::unordered_map<framework::core::GameObjectId, const framework::core::SceneGraph::NodeSnapshot*>& node_map,
        const std::unordered_map<framework::core::GameObjectId, std::string>& names
    ) const {
        const auto* node = find_node_snapshot(node_map, node_id);
        if (node == nullptr) {
            return;
        }

        ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow |
            ImGuiTreeNodeFlags_OpenOnDoubleClick |
            ImGuiTreeNodeFlags_SpanAvailWidth;

        if (ctx.selection().has_game_object() &&
            ctx.selection().scene_id == scene_id &&
            ctx.selection().game_object_id == node_id) {
            flags |= ImGuiTreeNodeFlags_Selected;
        }

        const bool is_leaf = node->children.empty();
        if (is_leaf) {
            flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
        }

        const std::string label = std::string(find_name_or_default(names, node_id)) + "##" + std::to_string(node_id);
        const bool open = ImGui::TreeNodeEx(
            reinterpret_cast<void*>(static_cast<std::uintptr_t>(node_id)),
            flags,
            "%s",
            label.c_str()
        );

        if (ImGui::IsItemClicked()) {
            const bool selection_changed =
                !ctx.selection().has_game_object() ||
                ctx.selection().scene_id != scene_id ||
                ctx.selection().game_object_id != node_id;
            ctx.set_selection(scene_id, node_id);
            if (selection_changed) {
                logger()->debug(
                    "Hierarchy selected node (scene_id={}, game_object_id={}, name='{}').",
                    scene_id,
                    node_id,
                    find_name_or_default(names, node_id)
                );
            }
        }

        if (!is_leaf && open) {
            for (const auto child_id : node->children) {
                draw_node_recursive(ctx, scene_id, child_id, node_map, names);
            }
            ImGui::TreePop();
        }
    }

public:
    std::string_view id() const override {
        return "hierarchy";
    }

    int order() const override {
        return m_order;
    }

    bool visible() const override {
        return m_visible;
    }

    void set_visible(bool visible) override {
        m_visible = visible;
    }

    void on_imgui(EditorContext& ctx) override {
        if (!m_visible) {
            return;
        }

        if (!ImGui::Begin("Hierarchy", &m_visible)) {
            ImGui::End();
            return;
        }

        auto* scene = ctx.world().active_scene();
        if (scene == nullptr) {
            ImGui::TextDisabled("No active scene.");
            ImGui::End();
            return;
        }

        const auto snapshot = scene->scene_graph().to_snapshot();

        std::unordered_map<framework::core::GameObjectId, const framework::core::SceneGraph::NodeSnapshot*> node_map{};
        node_map.reserve(snapshot.nodes.size());
        for (const auto& node : snapshot.nodes) {
            node_map.emplace(node.id, &node);
        }

        std::unordered_map<framework::core::GameObjectId, std::string> names{};
        names.reserve(scene->game_objects().size());
        for (const auto& game_object : scene->game_objects()) {
            if (game_object) {
                names.emplace(game_object->id(),
                              std::string(scene->game_object_name(game_object->id()).value_or("GameObject")));
            }
        }

        for (const auto root_id : snapshot.root_children) {
            draw_node_recursive(ctx, scene->id(), root_id, node_map, names);
        }

        if (snapshot.root_children.empty()) {
            ImGui::TextDisabled("Scene is empty.");
        }

        ImGui::End();
    }
};

} // namespace rtr::editor
