#pragma once

#include <memory>

#include "framework/component/component.hpp"
#include "framework/component/node.hpp"

namespace rtr::framework::component {

class NodeComponent final : public Component {
private:
    std::shared_ptr<Node> m_node{};

public:
    NodeComponent() = default;
    ~NodeComponent() override = default;

    void on_awake() override {
        m_node = Node::create();
    }

    void on_destroy() override {
        m_node.reset();
    }

    const std::shared_ptr<Node>& node() const {
        return m_node;
    }

    std::shared_ptr<Node> node() {
        return m_node;
    }
};

} // namespace rtr::framework::component
