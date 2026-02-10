#pragma once

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "render/frame_context.hpp"

namespace rtr::render {

enum class ResourceAccess {
    eRead,
    eWrite,
    eReadWrite
};

struct ResourceDependency {
    std::string resource_name;
    ResourceAccess access{ResourceAccess::eRead};
};

class IPassResources {
public:
    virtual ~IPassResources() = default;
};

class EmptyPassResources final : public IPassResources {};

class IRenderPass {
public:
    virtual ~IRenderPass() = default;
    virtual std::string_view name() const = 0;
    virtual const std::vector<ResourceDependency>& dependencies() const = 0;
    virtual std::unique_ptr<IPassResources> create_resources() const = 0;
    virtual void execute(render::FrameContext& ctx, IPassResources& resources) = 0;
};

class CallbackPass final : public IRenderPass {
private:
    std::string m_name;
    std::vector<ResourceDependency> m_dependencies;
    std::function<void(render::FrameContext&, IPassResources&)> m_execute_fn;

public:
    CallbackPass(
        std::string name,
        std::vector<ResourceDependency> dependencies,
        std::function<void(render::FrameContext&, IPassResources&)> execute_fn
    )
        : m_name(std::move(name)),
          m_dependencies(std::move(dependencies)),
          m_execute_fn(std::move(execute_fn)) {}

    std::string_view name() const override { return m_name; }
    const std::vector<ResourceDependency>& dependencies() const override { return m_dependencies; }

    std::unique_ptr<IPassResources> create_resources() const override {
        return std::make_unique<EmptyPassResources>();
    }

    void execute(render::FrameContext& ctx, IPassResources& resources) override {
        m_execute_fn(ctx, resources);
    }
};

class RenderGraph {
private:
    struct PassNode {
        std::unique_ptr<IRenderPass> pass;
        std::unique_ptr<IPassResources> resources;
    };

    std::vector<PassNode> m_pass_nodes;

public:
    RenderGraph() = default;

    RenderGraph(const RenderGraph&) = delete;
    RenderGraph& operator=(const RenderGraph&) = delete;

    void add_pass(std::unique_ptr<IRenderPass> pass) {
        if (!pass) {
            throw std::runtime_error("Cannot add null render pass.");
        }
        auto resources = pass->create_resources();
        if (!resources) {
            throw std::runtime_error("Render pass returned null resource object.");
        }
        m_pass_nodes.push_back(PassNode{
            .pass = std::move(pass),
            .resources = std::move(resources)
        });
    }

    std::vector<const IRenderPass*> passes() const {
        std::vector<const IRenderPass*> out;
        out.reserve(m_pass_nodes.size());
        for (const auto& node : m_pass_nodes) {
            out.push_back(node.pass.get());
        }
        return out;
    }
};

} // namespace rtr::render
