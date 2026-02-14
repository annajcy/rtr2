#include <memory>
#include <type_traits>

#include "gtest/gtest.h"

#include "rtr/editor/editor_attach.hpp"
#include "rtr/system/render/pipeline/forward/forward_pipeline.hpp"
#include "rtr/system/render/pipeline/shadertoy/shadertoy_pipeline.hpp"

namespace rtr::editor::test {

namespace {

class DummyOverlay final : public system::render::IImGuiOverlay {
public:
    void draw_imgui() override {}
};

class FakeOverlayPipeline final : public system::render::IRenderPipeline,
                                  public system::render::IImGuiOverlayPipeline {
private:
    std::shared_ptr<system::render::IImGuiOverlay> m_overlay{};
    bool m_mouse_capture{false};
    bool m_keyboard_capture{false};

public:
    void render(system::render::FrameContext& /*ctx*/) override {}

    void set_imgui_overlay(std::shared_ptr<system::render::IImGuiOverlay> overlay) override {
        m_overlay = std::move(overlay);
    }

    void clear_imgui_overlay() override {
        m_overlay.reset();
    }

    bool wants_imgui_capture_mouse() const override {
        return m_mouse_capture;
    }

    bool wants_imgui_capture_keyboard() const override {
        return m_keyboard_capture;
    }

    bool has_overlay() const {
        return static_cast<bool>(m_overlay);
    }

    void set_capture(bool mouse, bool keyboard) {
        m_mouse_capture = mouse;
        m_keyboard_capture = keyboard;
    }
};

} // namespace

TEST(EditorPipelineBridgeTest, ForwardAndShaderToyImplementOverlayPipeline) {
    EXPECT_TRUE(
        (std::is_base_of_v<system::render::IImGuiOverlayPipeline, system::render::ForwardPipeline>)
    );
    EXPECT_TRUE(
        (std::is_base_of_v<system::render::IImGuiOverlayPipeline, system::render::ShaderToyPipeline>)
    );
}

TEST(EditorPipelineBridgeTest, AttachAndDetachEditorHostThroughBridgeHelpers) {
    FakeOverlayPipeline pipeline;
    auto host = std::make_shared<EditorHost>();

    attach_editor_host(pipeline, host);
    EXPECT_TRUE(pipeline.has_overlay());

    detach_editor_host(pipeline);
    EXPECT_FALSE(pipeline.has_overlay());
}

TEST(EditorPipelineBridgeTest, RequireOverlayPipelineRejectsNonOverlayPipeline) {
    class NonOverlayPipeline final : public system::render::IRenderPipeline {
    public:
        void render(system::render::FrameContext& /*ctx*/) override {}
    } pipeline;

    EXPECT_THROW(require_imgui_overlay_pipeline(pipeline), std::runtime_error);
}

TEST(EditorPipelineBridgeTest, InputCaptureBindingUsesOverlayPipelineQuery) {
    FakeOverlayPipeline pipeline;

    system::input::InputSystem::RawEventSource source{};
    system::input::InputSystem input(source);
    bind_input_capture_to_pipeline(input, pipeline);

    pipeline.set_capture(true, false);
    input.handle_mouse_move_raw(100.0, 100.0);
    EXPECT_DOUBLE_EQ(input.state().mouse_x(), 0.0);
    EXPECT_DOUBLE_EQ(input.state().mouse_y(), 0.0);

    pipeline.set_capture(false, false);
    input.handle_mouse_move_raw(100.0, 100.0);
    EXPECT_DOUBLE_EQ(input.state().mouse_x(), 100.0);
    EXPECT_DOUBLE_EQ(input.state().mouse_y(), 100.0);
}

} // namespace rtr::editor::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

