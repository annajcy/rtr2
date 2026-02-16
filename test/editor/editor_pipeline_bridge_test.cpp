#include <memory>
#include <type_traits>

#include "gtest/gtest.h"

#include "rtr/editor/editor_attach.hpp"
#include "rtr/system/render/frame_color_source.hpp"
#include "rtr/system/render/pipeline/forward/forward_pipeline.hpp"
#include "rtr/system/render/pipeline/shadertoy/shadertoy_pipeline.hpp"

namespace rtr::editor::test {

namespace {

class FakeCaptureSource final : public IEditorInputCaptureSource {
private:
    bool m_mouse_capture{false};
    bool m_keyboard_capture{false};

public:
    void set_capture(bool mouse, bool keyboard) {
        m_mouse_capture = mouse;
        m_keyboard_capture = keyboard;
    }

    bool wants_imgui_capture_mouse() const override {
        return m_mouse_capture;
    }

    bool wants_imgui_capture_keyboard() const override {
        return m_keyboard_capture;
    }
};

class DummyRenderPipeline final : public system::render::IRenderPipeline {
public:
    void render(system::render::FrameContext& /*ctx*/) override {}
};

} // namespace

TEST(EditorPipelineBridgeTest, ForwardAndShaderToyExposeFrameColorSourceInterfaces) {
    EXPECT_TRUE(
        (std::is_base_of_v<system::render::IFrameColorSource, system::render::ForwardPipeline>)
    );
    EXPECT_TRUE(
        (std::is_base_of_v<system::render::IFrameColorSource, system::render::ShaderToyPipeline>)
    );

    EXPECT_TRUE(
        (std::is_base_of_v<system::render::ISceneViewportSink, system::render::ForwardPipeline>)
    );
    EXPECT_TRUE(
        (std::is_base_of_v<system::render::ISceneViewportSink, system::render::ShaderToyPipeline>)
    );
}

TEST(EditorPipelineBridgeTest, CreateEditorPipelineRejectsNullInputs) {
    const system::render::PipelineRuntime runtime{};
    auto host = std::make_shared<EditorHost>();

    EXPECT_THROW(
        create_editor_pipeline(runtime, nullptr, host),
        std::invalid_argument
    );

    auto runtime_pipeline = std::make_unique<DummyRenderPipeline>();
    EXPECT_THROW(
        create_editor_pipeline(runtime, std::move(runtime_pipeline), nullptr),
        std::invalid_argument
    );
}

TEST(EditorPipelineBridgeTest, InputCaptureBindingUsesEditorCaptureSourceQuery) {
    FakeCaptureSource source;

    system::input::InputSystem::RawEventSource event_source{};
    system::input::InputSystem input(event_source);
    bind_input_capture_to_editor(input, source);

    source.set_capture(true, false);
    input.handle_mouse_move_raw(100.0, 100.0);
    EXPECT_DOUBLE_EQ(input.state().mouse_x(), 0.0);
    EXPECT_DOUBLE_EQ(input.state().mouse_y(), 0.0);

    source.set_capture(false, false);
    input.handle_mouse_move_raw(100.0, 100.0);
    EXPECT_DOUBLE_EQ(input.state().mouse_x(), 100.0);
    EXPECT_DOUBLE_EQ(input.state().mouse_y(), 100.0);
}

} // namespace rtr::editor::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
