#include <memory>
#include <type_traits>

#include "gtest/gtest.h"

#include "rtr/editor/core/editor_capture.hpp"
#include "rtr/editor/core/editor_host.hpp"
#include "rtr/system/render/pipeline/forward/forward_pipeline.hpp"
#include "rtr/system/render/pipeline/shadertoy/shadertoy_pipeline.hpp"

// Test removed since EditorOverlayPipeline and bind_input_capture_to_editor were removed.
// We will test editor interactions during Phase 3 after ForwardEditorPipeline is cleanly assembled.
#if 0
namespace rtr::editor::test {
// ... old test code ...
}
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#endif
