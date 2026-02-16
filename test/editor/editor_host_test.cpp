#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rtr/editor/editor_host.hpp"
#include "rtr/editor/editor_types.hpp"

namespace rtr::editor::test {

namespace {

class ProbePanel final : public IEditorPanel {
private:
    std::string m_id{};
    int m_order{0};
    bool m_visible{true};
    std::vector<std::string>* m_log{};

public:
    ProbePanel(std::string id, int order, std::vector<std::string>* log, bool visible = true)
        : m_id(std::move(id)), m_order(order), m_visible(visible), m_log(log) {}

    std::string_view id() const override {
        return m_id;
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

    void on_frame(EditorContext& /*ctx*/) override {
        if (m_log) {
            m_log->emplace_back("frame:" + m_id);
        }
    }

    void on_imgui(EditorContext& /*ctx*/) override {
        if (m_log) {
            m_log->emplace_back("imgui:" + m_id);
        }
    }
};

} // namespace

TEST(EditorHostTest, OrdersPanelsByOrderThenId) {
    EditorHost host;
    std::vector<std::string> log{};

    host.register_panel(std::make_unique<ProbePanel>("z", 20, &log));
    host.register_panel(std::make_unique<ProbePanel>("a", 10, &log));
    host.register_panel(std::make_unique<ProbePanel>("b", 20, &log));

    host.begin_frame(EditorFrameData{});
    host.draw_imgui();

    ASSERT_EQ(log.size(), 6u);
    EXPECT_EQ(log[0], "frame:a");
    EXPECT_EQ(log[1], "frame:b");
    EXPECT_EQ(log[2], "frame:z");
    EXPECT_EQ(log[3], "imgui:a");
    EXPECT_EQ(log[4], "imgui:b");
    EXPECT_EQ(log[5], "imgui:z");
}

TEST(EditorHostTest, SkipsInvisiblePanelsInFrameAndImgui) {
    EditorHost host;
    std::vector<std::string> log{};

    host.register_panel(std::make_unique<ProbePanel>("visible", 0, &log, true));
    host.register_panel(std::make_unique<ProbePanel>("hidden", 1, &log, false));

    host.begin_frame(EditorFrameData{});
    host.draw_imgui();

    ASSERT_EQ(log.size(), 2u);
    EXPECT_EQ(log[0], "frame:visible");
    EXPECT_EQ(log[1], "imgui:visible");
}

TEST(EditorHostTest, CanRemovePanelById) {
    EditorHost host;
    std::vector<std::string> log{};

    host.register_panel(std::make_unique<ProbePanel>("alpha", 0, &log));
    host.register_panel(std::make_unique<ProbePanel>("beta", 1, &log));

    EXPECT_TRUE(host.remove_panel("alpha"));
    EXPECT_FALSE(host.remove_panel("missing"));

    host.begin_frame(EditorFrameData{});
    host.draw_imgui();

    ASSERT_EQ(log.size(), 2u);
    EXPECT_EQ(log[0], "frame:beta");
    EXPECT_EQ(log[1], "imgui:beta");
}

TEST(EditorHostTest, RejectsDuplicatePanelId) {
    EditorHost host;
    std::vector<std::string> log{};
    host.register_panel(std::make_unique<ProbePanel>("dup", 0, &log));

    EXPECT_THROW(
        host.register_panel(std::make_unique<ProbePanel>("dup", 1, &log)),
        std::runtime_error
    );
}

TEST(EditorHostTest, CanTogglePanelVisibilityById) {
    EditorHost host;
    std::vector<std::string> log{};
    host.register_panel(std::make_unique<ProbePanel>("inspector", 0, &log, true));

    ASSERT_TRUE(host.panel_visible("inspector").has_value());
    EXPECT_TRUE(host.panel_visible("inspector").value());

    EXPECT_TRUE(host.set_panel_visible("inspector", false));
    ASSERT_TRUE(host.panel_visible("inspector").has_value());
    EXPECT_FALSE(host.panel_visible("inspector").value());

    EXPECT_TRUE(host.set_panel_visible("inspector", true));
    ASSERT_TRUE(host.panel_visible("inspector").has_value());
    EXPECT_TRUE(host.panel_visible("inspector").value());
}

TEST(EditorHostTest, ReturnsMissingForUnknownPanelVisibility) {
    EditorHost host;
    std::vector<std::string> log{};
    host.register_panel(std::make_unique<ProbePanel>("known", 0, &log, true));

    EXPECT_FALSE(host.panel_visible("missing").has_value());
    EXPECT_FALSE(host.set_panel_visible("missing", true));

    host.reset_layout();
}

} // namespace rtr::editor::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

