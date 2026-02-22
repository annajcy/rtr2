#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "imgui.h"

#include "rtr/editor/core/editor_panel.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::editor {

class LoggerPanel final : public IEditorPanel {
private:
    static constexpr std::size_t kDefaultMaxEntries = 1500;
    static constexpr std::size_t kFilterBufferSize = 256;

    struct SharedState {
        std::mutex mutex{};
        std::deque<utils::LogEntry> entries{};
        std::size_t max_entries{kDefaultMaxEntries};
        bool paused{false};
        bool has_new_entries{false};
    };

    std::shared_ptr<SharedState> m_state{std::make_shared<SharedState>()};
    utils::LogSubscriptionHandle m_subscription{0};
    bool m_visible{true};
    int m_order{350};
    bool m_auto_scroll{true};
    int m_level_filter_index{0};
    std::array<char, kFilterBufferSize> m_filter_buffer{};
    int m_global_level_index{static_cast<int>(utils::build_default_log_level())};

    static const char* const* level_labels() {
        static const char* const kLabels[] = {
            "trace",
            "debug",
            "info",
            "warn",
            "error",
            "critical",
            "off"
        };
        return kLabels;
    }

    static constexpr int level_label_count() {
        return 7;
    }

    static const char* const* filter_labels() {
        static const char* const kLabels[] = {
            "all",
            "trace",
            "debug",
            "info",
            "warn",
            "error",
            "critical",
            "off"
        };
        return kLabels;
    }

    static constexpr int filter_label_count() {
        return 8;
    }

    static utils::LogLevel index_to_level(int index) {
        switch (index) {
        case 0:
            return utils::LogLevel::trace;
        case 1:
            return utils::LogLevel::debug;
        case 2:
            return utils::LogLevel::info;
        case 3:
            return utils::LogLevel::warn;
        case 4:
            return utils::LogLevel::err;
        case 5:
            return utils::LogLevel::critical;
        case 6:
            return utils::LogLevel::off;
        default:
            return utils::build_default_log_level();
        }
    }

    static const char* level_text(utils::LogLevel level) {
        switch (level) {
        case utils::LogLevel::trace:
            return "trace";
        case utils::LogLevel::debug:
            return "debug";
        case utils::LogLevel::info:
            return "info";
        case utils::LogLevel::warn:
            return "warn";
        case utils::LogLevel::err:
            return "error";
        case utils::LogLevel::critical:
            return "critical";
        case utils::LogLevel::off:
            return "off";
        }
        return "info";
    }

    static ImVec4 level_color(utils::LogLevel level) {
        switch (level) {
        case utils::LogLevel::trace:
            return ImVec4(0.65f, 0.65f, 0.65f, 1.0f);
        case utils::LogLevel::debug:
            return ImVec4(0.55f, 0.75f, 1.0f, 1.0f);
        case utils::LogLevel::info:
            return ImVec4(0.75f, 0.90f, 0.75f, 1.0f);
        case utils::LogLevel::warn:
            return ImVec4(1.0f, 0.85f, 0.40f, 1.0f);
        case utils::LogLevel::err:
            return ImVec4(1.0f, 0.50f, 0.45f, 1.0f);
        case utils::LogLevel::critical:
            return ImVec4(1.0f, 0.35f, 0.80f, 1.0f);
        case utils::LogLevel::off:
            return ImVec4(0.45f, 0.45f, 0.45f, 1.0f);
        }
        return ImVec4(0.75f, 0.90f, 0.75f, 1.0f);
    }

    static std::string format_timestamp(const utils::LogEntry& entry) {
        const auto time_t = spdlog::log_clock::to_time_t(entry.timestamp);
        std::tm local_time{};
#if defined(_WIN32)
        localtime_s(&local_time, &time_t);
#else
        localtime_r(&time_t, &local_time);
#endif

        std::array<char, 16> time_buf{};
        std::strftime(time_buf.data(), time_buf.size(), "%H:%M:%S", &local_time);

        const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                                entry.timestamp.time_since_epoch()
                            ) %
            1000;

        char full_buf[24]{};
        std::snprintf(
            full_buf,
            sizeof(full_buf),
            "%s.%03lld",
            time_buf.data(),
            static_cast<long long>(millis.count())
        );
        return std::string(full_buf);
    }

    static void push_entry(const std::shared_ptr<SharedState>& state, const utils::LogEntry& entry) {
        std::scoped_lock lock(state->mutex);
        if (state->paused) {
            return;
        }
        state->entries.push_back(entry);
        while (state->entries.size() > state->max_entries) {
            state->entries.pop_front();
        }
        state->has_new_entries = true;
    }

    static bool level_matches(int filter_index, utils::LogLevel level) {
        if (filter_index <= 0) {
            return true;
        }
        return index_to_level(filter_index - 1) == level;
    }

    bool text_matches(const utils::LogEntry& entry) const {
        if (m_filter_buffer[0] == '\0') {
            return true;
        }
        return entry.message.find(m_filter_buffer.data()) != std::string::npos ||
            entry.logger_name.find(m_filter_buffer.data()) != std::string::npos;
    }

    std::vector<utils::LogEntry> snapshot_entries(bool* has_new_entries = nullptr) {
        std::vector<utils::LogEntry> snapshot{};
        std::scoped_lock lock(m_state->mutex);
        snapshot.reserve(m_state->entries.size());
        for (const auto& entry : m_state->entries) {
            snapshot.push_back(entry);
        }
        if (has_new_entries != nullptr) {
            *has_new_entries = m_state->has_new_entries;
            m_state->has_new_entries = false;
        }
        return snapshot;
    }

    void set_paused_state(bool paused) {
        std::scoped_lock lock(m_state->mutex);
        m_state->paused = paused;
    }

    bool paused_state() const {
        std::scoped_lock lock(m_state->mutex);
        return m_state->paused;
    }

public:
    explicit LoggerPanel(std::size_t max_entries = kDefaultMaxEntries) {
        m_state->max_entries = std::max<std::size_t>(max_entries, 1);
        auto weak_state = std::weak_ptr<SharedState>(m_state);
        m_subscription = utils::subscribe_logs([weak_state](const utils::LogEntry& entry) {
            if (auto state = weak_state.lock()) {
                push_entry(state, entry);
            }
        });
    }

    ~LoggerPanel() override {
        if (m_subscription != 0) {
            (void)utils::unsubscribe_logs(m_subscription);
            m_subscription = 0;
        }
    }

    LoggerPanel(const LoggerPanel&) = delete;
    LoggerPanel& operator=(const LoggerPanel&) = delete;
    LoggerPanel(LoggerPanel&&) = delete;
    LoggerPanel& operator=(LoggerPanel&&) = delete;

    std::string_view id() const override {
        return "logger";
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

    std::size_t buffered_count() const {
        std::scoped_lock lock(m_state->mutex);
        return m_state->entries.size();
    }

    bool contains_message(std::string_view needle) const {
        std::scoped_lock lock(m_state->mutex);
        for (const auto& entry : m_state->entries) {
            if (entry.message.find(needle) != std::string::npos) {
                return true;
            }
        }
        return false;
    }

    void on_imgui(EditorContext& /*ctx*/) override {
        if (!m_visible) {
            return;
        }

        if (!ImGui::Begin("Logger", &m_visible)) {
            ImGui::End();
            return;
        }

        ImGui::SetNextItemWidth(200.0f);
        ImGui::InputTextWithHint(
            "##logger_filter",
            "Filter message/logger",
            m_filter_buffer.data(),
            m_filter_buffer.size()
        );
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120.0f);
        ImGui::Combo(
            "##logger_level_filter",
            &m_level_filter_index,
            filter_labels(),
            filter_label_count()
        );

        bool paused = paused_state();
        if (ImGui::Checkbox("Pause", &paused)) {
            set_paused_state(paused);
        }
        ImGui::SameLine();
        ImGui::Checkbox("Auto Scroll", &m_auto_scroll);
        ImGui::SameLine();
        if (ImGui::Button("Clear")) {
            std::scoped_lock lock(m_state->mutex);
            m_state->entries.clear();
            m_state->has_new_entries = false;
        }

        ImGui::SetNextItemWidth(140.0f);
        if (ImGui::Combo(
                "Global Level",
                &m_global_level_index,
                level_labels(),
                level_label_count()
            )) {
            utils::set_level(index_to_level(m_global_level_index));
        }

        bool has_new_entries = false;
        auto entries = snapshot_entries(&has_new_entries);

        ImGui::Separator();
        ImGui::Text("Entries: %zu", entries.size());

        ImGui::BeginChild("##logger_entries", ImVec2(0.0f, 0.0f), false, ImGuiWindowFlags_HorizontalScrollbar);
        for (const auto& entry : entries) {
            if (!level_matches(m_level_filter_index, entry.level) || !text_matches(entry)) {
                continue;
            }

            const std::string ts = format_timestamp(entry);
            ImGui::PushStyleColor(ImGuiCol_Text, level_color(entry.level));
            ImGui::TextUnformatted(ts.c_str());
            ImGui::PopStyleColor();
            ImGui::SameLine();
            ImGui::Text("[%s] [%s] %s", level_text(entry.level), entry.logger_name.c_str(), entry.message.c_str());
        }

        if (m_auto_scroll && has_new_entries) {
            ImGui::SetScrollHereY(1.0f);
        }

        ImGui::EndChild();
        ImGui::End();
    }
};

} // namespace rtr::editor
