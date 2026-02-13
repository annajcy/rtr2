#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rtr::utils {

template <typename... Args>
class Event {
public:
    using Action = std::function<void(Args...)>;
    using ActionHandle = uint64_t;

private:
    struct ActionEntry {
        ActionHandle handle{0};
        Action action{};
    };

    std::vector<ActionEntry> m_actions{};
    ActionHandle m_next_handle{1};

public:
    Event() = default;

    explicit Event(const std::vector<Action>& actions) {
        for (const auto& action : actions) {
            add(action);
        }
    }

    explicit Event(const Action& action) {
        add(action);
    }

    ActionHandle add(const Action& action) {
        if (!action) {
            throw std::runtime_error("Event action must be valid.");
        }
        ActionEntry entry{};
        entry.handle = m_next_handle++;
        entry.action = action;
        m_actions.emplace_back(std::move(entry));
        return m_actions.back().handle;
    }

    bool remove(ActionHandle handle) {
        auto it = std::find_if(
            m_actions.begin(),
            m_actions.end(),
            [handle](const ActionEntry& entry) { return entry.handle == handle; }
        );
        if (it == m_actions.end()) {
            return false;
        }
        m_actions.erase(it);
        return true;
    }

    void clear() {
        m_actions.clear();
    }

    void execute(Args... args) {
        for (const auto& action : m_actions) {
            action.action(args...);
        }
    }

    size_t size() const {
        return m_actions.size();
    }
};

template <typename... Args>
class EventCenter {
public:
    using EventType = Event<Args...>;
    using Action = typename EventType::Action;
    using ActionHandle = typename EventType::ActionHandle;

private:
    std::unordered_map<std::string, EventType> m_event_map{};

public:
    void register_event(const std::string& event_name) {
        if (!m_event_map.contains(event_name)) {
            m_event_map.emplace(event_name, EventType{});
        }
    }

    void register_event(const std::string& event_name, const EventType& event) {
        m_event_map[event_name] = event;
    }

    bool unregister_event(const std::string& event_name) {
        return m_event_map.erase(event_name) > 0;
    }

    ActionHandle add_action(const std::string& event_name, const Action& action) {
        auto it = m_event_map.find(event_name);
        if (it == m_event_map.end()) {
            throw std::runtime_error("Event not found: " + event_name);
        }
        return it->second.add(action);
    }

    bool remove_action(const std::string& event_name, ActionHandle handle) {
        auto it = m_event_map.find(event_name);
        if (it == m_event_map.end()) {
            throw std::runtime_error("Event not found: " + event_name);
        }
        return it->second.remove(handle);
    }

    void trigger_event(const std::string& event_name, Args... args) {
        auto it = m_event_map.find(event_name);
        if (it == m_event_map.end()) {
            throw std::runtime_error("Event not found: " + event_name);
        }
        it->second.execute(args...);
    }

    bool has_event(const std::string& event_name) const {
        return m_event_map.contains(event_name);
    }

    size_t action_count(const std::string& event_name) const {
        auto it = m_event_map.find(event_name);
        if (it == m_event_map.end()) {
            throw std::runtime_error("Event not found: " + event_name);
        }
        return it->second.size();
    }

    void clear() {
        m_event_map.clear();
    }
};

} // namespace rtr
