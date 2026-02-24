#pragma once

#include <algorithm>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rtr/utils/log.hpp"

namespace rtr::utils {

class ISubscriptionOwner {
public:
    virtual ~ISubscriptionOwner() = default;
    virtual void unsubscribe(std::uint64_t id) = 0;
};

class SubscriptionToken {
private:
    std::weak_ptr<ISubscriptionOwner> m_owner{};
    std::uint64_t m_id{0};

public:
    SubscriptionToken() = default;

    SubscriptionToken(std::weak_ptr<ISubscriptionOwner> owner, std::uint64_t id)
        : m_owner(std::move(owner)),
          m_id(id) {}

    ~SubscriptionToken() {
        reset();
    }

    SubscriptionToken(const SubscriptionToken&) = delete;
    SubscriptionToken& operator=(const SubscriptionToken&) = delete;

    SubscriptionToken(SubscriptionToken&& other) noexcept
        : m_owner(std::move(other.m_owner)),
          m_id(other.m_id) {
        other.m_id = 0;
    }

    SubscriptionToken& operator=(SubscriptionToken&& other) noexcept {
        if (this == &other) {
            return *this;
        }
        reset();
        m_owner = std::move(other.m_owner);
        m_id = other.m_id;
        other.m_id = 0;
        return *this;
    }

    void reset() {
        if (m_id == 0) {
            return;
        }
        if (auto owner = m_owner.lock(); owner != nullptr) {
            owner->unsubscribe(m_id);
        }
        m_owner.reset();
        m_id = 0;
    }

    bool valid() const {
        return m_id != 0;
    }
};

class EventDispatchException : public std::exception {
private:
    std::vector<std::exception_ptr> m_exceptions{};
    std::string m_message{};

public:
    explicit EventDispatchException(std::vector<std::exception_ptr> exceptions)
        : m_exceptions(std::move(exceptions)) {
        if (m_exceptions.empty()) {
            m_message = "Event dispatch failed with no captured exceptions.";
        } else {
            m_message =
                "Event dispatch encountered " + std::to_string(m_exceptions.size()) + " subscriber exception(s).";
        }
    }

    const std::vector<std::exception_ptr>& exceptions() const {
        return m_exceptions;
    }

    const char* what() const noexcept override {
        return m_message.c_str();
    }
};

template <typename... Args>
class Event {
public:
    using Action = std::function<void(Args...)>;
    using Subscription = SubscriptionToken;

private:
    struct ActionEntry {
        std::uint64_t handle{0};
        Action action{};
        bool active{true};
    };

    struct PendingAdd {
        std::uint64_t handle{0};
        Action action{};
    };

    class EventCore final : public ISubscriptionOwner {
    private:
        std::unordered_map<std::uint64_t, ActionEntry> m_actions{};
        std::vector<PendingAdd> m_pending_add{};
        std::vector<std::uint64_t> m_pending_remove{};
        std::uint64_t m_next_handle{1};
        std::uint32_t m_dispatch_depth{0};

        static std::shared_ptr<spdlog::logger> logger() {
            return get_logger("utils.event_center");
        }

        void flush_pending() {
            if (!m_pending_remove.empty()) {
                std::sort(m_pending_remove.begin(), m_pending_remove.end());
                m_pending_remove.erase(
                    std::unique(m_pending_remove.begin(), m_pending_remove.end()),
                    m_pending_remove.end()
                );
                for (const auto handle : m_pending_remove) {
                    m_actions.erase(handle);
                }
                m_pending_remove.clear();
            }

            for (auto& pending : m_pending_add) {
                m_actions[pending.handle] = ActionEntry{
                    .handle = pending.handle,
                    .action = std::move(pending.action),
                    .active = true
                };
            }
            m_pending_add.clear();
        }

        void remove_pending_add(std::uint64_t handle) {
            m_pending_add.erase(
                std::remove_if(
                    m_pending_add.begin(),
                    m_pending_add.end(),
                    [handle](const PendingAdd& entry) { return entry.handle == handle; }
                ),
                m_pending_add.end()
            );
        }

    public:
        std::uint64_t add(const Action& action) {
            if (!action) {
                logger()->error("Event::subscribe failed: action is invalid.");
                throw std::runtime_error("Event action must be valid.");
            }

            const std::uint64_t handle = m_next_handle++;
            if (m_dispatch_depth > 0) {
                m_pending_add.emplace_back(PendingAdd{
                    .handle = handle,
                    .action = action
                });
                logger()->debug(
                    "Event::subscribe queued during dispatch (handle={}, dispatch_depth={}).",
                    handle,
                    m_dispatch_depth
                );
                return handle;
            }

            m_actions[handle] = ActionEntry{
                .handle = handle,
                .action = action,
                .active = true
            };
            logger()->debug("Event::subscribe added (handle={}, subscriber_count={}).", handle, m_actions.size());
            return handle;
        }

        void unsubscribe(std::uint64_t handle) override {
            if (handle == 0) {
                return;
            }

            remove_pending_add(handle);
            auto it = m_actions.find(handle);
            if (it == m_actions.end()) {
                return;
            }

            if (m_dispatch_depth > 0) {
                it->second.active = false;
                m_pending_remove.emplace_back(handle);
                logger()->debug(
                    "Event::unsubscribe deferred during dispatch (handle={}, dispatch_depth={}).",
                    handle,
                    m_dispatch_depth
                );
                return;
            }

            m_actions.erase(it);
            logger()->debug("Event::unsubscribe applied (handle={}, subscriber_count={}).", handle, m_actions.size());
        }

        void clear() {
            if (m_dispatch_depth > 0) {
                m_pending_add.clear();
                m_pending_remove.reserve(m_pending_remove.size() + m_actions.size());
                for (auto& [handle, action] : m_actions) {
                    action.active = false;
                    m_pending_remove.emplace_back(handle);
                }
                return;
            }

            m_actions.clear();
            m_pending_add.clear();
            m_pending_remove.clear();
            logger()->debug("Event::clear applied immediately.");
        }

        void execute(Args... args) {
            std::vector<ActionEntry> snapshot;
            snapshot.reserve(m_actions.size());
            for (const auto& [_, entry] : m_actions) {
                snapshot.emplace_back(entry);
            }

            logger()->trace(
                "Event::publish begin (subscriber_count={}, dispatch_depth={}).",
                snapshot.size(),
                m_dispatch_depth
            );

            ++m_dispatch_depth;
            std::vector<std::exception_ptr> exceptions{};
            for (auto& entry : snapshot) {
                if (!entry.active) {
                    continue;
                }
                try {
                    entry.action(args...);
                } catch (...) {
                    auto log = logger();
                    try {
                        throw;
                    } catch (const std::exception& ex) {
                        log->warn("Event subscriber {} threw exception: {}", entry.handle, ex.what());
                    } catch (...) {
                        log->warn("Event subscriber {} threw non-std exception.", entry.handle);
                    }
                    exceptions.emplace_back(std::current_exception());
                }
            }
            --m_dispatch_depth;

            if (m_dispatch_depth == 0) {
                flush_pending();
            }

            if (!exceptions.empty()) {
                logger()->warn(
                    "Event::publish finished with subscriber exceptions (count={}).",
                    exceptions.size()
                );
                throw EventDispatchException(std::move(exceptions));
            }

            logger()->trace("Event::publish completed successfully.");
        }

        size_t size() const {
            return m_actions.size() + m_pending_add.size();
        }
    };

    std::shared_ptr<EventCore> m_core{std::make_shared<EventCore>()};

public:
    Event() = default;

    explicit Event(const std::vector<Action>& actions) {
        for (const auto& action : actions) {
            (void)m_core->add(action);
        }
    }

    explicit Event(const Action& action) {
        (void)m_core->add(action);
    }

    Subscription subscribe(const Action& action) {
        const auto handle = m_core->add(action);
        return Subscription{
            std::static_pointer_cast<ISubscriptionOwner>(m_core),
            handle
        };
    }

    void clear() {
        m_core->clear();
    }

    void publish(Args... args) {
        m_core->execute(args...);
    }

    void execute(Args... args) {
        publish(args...);
    }

    size_t size() const {
        return m_core->size();
    }
};

class TypedEventCenter {
private:
    class ITypeErasedEvent {
    public:
        virtual ~ITypeErasedEvent() = default;
        virtual void clear() = 0;
    };

    template <typename TEvent>
    class TypedEventHolder final : public ITypeErasedEvent {
    public:
        Event<const TEvent&> event{};

        void clear() override {
            event.clear();
        }
    };

    std::unordered_map<std::type_index, std::shared_ptr<ITypeErasedEvent>> m_events{};

    template <typename TEvent>
    std::shared_ptr<TypedEventHolder<TEvent>> get_or_create_holder() {
        static_assert(!std::is_reference_v<TEvent>, "TEvent must be a non-reference type.");

        const auto type = std::type_index(typeid(TEvent));
        auto it = m_events.find(type);
        if (it != m_events.end()) {
            return std::static_pointer_cast<TypedEventHolder<TEvent>>(it->second);
        }

        auto holder = std::make_shared<TypedEventHolder<TEvent>>();
        m_events.emplace(type, holder);
        return holder;
    }

    template <typename TEvent>
    std::shared_ptr<TypedEventHolder<TEvent>> find_holder() const {
        static_assert(!std::is_reference_v<TEvent>, "TEvent must be a non-reference type.");

        const auto type = std::type_index(typeid(TEvent));
        auto it = m_events.find(type);
        if (it == m_events.end()) {
            return nullptr;
        }
        return std::static_pointer_cast<TypedEventHolder<TEvent>>(it->second);
    }

public:
    template <typename TEvent>
    SubscriptionToken subscribe(std::function<void(const TEvent&)> action) {
        auto token = get_or_create_holder<TEvent>()->event.subscribe(std::move(action));
        get_logger("utils.event_center")
            ->debug("TypedEventCenter::subscribe type={} (action_count={}).",
                     typeid(TEvent).name(),
                     action_count<TEvent>());
        return token;
    }

    template <typename TEvent>
    void publish(const TEvent& event) {
        auto holder = find_holder<TEvent>();
        if (holder == nullptr) {
            get_logger("utils.event_center")
                ->trace("TypedEventCenter::publish ignored (type={} has no subscribers).", typeid(TEvent).name());
            return;
        }
        get_logger("utils.event_center")
            ->trace("TypedEventCenter::publish type={} (action_count={}).", typeid(TEvent).name(), holder->event.size());
        holder->event.publish(event);
    }

    template <typename TEvent>
    size_t action_count() const {
        auto holder = find_holder<TEvent>();
        if (holder == nullptr) {
            return 0;
        }
        return holder->event.size();
    }

    void clear() {
        get_logger("utils.event_center")
            ->debug("TypedEventCenter::clear all event types (type_count={}).", m_events.size());
        for (auto& [_, holder] : m_events) {
            holder->clear();
        }
        m_events.clear();
    }
};

} // namespace rtr::utils
