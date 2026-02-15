#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "spdlog/details/log_msg.h"
#include "spdlog/logger.h"
#include "spdlog/sinks/sink.h"
#include "spdlog/sinks/null_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

namespace rtr::utils {

enum class LogLevel {
    trace,
    debug,
    info,
    warn,
    err,
    critical,
    off,
};

struct LogEntry {
    spdlog::log_clock::time_point timestamp{};
    LogLevel level{LogLevel::info};
    std::string logger_name{};
    std::string message{};
    std::optional<std::uint64_t> sequence{};
};

using LogSubscriber = std::function<void(const LogEntry&)>;
using LogSubscriptionHandle = std::uint64_t;

constexpr LogLevel build_default_log_level() {
#if !defined(NDEBUG)
    return LogLevel::debug;
#else
    return LogLevel::info;
#endif
}

struct LogConfig {
    bool enable_console{true};
    bool enable_file{true};
    std::string file_path{"./output/logs/rtr.log"};
    std::size_t max_file_size{10u * 1024u * 1024u};
    std::size_t max_files{3};
    LogLevel level{build_default_log_level()};
};

namespace log_detail {

inline constexpr const char* kDefaultPattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] %v";

inline spdlog::level::level_enum to_spdlog_level(LogLevel level) {
    switch (level) {
    case LogLevel::trace:
        return spdlog::level::trace;
    case LogLevel::debug:
        return spdlog::level::debug;
    case LogLevel::info:
        return spdlog::level::info;
    case LogLevel::warn:
        return spdlog::level::warn;
    case LogLevel::err:
        return spdlog::level::err;
    case LogLevel::critical:
        return spdlog::level::critical;
    case LogLevel::off:
        return spdlog::level::off;
    }
    return spdlog::level::info;
}

inline LogLevel to_log_level(spdlog::level::level_enum level) {
    switch (level) {
    case spdlog::level::trace:
        return LogLevel::trace;
    case spdlog::level::debug:
        return LogLevel::debug;
    case spdlog::level::info:
        return LogLevel::info;
    case spdlog::level::warn:
        return LogLevel::warn;
    case spdlog::level::err:
        return LogLevel::err;
    case spdlog::level::critical:
        return LogLevel::critical;
    case spdlog::level::off:
        return LogLevel::off;
    default:
        return LogLevel::info;
    }
}

struct Registry {
    std::mutex mutex{};
    std::mutex subscribers_mutex{};
    bool initialized{false};
    LogConfig config{};
    std::vector<spdlog::sink_ptr> sinks{};
    std::unordered_map<std::string, std::shared_ptr<spdlog::logger>> loggers{};
    std::unordered_map<LogSubscriptionHandle, LogSubscriber> subscribers{};
    LogSubscriptionHandle next_subscriber_handle{1};
    std::atomic<std::uint64_t> next_sequence{1};
};

inline Registry& registry() {
    static Registry instance{};
    return instance;
}

class LogSubscriberSink final : public spdlog::sinks::sink {
public:
    void log(const spdlog::details::log_msg& msg) override {
        auto& reg = registry();

        LogEntry entry{};
        entry.timestamp = msg.time;
        entry.level = to_log_level(msg.level);
        entry.logger_name.assign(msg.logger_name.data(), msg.logger_name.size());
        entry.message.assign(msg.payload.data(), msg.payload.size());
        entry.sequence = reg.next_sequence.fetch_add(1, std::memory_order_relaxed);

        std::vector<LogSubscriber> callbacks{};
        {
            std::scoped_lock lock(reg.subscribers_mutex);
            callbacks.reserve(reg.subscribers.size());
            for (const auto& [_, subscriber] : reg.subscribers) {
                callbacks.push_back(subscriber);
            }
        }

        for (const auto& callback : callbacks) {
            if (!callback) {
                continue;
            }
            try {
                callback(entry);
            } catch (...) {
                // Subscriber callback errors must not break the logging pipeline.
            }
        }
    }

    void flush() override {}
    void set_pattern(const std::string& /*pattern*/) override {}
    void set_formatter(std::unique_ptr<spdlog::formatter> /*sink_formatter*/) override {}
};

inline std::shared_ptr<spdlog::logger> create_logger_unlocked(
    Registry& reg,
    const std::string& name
) {
    auto logger = std::make_shared<spdlog::logger>(name, reg.sinks.begin(), reg.sinks.end());
    logger->set_level(to_spdlog_level(reg.config.level));
    logger->flush_on(spdlog::level::warn);

    if (spdlog::get(name) == nullptr) {
        spdlog::register_logger(logger);
    }
    return logger;
}

inline void set_level_unlocked(Registry& reg, LogLevel level) {
    reg.config.level = level;
    const auto spdlog_level = to_spdlog_level(level);
    for (auto& [_, logger] : reg.loggers) {
        logger->set_level(spdlog_level);
    }
}

inline void init_unlocked(Registry& reg, const LogConfig& config) {
    if (reg.initialized) {
        set_level_unlocked(reg, config.level);
        return;
    }

    reg.config = config;
    reg.sinks.clear();
    reg.loggers.clear();
    reg.next_sequence.store(1, std::memory_order_relaxed);

    reg.sinks.push_back(std::make_shared<LogSubscriberSink>());

    if (config.enable_console) {
        auto sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        sink->set_pattern(kDefaultPattern);
        reg.sinks.push_back(std::move(sink));
    }

    if (config.enable_file) {
        const std::filesystem::path output_path(config.file_path);
        if (output_path.has_parent_path()) {
            std::filesystem::create_directories(output_path.parent_path());
        }

        const auto max_file_size = std::max<std::size_t>(config.max_file_size, 1u);
        const auto max_files = std::max<std::size_t>(config.max_files, 1u);
        auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            output_path.string(),
            max_file_size,
            max_files,
            true
        );
        sink->set_pattern(kDefaultPattern);
        reg.sinks.push_back(std::move(sink));
    }

    if (reg.sinks.empty()) {
        auto sink = std::make_shared<spdlog::sinks::null_sink_mt>();
        sink->set_pattern(kDefaultPattern);
        reg.sinks.push_back(std::move(sink));
    }

    auto core_logger = create_logger_unlocked(reg, "core");
    reg.loggers.emplace("core", core_logger);
    reg.initialized = true;

    core_logger->info(
        "Logging initialized (console={}, file={}, path='{}', level={})",
        config.enable_console,
        config.enable_file,
        config.file_path,
        spdlog::level::to_string_view(to_spdlog_level(config.level))
    );
}

} // namespace log_detail

inline void init_logging(const LogConfig& config) {
    auto& reg = log_detail::registry();
    std::scoped_lock lock(reg.mutex);
    log_detail::init_unlocked(reg, config);
}

inline void shutdown_logging() {
    auto& reg = log_detail::registry();
    std::scoped_lock lock(reg.mutex);
    if (!reg.initialized) {
        return;
    }

    for (const auto& [name, _] : reg.loggers) {
        spdlog::drop(name);
    }

    reg.loggers.clear();
    reg.sinks.clear();
    reg.next_sequence.store(1, std::memory_order_relaxed);
    reg.initialized = false;
    {
        std::scoped_lock subscribers_lock(reg.subscribers_mutex);
        reg.subscribers.clear();
        reg.next_subscriber_handle = 1;
    }
}

inline void set_level(LogLevel level) {
    auto& reg = log_detail::registry();
    std::scoped_lock lock(reg.mutex);
    if (!reg.initialized) {
        LogConfig config{};
        config.level = level;
        log_detail::init_unlocked(reg, config);
        return;
    }
    log_detail::set_level_unlocked(reg, level);
}

inline std::shared_ptr<spdlog::logger> get_logger(std::string_view module) {
    auto& reg = log_detail::registry();
    std::scoped_lock lock(reg.mutex);

    if (!reg.initialized) {
        log_detail::init_unlocked(reg, LogConfig{});
    }

    const std::string name = module.empty() ? std::string("core") : std::string(module);
    if (const auto it = reg.loggers.find(name); it != reg.loggers.end()) {
        return it->second;
    }

    auto logger = log_detail::create_logger_unlocked(reg, name);
    reg.loggers.emplace(name, logger);
    return logger;
}

inline LogSubscriptionHandle subscribe_logs(LogSubscriber cb) {
    if (!cb) {
        return 0;
    }

    auto& reg = log_detail::registry();
    {
        std::scoped_lock lock(reg.mutex);
        if (!reg.initialized) {
            log_detail::init_unlocked(reg, LogConfig{});
        }
    }

    std::scoped_lock subscribers_lock(reg.subscribers_mutex);
    const auto handle = reg.next_subscriber_handle++;
    reg.subscribers.emplace(handle, std::move(cb));
    return handle;
}

inline bool unsubscribe_logs(LogSubscriptionHandle handle) {
    if (handle == 0) {
        return false;
    }
    auto& reg = log_detail::registry();
    std::scoped_lock subscribers_lock(reg.subscribers_mutex);
    return reg.subscribers.erase(handle) > 0;
}

} // namespace rtr::utils
