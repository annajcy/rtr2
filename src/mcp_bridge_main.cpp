#include <cstdio>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "rtr/mcp/bridge_core.hpp"
#include "rtr/utils/log.hpp"

#if defined(_WIN32)
#include <fcntl.h>
#include <io.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace {

std::FILE* open_protocol_stream() {
#if defined(_WIN32)
    const int protocol_fd = _dup(_fileno(stdout));
    const int null_fd = _open("NUL", _O_WRONLY);
    if (protocol_fd == -1 || null_fd == -1) {
        return stdout;
    }
    (void)_dup2(null_fd, _fileno(stdout));
    _close(null_fd);
    std::FILE* stream = _fdopen(protocol_fd, "w");
#else
    const int protocol_fd = dup(STDOUT_FILENO);
    const int null_fd = open("/dev/null", O_WRONLY);
    if (protocol_fd == -1 || null_fd == -1) {
        return stdout;
    }
    (void)dup2(null_fd, STDOUT_FILENO);
    close(null_fd);
    std::FILE* stream = fdopen(protocol_fd, "w");
#endif
    return stream == nullptr ? stdout : stream;
}

} // namespace

int main() {
    rtr::utils::init_logging(rtr::utils::LogConfig{
        .enable_console = false,
        .enable_file = true,
        .file_path = "./output/logs/rtr_mcp_bridge.log",
        .max_file_size = 5u * 1024u * 1024u,
        .max_files = 2,
        .level = rtr::utils::LogLevel::info,
    });

    std::FILE* protocol_stream = open_protocol_stream();
    rtr::mcp::BridgeCore core;
    std::string line{};

    while (std::getline(std::cin, line)) {
        if (line.empty()) {
            continue;
        }

        nlohmann::json response{};
        try {
            const auto request = nlohmann::json::parse(line);
            response = core.handle_request(request);
        } catch (const std::exception& e) {
            response = nlohmann::json{
                {"id", nullptr},
                {"ok", false},
                {"error_code", "invalid_json"},
                {"message", e.what()},
                {"data", nlohmann::json::object()},
            };
        }

        const std::string payload = response.dump() + "\n";
        std::fputs(payload.c_str(), protocol_stream);
        std::fflush(protocol_stream);
    }

    return 0;
}
