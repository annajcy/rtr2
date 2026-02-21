#pragma once

#include <string_view>
#include <tuple>
#include <utility>
#include <stdexcept>
#include <string>

namespace rtr::framework::integration {

struct DispatchResult {
    bool             matched{false};
    std::string_view mapper_name{""};
};

template <typename... Mappers, typename... Args>
DispatchResult dispatch_impl(std::tuple<Mappers...>, Args&&... args) {
    DispatchResult result;
    (void)((!result.matched &&
                    [&]() {
                        try {
                            return Mappers::matches(args...);
                        } catch (const std::exception& e) {
                            throw std::runtime_error(std::string("[mapper=") + std::string(Mappers::kName) +
                                                     "] matches failed: " + e.what());
                        }
                    }()
                ? (
                      [&]() {
                          try {
                              Mappers::map(std::forward<Args>(args)...);
                          } catch (const std::exception& e) {
                              throw std::runtime_error(std::string("[mapper=") + std::string(Mappers::kName) +
                                                       "] map failed: " + e.what());
                          }
                      }(),
                      result.matched = true, result.mapper_name = Mappers::kName, true)
                : false) ||
           ...);
    return result;
}

}  // namespace rtr::framework::integration
