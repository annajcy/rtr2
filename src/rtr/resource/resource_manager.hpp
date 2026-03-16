#pragma once

#include <algorithm>
#include <concepts>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rtr/resource/resource_fwd.hpp"
#include "rtr/resource/resource_kinds.hpp"
#include "rtr/resource/resource_types.hpp"
#include "rtr/rhi/device.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::resource {

namespace detail {

template <class... Types>
struct unique_types : std::true_type {};

template <class Type, class... Rest>
struct unique_types<Type, Rest...>
    : std::bool_constant<(!std::same_as<Type, Rest> && ...) && unique_types<Rest...>::value> {};

template <class Kind, class... SupportedKinds>
inline constexpr bool contains_kind_v = (std::same_as<Kind, SupportedKinds> || ...);

} // namespace detail

template <std::uint32_t FramesInFlight, class Kind0, class Kind1, class... Kinds>
class ResourceManagerT {
public:
    static constexpr std::string_view kDefaultResourceRootDir = "./assets/";

private:
    template <BaseResourceKind Kind>
    struct ResourceRecord {
        typename Kind::cpu_type cpu{};
        typename Kind::options_type options{};
        std::unique_ptr<typename Kind::gpu_type> gpu{};
        bool cpu_dirty{false};
    };

    template <BaseResourceKind Kind>
    struct RetiredGpu {
        std::uint64_t retire_after_frame{0};
        std::unique_ptr<typename Kind::gpu_type> gpu{};
    };

    template <BaseResourceKind Kind>
    struct ResourceStorage {
        std::uint64_t next_id{1};
        std::unordered_map<ResourceHandle<Kind>, ResourceRecord<Kind>> records{};
        std::vector<RetiredGpu<Kind>> retired{};
    };

    static_assert(
        BaseResourceKind<Kind0> && BaseResourceKind<Kind1> && (BaseResourceKind<Kinds> && ...),
        "ResourceManager template parameters must satisfy BaseResourceKind."
    );
    static_assert(
        detail::unique_types<Kind0, Kind1, Kinds...>::value,
        "ResourceManager kinds must be unique."
    );
    static_assert(FramesInFlight > 0, "FramesInFlight must be greater than zero.");

    std::uint64_t m_current_frame_serial{0};
    std::filesystem::path m_resource_root_dir{std::string(kDefaultResourceRootDir)};

    std::tuple<ResourceStorage<Kind0>, ResourceStorage<Kind1>, ResourceStorage<Kinds>...> m_storages{};

public:
    explicit ResourceManagerT(
        std::filesystem::path resource_root_dir = std::filesystem::path(std::string(kDefaultResourceRootDir))
    )
        : m_resource_root_dir(std::move(resource_root_dir)) {
        logger()->info(
            "ResourceManager initialized (frames_in_flight={}, root='{}')",
            FramesInFlight,
            m_resource_root_dir.string()
        );
    }

    const std::filesystem::path& resource_root_dir() const {
        return m_resource_root_dir;
    }

    void set_resource_root_dir(std::filesystem::path resource_root_dir) {
        m_resource_root_dir = std::move(resource_root_dir);
    }

    template <BaseResourceKind Kind>
    ResourceHandle<Kind> create(
        typename Kind::cpu_type cpu,
        typename Kind::options_type options = {}
    ) {
        auto& store = storage<Kind>();

        Kind::validate_cpu(cpu);
        cpu = Kind::normalize_cpu(std::move(cpu), options);
        Kind::validate_cpu(cpu);

        const ResourceHandle<Kind> handle{store.next_id++};
        store.records.emplace(handle, ResourceRecord<Kind>{
            .cpu = std::move(cpu),
            .options = std::move(options),
            .gpu = nullptr,
            .cpu_dirty = false
        });
        logger()->debug("Resource created (handle={})", handle.value);
        return handle;
    }

    template <BaseResourceKind Kind>
    ResourceHandle<Kind> create_from_relative_path(
        const std::string& rel_path,
        typename Kind::options_type options = {}
    ) {
        const auto abs_path = resolve_resource_path(rel_path);
        logger()->debug("Loading resource from relative path '{}' -> '{}'", rel_path, abs_path.string());
        auto cpu = Kind::load_from_path(abs_path, options);
        return create<Kind>(std::move(cpu), std::move(options));
    }

    template <BaseResourceKind Kind>
    void save_cpu_to_relative_path(ResourceHandle<Kind> handle, const std::string& rel_path) {
        const auto abs_path = resolve_resource_path(rel_path);
        const auto& record = require_record<Kind>(handle);
        ensure_cpu_loaded(record);
        Kind::save_to_path(record.cpu, abs_path);
    }

    template <BaseResourceKind Kind>
    void unload(ResourceHandle<Kind> handle) {
        auto& store = storage<Kind>();
        auto it = store.records.find(handle);
        if (it == store.records.end()) {
            logger()->warn("unload ignored: invalid handle={}", handle.value);
            return;
        }

        ResourceRecord<Kind>& record = it->second;
        retire_gpu(record);
        store.records.erase(it);
        logger()->debug("Resource unloaded (handle={})", handle.value);
    }

    template <BaseResourceKind Kind>
    const typename Kind::cpu_type& cpu(ResourceHandle<Kind> handle) {
        auto& record = require_record<Kind>(handle);
        ensure_cpu_loaded(record);
        return record.cpu;
    }

    template <MutableResourceKind Kind, typename Fn>
        requires std::invocable<Fn&, typename Kind::cpu_type&>
    void update_cpu(ResourceHandle<Kind> handle, Fn&& mutate) {
        auto& record = require_record<Kind>(handle);
        ensure_cpu_loaded(record);

        auto updated_cpu = record.cpu;
        std::forward<Fn>(mutate)(updated_cpu);
        updated_cpu = Kind::normalize_cpu(std::move(updated_cpu), record.options);
        Kind::validate_cpu(updated_cpu);

        record.cpu = std::move(updated_cpu);
        record.cpu_dirty = true;
    }

    template <BaseResourceKind Kind>
    bool alive(ResourceHandle<Kind> handle) const {
        const auto& store = storage<Kind>();
        return store.records.find(handle) != store.records.end();
    }

    template <ImmutableResourceKind Kind>
    typename Kind::gpu_type& require_gpu(ResourceHandle<Kind> handle, rhi::Device& device) {
        auto& record = require_record<Kind>(handle);
        ensure_cpu_loaded(record);

        if (!record.gpu) {
            logger()->debug("Handle={} triggering first GPU upload.", handle.value);
            record.gpu = std::make_unique<typename Kind::gpu_type>(
                Kind::upload_to_gpu(device, record.cpu, record.options)
            );
            record.cpu_dirty = false;
            return *record.gpu;
        }

        if (!record.cpu_dirty) {
            return *record.gpu;
        }

        if constexpr (MutableResourceKind<Kind>) {
            logger()->debug("Handle={} syncing dirty mutable CPU resource to existing GPU allocation.", handle.value);
            Kind::sync_gpu(device, record.cpu, *record.gpu, record.options);
            record.cpu_dirty = false;
        }

        return *record.gpu;
    }

    void tick(std::uint64_t frame_serial) {
        m_current_frame_serial = frame_serial;

        for_each_storage([frame_serial](auto& store) {
            std::erase_if(store.retired, [frame_serial](const auto& retired) {
                return retired.retire_after_frame <= frame_serial;
            });
        });
    }

    void flush_after_wait_idle() {
        logger()->info(
            "Flushing GPU caches after wait_idle (live_resources={}, retired_resources={})",
            live_resource_count(),
            retired_resource_count()
        );

        for_each_storage([](auto& store) {
            store.retired.clear();
            for (auto& [_, record] : store.records) {
                record.gpu.reset();
            }
        });
    }

private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("resource.manager");
    }

    template <BaseResourceKind Kind>
    ResourceStorage<Kind>& storage() {
        static_assert(
            detail::contains_kind_v<Kind, Kind0, Kind1, Kinds...>,
            "Kind is not registered in this ResourceManager instance."
        );
        return std::get<ResourceStorage<Kind>>(m_storages);
    }

    template <BaseResourceKind Kind>
    const ResourceStorage<Kind>& storage() const {
        static_assert(
            detail::contains_kind_v<Kind, Kind0, Kind1, Kinds...>,
            "Kind is not registered in this ResourceManager instance."
        );
        return std::get<ResourceStorage<Kind>>(m_storages);
    }

    std::filesystem::path resolve_resource_path(const std::string& rel_path) const {
        if (rel_path.empty()) {
            logger()->error("resolve_resource_path failed: relative path is empty.");
            throw std::invalid_argument("Relative resource path must not be empty.");
        }

        const std::filesystem::path path(rel_path);
        if (path.is_absolute()) {
            logger()->error("resolve_resource_path failed: absolute path '{}' is not allowed.", rel_path);
            throw std::invalid_argument("Relative resource path API does not accept absolute path.");
        }

        return (m_resource_root_dir / path).lexically_normal();
    }

    std::uint64_t retire_after_frame() const {
        return m_current_frame_serial + static_cast<std::uint64_t>(FramesInFlight);
    }

    template <BaseResourceKind Kind>
    void ensure_cpu_loaded(const ResourceRecord<Kind>& record) const {
        try {
            Kind::validate_cpu(record.cpu);
        } catch (const std::exception&) {
            logger()->error("ensure_cpu_loaded failed: CPU payload missing for live handle.");
            throw std::runtime_error("Resource CPU data is missing for live handle.");
        }
    }

    template <BaseResourceKind Kind>
    void retire_gpu(ResourceRecord<Kind>& record) {
        if (!record.gpu) {
            return;
        }

        auto& store = storage<Kind>();
        const auto retire_frame = retire_after_frame();
        store.retired.emplace_back(RetiredGpu<Kind>{
            .retire_after_frame = retire_frame,
            .gpu = std::move(record.gpu)
        });
        logger()->debug("Retired GPU allocation (release_after_frame={})", retire_frame);
    }

    template <BaseResourceKind Kind>
    ResourceRecord<Kind>& require_record(ResourceHandle<Kind> handle) {
        auto& store = storage<Kind>();
        const auto it = store.records.find(handle);
        if (it == store.records.end()) {
            logger()->error("Invalid/unloaded resource handle requested: {}", handle.value);
            throw std::runtime_error("Resource handle is invalid or unloaded.");
        }
        return it->second;
    }

    template <BaseResourceKind Kind>
    const ResourceRecord<Kind>& require_record(ResourceHandle<Kind> handle) const {
        const auto& store = storage<Kind>();
        const auto it = store.records.find(handle);
        if (it == store.records.end()) {
            logger()->error("Invalid/unloaded resource handle requested: {}", handle.value);
            throw std::runtime_error("Resource handle is invalid or unloaded.");
        }
        return it->second;
    }

    template <class Fn>
    void for_each_storage(Fn&& fn) {
        std::apply([&](auto&... stores) {
            (fn(stores), ...);
        }, m_storages);
    }

    template <class Fn>
    void for_each_storage(Fn&& fn) const {
        std::apply([&](const auto&... stores) {
            (fn(stores), ...);
        }, m_storages);
    }

    std::size_t live_resource_count() const {
        std::size_t total = 0;
        for_each_storage([&total](const auto& store) {
            total += store.records.size();
        });
        return total;
    }

    std::size_t retired_resource_count() const {
        std::size_t total = 0;
        for_each_storage([&total](const auto& store) {
            total += store.retired.size();
        });
        return total;
    }
};

} // namespace rtr::resource
