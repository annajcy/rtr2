#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "rtr/resource/resource_manager.hpp"

namespace rtr::resource::test {

namespace {

struct DummyCpu {
    int value{0};
};

struct DummyGpu {
    int value{0};
};

struct DummyOptions {
    int scale{1};
};

struct DummyKind {
    using cpu_type = DummyCpu;
    using gpu_type = DummyGpu;
    using options_type = DummyOptions;

    static void validate_cpu(const cpu_type& cpu) {
        if (cpu.value <= 0) {
            throw std::invalid_argument("DummyCpu value must be positive.");
        }
    }

    static cpu_type normalize_cpu(cpu_type cpu, const options_type& options) {
        cpu.value *= options.scale;
        return cpu;
    }

    static cpu_type load_from_path(const std::filesystem::path& abs_path, const options_type&) {
        std::ifstream in(abs_path);
        if (!in.is_open()) {
            throw std::runtime_error("Failed to open file: " + abs_path.string());
        }
        int raw = 0;
        in >> raw;
        if (!in.good() && !in.eof()) {
            throw std::runtime_error("Failed to read file: " + abs_path.string());
        }
        return cpu_type{.value = raw};
    }

    static void save_to_path(const cpu_type& cpu, const std::filesystem::path& abs_path) {
        std::filesystem::create_directories(abs_path.parent_path());
        std::ofstream out(abs_path);
        if (!out.is_open()) {
            throw std::runtime_error("Failed to open file: " + abs_path.string());
        }
        out << cpu.value;
    }

    static gpu_type upload_to_gpu(rhi::Device&, const cpu_type& cpu, const options_type& options) {
        return gpu_type{.value = cpu.value + options.scale};
    }
};

struct TempDir {
    std::filesystem::path path{};

    explicit TempDir(const std::string& name)
        : path(std::filesystem::temp_directory_path() / name) {
        std::filesystem::remove_all(path);
        std::filesystem::create_directories(path);
    }

    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
};

} // namespace

TEST(ResourceManagerGenericKindsTest, SupportsCustomKindLifecycleInSameManager) {
    ResourceManagerT<rhi::kFramesInFlight, MeshResourceKind, TextureResourceKind, DummyKind> manager{};

    const auto handle = manager.create<DummyKind>(DummyCpu{.value = 2}, DummyOptions{.scale = 3});
    EXPECT_TRUE(handle.is_valid());
    EXPECT_TRUE(manager.alive<DummyKind>(handle));

    const auto& cpu = manager.cpu<DummyKind>(handle);
    EXPECT_EQ(cpu.value, 6);

    auto& gpu = manager.require_gpu<DummyKind>(handle, *reinterpret_cast<rhi::Device*>(0x1));
    EXPECT_EQ(gpu.value, 9);

    manager.unload<DummyKind>(handle);
    EXPECT_FALSE(manager.alive<DummyKind>(handle));
    EXPECT_THROW((void)manager.cpu<DummyKind>(handle), std::runtime_error);
}

TEST(ResourceManagerGenericKindsTest, RelativePathLoadAndSaveUseKindHooks) {
    TempDir temp_dir("rtr_resource_manager_generic_kind_test");
    const auto input_path = temp_dir.path / "in" / "value.txt";
    std::filesystem::create_directories(input_path.parent_path());
    {
        std::ofstream out(input_path);
        ASSERT_TRUE(out.is_open());
        out << 7;
    }

    ResourceManagerT<rhi::kFramesInFlight, MeshResourceKind, TextureResourceKind, DummyKind> manager(temp_dir.path);
    const auto handle = manager.create_from_relative_path<DummyKind>("in/value.txt", DummyOptions{.scale = 2});

    EXPECT_TRUE(handle.is_valid());
    EXPECT_EQ(manager.cpu<DummyKind>(handle).value, 14);

    manager.save_cpu_to_relative_path<DummyKind>(handle, "out/saved.txt");

    std::ifstream in(temp_dir.path / "out" / "saved.txt");
    ASSERT_TRUE(in.is_open());
    int saved = 0;
    in >> saved;
    EXPECT_EQ(saved, 14);
}

} // namespace rtr::resource::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
