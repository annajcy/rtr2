#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rtr/utils/file_loder.hpp"

namespace rtr::utils::test {

namespace {

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

TEST(FileLoaderTest, ReadsBinaryFileExactly) {
    TempDir temp_dir("rtr_file_loader_binary_test");
    const auto file_path = temp_dir.path / "binary.bin";

    const std::vector<char> expected{
        static_cast<char>(0x00),
        static_cast<char>(0x01),
        static_cast<char>(0x7f),
        static_cast<char>(0x20),
        static_cast<char>(0xff)
    };

    std::ofstream out(file_path, std::ios::binary);
    ASSERT_TRUE(out.is_open());
    out.write(expected.data(), static_cast<std::streamsize>(expected.size()));
    out.close();

    const auto actual = read_file(file_path.string());
    EXPECT_EQ(actual, expected);
}

TEST(FileLoaderTest, ReadsEmptyFileAsEmptyBuffer) {
    TempDir temp_dir("rtr_file_loader_empty_test");
    const auto file_path = temp_dir.path / "empty.bin";

    std::ofstream out(file_path, std::ios::binary);
    ASSERT_TRUE(out.is_open());
    out.close();

    const auto actual = read_file(file_path.string());
    EXPECT_TRUE(actual.empty());
}

TEST(FileLoaderTest, ThrowsWhenFileDoesNotExist) {
    TempDir temp_dir("rtr_file_loader_missing_test");
    const auto missing_file = temp_dir.path / "missing.bin";

    EXPECT_THROW(
        (void)read_file(missing_file.string()),
        std::runtime_error
    );
}

} // namespace rtr::utils::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
