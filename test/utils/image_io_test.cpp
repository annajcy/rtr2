#include <filesystem>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "rtr/utils/image_io.hpp"

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

ImageData make_rgba_2x1() {
    ImageData image{};
    image.width = 2;
    image.height = 1;
    image.channels = 4;
    image.pixels = {
        255, 0, 0, 255,
        0, 255, 0, 255,
    };
    return image;
}

} // namespace

TEST(ImageIoTest, WritesAndReadsPng) {
    TempDir temp_dir("rtr_image_io_png_test");
    const auto path = temp_dir.path / "image.png";

    write_image_to_path(make_rgba_2x1(), path.string());
    const auto loaded = load_image_from_path(path.string(), false, 4);
    ASSERT_EQ(loaded.width, 2u);
    ASSERT_EQ(loaded.height, 1u);
    ASSERT_EQ(loaded.channels, 4u);
    ASSERT_EQ(loaded.pixels.size(), 8u);
}

TEST(ImageIoTest, WritesAndReadsPpm) {
    TempDir temp_dir("rtr_image_io_ppm_test");
    const auto path = temp_dir.path / "image.ppm";

    write_image_to_path(make_rgba_2x1(), path.string());
    const auto loaded = load_image_from_path(path.string(), false, 4);
    ASSERT_EQ(loaded.width, 2u);
    ASSERT_EQ(loaded.height, 1u);
    ASSERT_EQ(loaded.channels, 4u);
}

TEST(ImageIoTest, FlipYSwapsRowsOnLoad) {
    TempDir temp_dir("rtr_image_io_flip_test");
    const auto path = temp_dir.path / "flip.ppm";

    ImageData image{};
    image.width = 1;
    image.height = 2;
    image.channels = 3;
    // Top row red, bottom row blue.
    image.pixels = {
        255, 0, 0,
        0, 0, 255,
    };

    write_image_to_path(image, path.string());
    const auto no_flip = load_image_from_path(path.string(), false, 3);
    const auto flipped = load_image_from_path(path.string(), true, 3);

    ASSERT_EQ(no_flip.pixels.size(), 6u);
    ASSERT_EQ(flipped.pixels.size(), 6u);

    EXPECT_EQ(no_flip.pixels[0], 255u);
    EXPECT_EQ(no_flip.pixels[1], 0u);
    EXPECT_EQ(no_flip.pixels[2], 0u);

    EXPECT_EQ(flipped.pixels[0], 0u);
    EXPECT_EQ(flipped.pixels[1], 0u);
    EXPECT_EQ(flipped.pixels[2], 255u);
}

TEST(ImageIoTest, UnsupportedExtensionThrows) {
    TempDir temp_dir("rtr_image_io_invalid_ext_test");
    const auto path = temp_dir.path / "image.bmp";
    EXPECT_THROW((void)write_image_to_path(make_rgba_2x1(), path.string()), std::invalid_argument);
}

} // namespace rtr::utils::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
