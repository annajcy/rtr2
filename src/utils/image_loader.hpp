#pragma once

#include "stb_image.h"

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include <cstring>

namespace rtr::utils {
    
class ImageLoader{
private:
    int m_width{0};
    int m_height{0};
    int m_channels{0};
    uint32_t m_data_size{0};
    uint8_t* m_data{nullptr};
    bool m_is_flipped_y{false};

public:
    // 1. 禁用拷贝构造和拷贝赋值（防止 Double Free）
    ImageLoader(const ImageLoader&) = delete;
    ImageLoader& operator=(const ImageLoader&) = delete;

    ImageLoader(ImageLoader&& other) noexcept 
        : m_width(other.m_width), m_height(other.m_height), 
          m_channels(other.m_channels), m_data_size(other.m_data_size), 
          m_data(other.m_data), m_is_flipped_y(other.m_is_flipped_y) {
        // 让源对象交出所有权，变成空壳
        other.m_data = nullptr; 
        other.m_data_size = 0;
        other.m_is_flipped_y = false;
    }

    // 3. 实现移动赋值 (Move Assignment)
    ImageLoader& operator=(ImageLoader&& other) noexcept {
        if (this != &other) {
            // 先释放自己的旧内存
            if (m_data) stbi_image_free(m_data);
            
            // 窃取对方的数据
            m_width = other.m_width;
            m_height = other.m_height;
            m_channels = other.m_channels;
            m_data_size = other.m_data_size;
            m_data = other.m_data;
            m_is_flipped_y = other.m_is_flipped_y;
            
            // 置空对方
            other.m_data = nullptr;
            other.m_data_size = 0;
            other.m_width = 0;
            other.m_height = 0;
            other.m_channels = 0;
            other.m_is_flipped_y = false;
        }
        return *this;
    }

    ImageLoader(const std::string& file_path, bool is_flip_y = true, int desired_channels = 4) {
        int original_channels = 0;
        m_data = stbi_load(file_path.c_str(), 
            &m_width, &m_height, &original_channels, 
            desired_channels
        );
        
        if (!m_data) {
            throw std::runtime_error("Failed to load image: " + file_path);
        }
        
        // 如果指定了 desired_channels，stbi_load 会自动转换
        m_channels = (desired_channels != 0) ? desired_channels : original_channels;
        
        if (original_channels != m_channels) {
            std::cout << "Image converted from " << original_channels 
                      << " to " << m_channels << " channels: " << file_path << std::endl;
        }
        
        m_data_size = m_width * m_height * m_channels;

        if (is_flip_y) {
            flip_y();
        }
    }

    ~ImageLoader() {
        if (m_data) {
            stbi_image_free(m_data);
        }
    }

    int width() const { return m_width; }
    int height() const { return m_height; }
    int channels() const { return m_channels; }
    uint32_t data_size() const { return m_data_size; }
    const uint8_t* data() const { return m_data; }
    bool is_flipped_y() const { return m_is_flipped_y; }

private:
    void flip_y() {
        if (!m_data) return;
        m_is_flipped_y = !m_is_flipped_y;

        int row_size = m_width * m_channels;
        std::vector<uint8_t> temp_row(row_size);

        for (int y = 0; y < m_height / 2; ++y) {
            uint8_t* row_top = m_data + y * row_size;
            uint8_t* row_bottom = m_data + (m_height - 1 - y) * row_size;

            // 交换行
            std::memcpy(temp_row.data(), row_top, row_size);
            std::memcpy(row_top, row_bottom, row_size);
            std::memcpy(row_bottom, temp_row.data(), row_size);
        }
    }
};

};