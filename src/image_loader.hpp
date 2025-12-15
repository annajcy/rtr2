#include "stb_image.h"

#include <cstdint>
#include <iostream>
#include <string>
#include <optional>
#include <tuple>
#include <utility>

namespace rtr {

class Image{
private:
    int m_width;
    int m_height;
    int m_channels;
    uint32_t m_data_size;
    uint8_t* m_data;

public:
    Image(const std::string& filepath, bool flip_vertically = true, int desired_channels = 4) {
        stbi_set_flip_vertically_on_load(flip_vertically);
        
        int original_channels = 0;
        m_data = stbi_load(filepath.c_str(), 
            &m_width, &m_height, &original_channels, 
            desired_channels
        );
        
        if (!m_data) {
            throw std::runtime_error("Failed to load image: " + filepath);
        }
        
        // 如果指定了 desired_channels，stbi_load 会自动转换
        m_channels = (desired_channels != 0) ? desired_channels : original_channels;
        
        if (original_channels != m_channels) {
            std::cout << "Image converted from " << original_channels 
                      << " to " << m_channels << " channels: " << filepath << std::endl;
        }
        
        m_data_size = m_width * m_height * m_channels;
    }

    ~Image() {
        if (m_data) {
            stbi_image_free(m_data);
        }
    }

    const int& width() const { return m_width; }
    const int& height() const { return m_height; }
    const int& channels() const { return m_channels; }
    const uint32_t& data_size() const { return m_data_size; }

    const uint8_t* data() const { return m_data; }
    uint8_t* data() { return m_data; }
};

}