#include <iostream>
#include <cstdlib>

#include "vk_application.hpp"
#include "image_loader.hpp"

int main() {
    
    rtr::VKApplication app{};

    rtr::Image image{"assets/textures/test.png"};
    std::cout << 
        "Loaded image with dimensions: " << image.width() << "x" << image.height() << 
        " channels: " << image.channels() << 
        " data_size: " << image.data_size() << std::endl;

    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
