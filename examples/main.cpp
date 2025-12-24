#include <iostream>
#include <cstdlib>

#include "core/application.hpp"
#include "core/context.hpp"
#include "utils/image_loader.hpp"


int main() {
    
    rtr::core::Application app{};

    rtr::utils::Image image{"assets/textures/test.png"};
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
