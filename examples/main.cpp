#include <iostream>
#include <cstdlib>

#include "core/application.hpp"

int main() {
    rtr::core::Application app{};
    
    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
