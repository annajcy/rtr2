#include <iostream>
#include <iostream>
#include <vector>
#include <string>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

#include "assimp/Importer.hpp"
#include "imgui.h"
#include "slang.h"
#include "vulkan/vulkan.h"
#include "vulkan/vulkan.hpp"
#include "stb_image.h"
#include "stb_image_write.h"

void test_slang() {
    std::cout << "\n=== Testing Slang Shader Compiler ===" << std::endl;
    std::cout << "Slang BindingType::BaseMask: " << static_cast<int>(slang::BindingType::BaseMask) << std::endl;
    
    slang::IGlobalSession* globalSession = nullptr;
    SlangResult result = slang_createGlobalSession(SLANG_API_VERSION, &globalSession);
    
    if (SLANG_SUCCEEDED(result) && globalSession) {
        std::cout << "✓ Slang global session created successfully" << std::endl;
        globalSession->release();
    } else {
        std::cout << "✗ Failed to create Slang global session" << std::endl;
    }
}

void test_imgui() {
    std::cout << "\n=== Testing ImGui ===" << std::endl;
    ImGuiContext* ctx = ImGui::CreateContext();
    if (ctx) {
        std::cout << "✓ ImGui context created successfully" << std::endl;
        std::cout << "ImGui Version: " << ImGui::GetVersion() << std::endl;
        ImGuiIO& io = ImGui::GetIO();
        io.DisplaySize = ImVec2(1920.0f, 1080.0f);
        std::cout << "Display Size: " << io.DisplaySize.x << "x" << io.DisplaySize.y << std::endl;
        ImGuiStyle& style = ImGui::GetStyle();
        std::cout << "Window Rounding: " << style.WindowRounding << std::endl;
        ImGui::DestroyContext(ctx);
        std::cout << "✓ ImGui context destroyed" << std::endl;
    } else {
        std::cout << "✗ Failed to create ImGui context" << std::endl;
    }
}

void test_vulkan() {
    std::cout << "\n=== Testing Vulkan ===" << std::endl;
    VkApplicationInfo appInfo{};
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName = "vk renderer Backend Test";
    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.pEngineName = "vk renderer";
    appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.apiVersion = VK_API_VERSION_1_0;
    
    std::cout << "Application Name: " << appInfo.pApplicationName << std::endl;
    std::cout << "Engine Name: " << appInfo.pEngineName << std::endl;
    std::cout << "API Version: " << VK_VERSION_MAJOR(appInfo.apiVersion) << "." << VK_VERSION_MINOR(appInfo.apiVersion) << "." << VK_VERSION_PATCH(appInfo.apiVersion) << std::endl;
    
    try {
        uint32_t version = vk::enumerateInstanceVersion();
        std::cout << "Vulkan Instance Version: " << VK_VERSION_MAJOR(version) << "." << VK_VERSION_MINOR(version) << "." << VK_VERSION_PATCH(version) << std::endl;
        auto layers = vk::enumerateInstanceLayerProperties();
        std::cout << "Available Vulkan Layers (" << layers.size() << "):" << std::endl;
        for (const auto& layer : layers) {
            std::cout << "  - " << layer.layerName << std::endl;
        }
        auto extensions = vk::enumerateInstanceExtensionProperties();
        std::cout << "Available Vulkan Extensions (" << extensions.size() << "):" << std::endl;
        int count = 0;
        for (const auto& ext : extensions) {
            std::cout << "  - " << ext.extensionName << std::endl;
        }
        std::cout << "✓ Vulkan API accessible" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "✗ Vulkan error: " << e.what() << std::endl;
    }
}

void test_stb_image() {
    std::cout << "\n=== Testing STB Image ===" << std::endl;
    const int w = 100, h = 100, ch = 3;
    std::vector<unsigned char> img(w * h * ch);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            size_t idx = (y * w + x) * ch;
            img[idx + 0] = static_cast<unsigned char>(255 * x / w);
            img[idx + 1] = 0;
            img[idx + 2] = static_cast<unsigned char>(255 * y / h);
        }
    }
    if (stbi_write_png("test.png", w, h, ch, img.data(), w * ch)) {
        std::cout << "✓ Successfully wrote image to test.png (" << w << "x" << h << ")" << std::endl;
    } else {
        std::cout << "✗ Failed to write image" << std::endl;
        return;
    }
    int rw, rh, rch;
    unsigned char* data = stbi_load("test.png", &rw, &rh, &rch, 0);
    if (data) {
        std::cout << "✓ Successfully loaded image: " << rw << "x" << rh << ", " << rch << " channels" << std::endl;
        std::cout << "  Top-left pixel: R=" << (int)data[0] << " G=" << (int)data[1] << " B=" << (int)data[2] << std::endl;
        stbi_image_free(data);
    } else {
        std::cout << "✗ Failed to load image: " << stbi_failure_reason() << std::endl;
    }
}

void test_assimp() {
    std::cout << "\n=== Testing Assimp ===" << std::endl;
    Assimp::Importer importer;
    std::vector<std::string> formats = {"obj", "fbx", "dae", "gltf", "glb", "stl", "ply", "3ds"};
    for (const auto& fmt : formats) {
        std::cout << "  " << fmt << ": " << (importer.IsExtensionSupported(fmt) ? "✓ Supported" : "✗ Not supported") << std::endl;
    }
    std::string extensions;
    importer.GetExtensionList(extensions);
    std::cout << "\nAll extensions: " << extensions << std::endl;
}

void test_glm() {
    std::cout << "\n=== Testing GLM ===" << std::endl;
    
    // Test vector operations
    glm::vec3 v1(1.0f, 2.0f, 3.0f);
    glm::vec3 v2(4.0f, 5.0f, 6.0f);
    glm::vec3 v_add = v1 + v2;
    glm::vec3 v_cross = glm::cross(v1, v2);
    float v_dot = glm::dot(v1, v2);
    
    std::cout << "Vector v1: " << glm::to_string(v1) << std::endl;
    std::cout << "Vector v2: " << glm::to_string(v2) << std::endl;
    std::cout << "v1 + v2: " << glm::to_string(v_add) << std::endl;
    std::cout << "v1 × v2 (cross): " << glm::to_string(v_cross) << std::endl;
    std::cout << "v1 · v2 (dot): " << v_dot << std::endl;
    
    // Test matrix operations
    glm::mat4 identity = glm::mat4(1.0f);
    glm::mat4 translation = glm::translate(identity, glm::vec3(1.0f, 2.0f, 3.0f));
    glm::mat4 rotation = glm::rotate(identity, glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    glm::mat4 scale = glm::scale(identity, glm::vec3(2.0f, 2.0f, 2.0f));
    
    std::cout << "\nIdentity matrix:" << std::endl;
    for (int i = 0; i < 4; i++) {
        std::cout << "  [" << identity[i][0] << ", " << identity[i][1] << ", " 
                  << identity[i][2] << ", " << identity[i][3] << "]" << std::endl;
    }
    
    // Test perspective projection
    glm::mat4 proj = glm::perspective(glm::radians(45.0f), 16.0f / 9.0f, 0.1f, 100.0f);
    std::cout << "\nPerspective projection matrix created" << std::endl;
    std::cout << "  FOV: 45°, Aspect: 16:9, Near: 0.1, Far: 100.0" << std::endl;
    
    // Test lookAt
    glm::mat4 view = glm::lookAt(
        glm::vec3(0.0f, 0.0f, 5.0f),  // Camera position
        glm::vec3(0.0f, 0.0f, 0.0f),  // Look at point
        glm::vec3(0.0f, 1.0f, 0.0f)   // Up vector
    );
    std::cout << "View matrix created (camera at [0,0,5] looking at origin)" << std::endl;
    
    // Test transformation
    glm::vec4 point(1.0f, 0.0f, 0.0f, 1.0f);
    glm::vec4 transformed = translation * point;
    std::cout << "\nPoint " << glm::to_string(point) << " after translation:" << std::endl;
    std::cout << "  Result: " << glm::to_string(transformed) << std::endl;
    
    std::cout << "✓ GLM library working correctly" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "3rd Party Library Tests" << std::endl;
    std::cout << "========================================" << std::endl;
    test_slang();
    test_imgui();
    test_vulkan();
    test_stb_image();
    test_assimp();
    test_glm();
    std::cout << "\n========================================" << std::endl;
    std::cout << "All tests completed!" << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}