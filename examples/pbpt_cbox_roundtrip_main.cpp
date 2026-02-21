#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/pbpt/serde/scene_loader.hpp"
#include "rtr/framework/integration/pbpt/serde/scene_writer.hpp"
#include "rtr/resource/resource_manager.hpp"

int main(int argc, char** argv) {
    std::string input_xml_path = "assets/pbpt_scene/cbox/cbox.xml";
    std::string output_path    = "assets/pbpt_scene/cbox/cbox_rtr_roundtrip_.xml";

    if (argc >= 2) {
        input_xml_path = argv[1];
    }
    if (argc >= 3) {
        output_path = argv[2];
    }

    rtr::framework::core::Scene    scene(1, "pbpt_roundtrip");
    rtr::resource::ResourceManager resources(2);
    const auto import_package =
        rtr::framework::integration::load_scene(input_xml_path, scene, resources);
    const auto& import_result = import_package.result;

    uint32_t film_w = import_result.sensor ? import_result.sensor->film_width : 512;
    uint32_t film_h = import_result.sensor ? import_result.sensor->film_height : 512;
    int      spp    = import_result.sensor ? import_result.sensor->sample_count : 16;

    auto result =
        rtr::framework::integration::build_scene_result(scene, resources, nullptr, film_w, film_h, spp);

    rtr::framework::integration::write_scene_result(result, output_path);

    std::cout << "Imported shapes: " << import_result.imported_shape_count << '\n';
    std::cout << "Imported lights: " << import_result.imported_light_shape_count << '\n';
    std::cout << "Wrote roundtrip XML: " << output_path << '\n';
    return 0;
}
