#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/pbpt/pbpt_scene_export_builder.hpp"
#include "rtr/framework/integration/pbpt/pbpt_scene_importer.hpp"

int main(int argc, char** argv) {
    std::filesystem::path input_path =
        "/Users/jinceyang/Desktop/codebase/graphics/rtr2/external/pbpt/asset/scene/cbox/cbox.xml";
    std::filesystem::path output_path =
        "/Users/jinceyang/Desktop/codebase/graphics/rtr2/external/pbpt/asset/scene/cbox/cbox_rtr_roundtrip_.xml";

    if (argc >= 2) {
        input_path = argv[1];
    }
    if (argc >= 3) {
        output_path = argv[2];
    }

    rtr::framework::core::Scene scene(1, "pbpt_roundtrip");
    const auto import_result = rtr::framework::integration::import_pbpt_scene_xml_to_scene(
        input_path.string(),
        scene
    );

    auto record = rtr::framework::integration::build_pbpt_scene_record(scene);
    if (import_result.integrator.has_value()) {
        record.integrator = import_result.integrator;
    }
    if (import_result.sensor.has_value()) {
        record.sensor = import_result.sensor;
    }

    const std::string xml = rtr::framework::integration::serialize_pbpt_scene_xml(record);

    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream out(output_path);
    if (!out) {
        throw std::runtime_error("Failed to open output file for writing: " + output_path.string());
    }
    out << xml;
    out.close();

    std::cout << "Imported shapes: " << import_result.imported_shape_count << '\n';
    std::cout << "Imported lights: " << import_result.imported_light_shape_count << '\n';
    std::cout << "Wrote roundtrip XML: " << output_path << '\n';
    return 0;
}
