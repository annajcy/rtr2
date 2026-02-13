#include "rtr/framework/integration/pbpt_offline_render_service.hpp"

int main() {
    rtr::framework::integration::PbptOfflineRenderService service;
    return service.is_running() ? 1 : 0;
}
