#include <type_traits>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "rtr/system/physics/ipc/energy/energy_concept.hpp"
#include "rtr/system/physics/ipc/energy/gravity_energy.hpp"
#include "rtr/system/physics/ipc/energy/inertial_energy.hpp"

namespace rtr::system::physics::ipc {
namespace {

static_assert(Energy<InertialEnergy>);
static_assert(Energy<GravityEnergy>);

TEST(EnergyConceptTest, DocumentsCurrentEnergyCoverage) {
    EXPECT_TRUE((Energy<InertialEnergy>));
    EXPECT_TRUE((Energy<GravityEnergy>));
}

}  // namespace
}  // namespace rtr::system::physics::ipc
