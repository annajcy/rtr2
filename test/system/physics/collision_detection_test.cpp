#include <cmath>

#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/system/physics/collision.hpp"

namespace rtr::system::physics::test {

TEST(CollisionDetectionTest, SphereSphereReportsIntersectionAndSeparation) {
    Collider a;
    a.shape          = SphereShape{.radius = 0.5f};
    a.world_position = pbpt::math::Vec3{-0.4f, 0.0f, 0.0f};

    Collider b;
    b.shape          = SphereShape{.radius = 0.5f};
    b.world_position = pbpt::math::Vec3{0.4f, 0.0f, 0.0f};

    const auto hit = collide_sphere_sphere(1, a, 2, b);
    ASSERT_TRUE(hit.has_value());
    EXPECT_GT(hit->penetration, 0.0f);
    EXPECT_NEAR(hit->normal.x(), 1.0f, 1e-5f);

    b.world_position = pbpt::math::Vec3{2.0f, 0.0f, 0.0f};
    EXPECT_FALSE(collide_sphere_sphere(1, a, 2, b).has_value());
}

TEST(CollisionDetectionTest, SphereRotatedBoxTopContactProducesFiniteContact) {
    Collider sphere;
    sphere.shape          = SphereShape{.radius = 0.5f};
    sphere.world_position = pbpt::math::Vec3{0.0f, 0.7f, 0.0f};

    Collider box;
    box.shape          = BoxShape{.half_extents = pbpt::math::Vec3{1.0f, 0.25f, 1.0f}};
    box.world_position = pbpt::math::Vec3{0.0f, 0.0f, 0.0f};
    box.world_rotation = pbpt::math::angle_axis(pbpt::math::radians(20.0f), pbpt::math::Vec3{0.0f, 0.0f, 1.0f});

    const auto hit = collide_sphere_box(1, sphere, 2, box);
    ASSERT_TRUE(hit.has_value());
    EXPECT_TRUE(std::isfinite(hit->point.x()));
    EXPECT_TRUE(std::isfinite(hit->point.y()));
    EXPECT_TRUE(std::isfinite(hit->point.z()));
    EXPECT_TRUE(std::isfinite(hit->normal.x()));
    EXPECT_TRUE(std::isfinite(hit->normal.y()));
    EXPECT_TRUE(std::isfinite(hit->normal.z()));
}

TEST(CollisionDetectionTest, SphereRotatedBoxSideContactProducesFiniteContact) {
    Collider sphere;
    sphere.shape          = SphereShape{.radius = 0.5f};
    sphere.world_position = pbpt::math::Vec3{1.0f, 0.0f, 0.0f};

    Collider box;
    box.shape          = BoxShape{.half_extents = pbpt::math::Vec3{0.5f, 1.0f, 1.0f}};
    box.world_position = pbpt::math::Vec3{0.0f, 0.0f, 0.0f};
    box.world_rotation = pbpt::math::angle_axis(pbpt::math::radians(35.0f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f});

    const auto hit = collide_sphere_box(1, sphere, 2, box);
    ASSERT_TRUE(hit.has_value());
    EXPECT_GT(pbpt::math::length(hit->normal), 0.9f);
}

TEST(CollisionDetectionTest, DegenerateSphereAndInsideBoxContactsStayFinite) {
    Collider sphere_a;
    sphere_a.shape          = SphereShape{.radius = 0.5f};
    sphere_a.world_position = pbpt::math::Vec3{0.0f, 0.0f, 0.0f};

    Collider sphere_b = sphere_a;
    const auto sphere_hit = collide_sphere_sphere(1, sphere_a, 2, sphere_b);
    ASSERT_TRUE(sphere_hit.has_value());
    EXPECT_TRUE(std::isfinite(sphere_hit->normal.x()));
    EXPECT_TRUE(std::isfinite(sphere_hit->normal.y()));
    EXPECT_TRUE(std::isfinite(sphere_hit->normal.z()));

    Collider box;
    box.shape          = BoxShape{.half_extents = pbpt::math::Vec3{1.0f, 1.0f, 1.0f}};
    box.world_position = pbpt::math::Vec3{0.0f, 0.0f, 0.0f};
    const auto box_hit = collide_sphere_box(1, sphere_a, 2, box);
    ASSERT_TRUE(box_hit.has_value());
    EXPECT_TRUE(std::isfinite(box_hit->normal.x()));
    EXPECT_TRUE(std::isfinite(box_hit->normal.y()));
    EXPECT_TRUE(std::isfinite(box_hit->normal.z()));
}

}  // namespace rtr::system::physics::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
