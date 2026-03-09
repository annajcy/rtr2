#include <cmath>
#include <type_traits>

#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/system/physics/collision.hpp"

namespace rtr::system::physics::test {

TEST(CollisionDetectionTest, SphereSphereReportsIntersectionAndSeparation) {
    const WorldSphere a{
        .center = pbpt::math::Vec3{-0.4f, 0.0f, 0.0f},
        .radius = 0.5f,
    };
    auto b = WorldSphere{
        .center = pbpt::math::Vec3{0.4f, 0.0f, 0.0f},
        .radius = 0.5f,
    };

    const auto hit = ContactPairTrait<WorldSphere, WorldSphere>::generate(a, b);
    ASSERT_TRUE(hit.is_valid());
    EXPECT_GT(hit.penetration, 0.0f);
    EXPECT_NEAR(hit.normal.x(), 1.0f, 1e-5f);

    b.center = pbpt::math::Vec3{2.0f, 0.0f, 0.0f};
    EXPECT_FALSE((ContactPairTrait<WorldSphere, WorldSphere>::generate(a, b).is_valid()));
}

TEST(CollisionDetectionTest, SphereRotatedBoxContactsProduceFiniteValues) {
    const WorldSphere sphere{
        .center = pbpt::math::Vec3{0.0f, 0.7f, 0.0f},
        .radius = 0.5f,
    };
    const WorldBox box{
        .center = pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        .rotation = pbpt::math::Quat::from_axis_angle(pbpt::math::radians(20.0f), pbpt::math::Vec3{0.0f, 0.0f, 1.0f}),
        .half_extents = pbpt::math::Vec3{1.0f, 0.25f, 1.0f},
    };

    const auto hit = ContactPairTrait<WorldSphere, WorldBox>::generate(sphere, box);
    ASSERT_TRUE(hit.is_valid());
    EXPECT_TRUE(std::isfinite(hit.point.x()));
    EXPECT_TRUE(std::isfinite(hit.point.y()));
    EXPECT_TRUE(std::isfinite(hit.point.z()));
    EXPECT_TRUE(std::isfinite(hit.normal.x()));
    EXPECT_TRUE(std::isfinite(hit.normal.y()));
    EXPECT_TRUE(std::isfinite(hit.normal.z()));
}

TEST(CollisionDetectionTest, DegenerateContactsStayFinite) {
    const WorldSphere sphere{
        .center = pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        .radius = 0.5f,
    };

    const auto sphere_hit = ContactPairTrait<WorldSphere, WorldSphere>::generate(sphere, sphere);
    ASSERT_TRUE(sphere_hit.is_valid());
    EXPECT_TRUE(std::isfinite(sphere_hit.normal.x()));
    EXPECT_TRUE(std::isfinite(sphere_hit.normal.y()));
    EXPECT_TRUE(std::isfinite(sphere_hit.normal.z()));

    const WorldBox box{
        .center = pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        .rotation = pbpt::math::Quat::identity(),
        .half_extents = pbpt::math::Vec3{1.0f, 1.0f, 1.0f},
    };
    const auto box_hit = ContactPairTrait<WorldSphere, WorldBox>::generate(sphere, box);
    ASSERT_TRUE(box_hit.is_valid());
    EXPECT_TRUE(std::isfinite(box_hit.normal.x()));
    EXPECT_TRUE(std::isfinite(box_hit.normal.y()));
    EXPECT_TRUE(std::isfinite(box_hit.normal.z()));
}

TEST(CollisionDetectionTest, ReversedSphereBoxPairFlipsNormalAndPreservesPenetration) {
    const WorldSphere sphere{
        .center = pbpt::math::Vec3{1.3f, 0.0f, 0.0f},
        .radius = 0.5f,
    };
    const WorldBox box{
        .center = pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        .rotation = pbpt::math::Quat::identity(),
        .half_extents = pbpt::math::Vec3{1.0f, 1.0f, 1.0f},
    };

    const auto forward = ContactPairTrait<WorldSphere, WorldBox>::generate(sphere, box);
    const auto reverse = ContactPairTrait<WorldBox, WorldSphere>::generate(box, sphere);

    ASSERT_TRUE(forward.is_valid());
    ASSERT_TRUE(reverse.is_valid());
    EXPECT_NEAR(forward.penetration, reverse.penetration, 1e-5f);
    EXPECT_NEAR(forward.normal.x(), -reverse.normal.x(), 1e-5f);
    EXPECT_NEAR(forward.normal.y(), -reverse.normal.y(), 1e-5f);
    EXPECT_NEAR(forward.normal.z(), -reverse.normal.z(), 1e-5f);
}

TEST(CollisionDetectionTest, BoxBoxReportsIntersectionAndSeparation) {
    const WorldBox a{
        .center = pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        .rotation = pbpt::math::Quat::identity(),
        .half_extents = pbpt::math::Vec3{1.0f, 1.0f, 1.0f},
    };
    auto b = WorldBox{
        .center = pbpt::math::Vec3{1.5f, 0.0f, 0.0f},
        .rotation = pbpt::math::Quat::identity(),
        .half_extents = pbpt::math::Vec3{1.0f, 1.0f, 1.0f},
    };

    const auto hit = ContactPairTrait<WorldBox, WorldBox>::generate(a, b);
    ASSERT_TRUE(hit.is_valid());
    EXPECT_GT(hit.penetration, 0.0f);
    EXPECT_NEAR(hit.normal.x(), 1.0f, 1e-5f);
    EXPECT_NEAR(hit.normal.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(hit.normal.z(), 0.0f, 1e-5f);

    b.center = pbpt::math::Vec3{3.0f, 0.0f, 0.0f};
    EXPECT_FALSE((ContactPairTrait<WorldBox, WorldBox>::generate(a, b).is_valid()));
}

TEST(CollisionDetectionTest, RotatedBoxBoxProducesFiniteContact) {
    const WorldBox a{
        .center = pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        .rotation = pbpt::math::Quat::from_axis_angle(pbpt::math::radians(20.0f), pbpt::math::Vec3{0.0f, 0.0f, 1.0f}),
        .half_extents = pbpt::math::Vec3{1.0f, 0.5f, 0.75f},
    };
    const WorldBox b{
        .center = pbpt::math::Vec3{1.1f, 0.1f, 0.0f},
        .rotation = pbpt::math::Quat::from_axis_angle(pbpt::math::radians(-15.0f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}),
        .half_extents = pbpt::math::Vec3{0.8f, 0.6f, 0.7f},
    };

    const auto hit = ContactPairTrait<WorldBox, WorldBox>::generate(a, b);
    ASSERT_TRUE(hit.is_valid());
    EXPECT_TRUE(std::isfinite(hit.point.x()));
    EXPECT_TRUE(std::isfinite(hit.point.y()));
    EXPECT_TRUE(std::isfinite(hit.point.z()));
    EXPECT_TRUE(std::isfinite(hit.normal.x()));
    EXPECT_TRUE(std::isfinite(hit.normal.y()));
    EXPECT_TRUE(std::isfinite(hit.normal.z()));
    EXPECT_TRUE(std::isfinite(hit.penetration));
}

TEST(CollisionDetectionTest, BoxBoxSymmetryPreservesPenetrationAndFlipsNormal) {
    const WorldBox a{
        .center = pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        .rotation = pbpt::math::Quat::identity(),
        .half_extents = pbpt::math::Vec3{1.0f, 1.0f, 1.0f},
    };
    const WorldBox b{
        .center = pbpt::math::Vec3{1.4f, 0.3f, 0.0f},
        .rotation = pbpt::math::Quat::from_axis_angle(pbpt::math::radians(10.0f), pbpt::math::Vec3{0.0f, 0.0f, 1.0f}),
        .half_extents = pbpt::math::Vec3{0.9f, 0.8f, 1.0f},
    };

    const auto forward = ContactPairTrait<WorldBox, WorldBox>::generate(a, b);
    const auto reverse = ContactPairTrait<WorldBox, WorldBox>::generate(b, a);

    ASSERT_TRUE(forward.is_valid());
    ASSERT_TRUE(reverse.is_valid());
    EXPECT_NEAR(forward.penetration, reverse.penetration, 1e-5f);
    EXPECT_NEAR(forward.normal.x(), -reverse.normal.x(), 1e-5f);
    EXPECT_NEAR(forward.normal.y(), -reverse.normal.y(), 1e-5f);
    EXPECT_NEAR(forward.normal.z(), -reverse.normal.z(), 1e-5f);
}

TEST(CollisionDetectionTest, BoxBoxNearParallelEdgesStayStable) {
    const WorldBox a{
        .center = pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        .rotation = pbpt::math::Quat::from_axis_angle(pbpt::math::radians(0.5f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}),
        .half_extents = pbpt::math::Vec3{1.0f, 0.2f, 0.2f},
    };
    const WorldBox b{
        .center = pbpt::math::Vec3{1.6f, 0.0f, 0.05f},
        .rotation = pbpt::math::Quat::from_axis_angle(pbpt::math::radians(1.0f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}),
        .half_extents = pbpt::math::Vec3{1.0f, 0.2f, 0.2f},
    };

    const auto hit = ContactPairTrait<WorldBox, WorldBox>::generate(a, b);
    ASSERT_TRUE(hit.is_valid());
    EXPECT_TRUE(std::isfinite(hit.normal.x()));
    EXPECT_TRUE(std::isfinite(hit.normal.y()));
    EXPECT_TRUE(std::isfinite(hit.normal.z()));
    EXPECT_TRUE(std::isfinite(hit.penetration));
}

TEST(CollisionDetectionTest, BoxBoxSkewedEdgesStayFiniteWhenDetectionMisses) {
    const WorldBox a{
        .center = pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        .rotation = pbpt::math::Quat::from_axis_angle(pbpt::math::radians(35.0f), pbpt::math::Vec3{0.0f, 0.0f, 1.0f}),
        .half_extents = pbpt::math::Vec3{1.0f, 0.2f, 0.2f},
    };
    const WorldBox b{
        .center = pbpt::math::Vec3{0.1f, 0.9f, 0.15f},
        .rotation = pbpt::math::Quat::from_axis_angle(pbpt::math::radians(70.0f), pbpt::math::Vec3{1.0f, 0.0f, 0.0f}),
        .half_extents = pbpt::math::Vec3{0.2f, 1.0f, 0.2f},
    };

    const auto hit = ContactPairTrait<WorldBox, WorldBox>::generate(a, b);
    EXPECT_TRUE(std::isfinite(hit.point.x()));
    EXPECT_TRUE(std::isfinite(hit.point.y()));
    EXPECT_TRUE(std::isfinite(hit.point.z()));
    EXPECT_TRUE(std::isfinite(hit.normal.x()));
    EXPECT_TRUE(std::isfinite(hit.normal.y()));
    EXPECT_TRUE(std::isfinite(hit.normal.z()));
    EXPECT_TRUE(std::isfinite(hit.penetration));
}

TEST(CollisionDetectionTest, VariantVisitDispatchUsesContactPairTrait) {
    const WorldCollider collider_a = WorldSphere{
        .center = pbpt::math::Vec3{1.3f, 0.0f, 0.0f},
        .radius = 0.5f,
    };
    const WorldCollider collider_b = WorldBox{
        .center = pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        .rotation = pbpt::math::Quat::identity(),
        .half_extents = pbpt::math::Vec3{1.0f, 1.0f, 1.0f},
    };

    ContactResult result{};
    std::visit(
        [&](auto&& shape_a, auto&& shape_b) {
            result = ContactPairTrait<
                std::decay_t<decltype(shape_a)>,
                std::decay_t<decltype(shape_b)>>::generate(shape_a, shape_b);
        },
        collider_a,
        collider_b);

    EXPECT_TRUE(result.is_valid());
    EXPECT_GT(result.penetration, 0.0f);
}

TEST(CollisionDetectionTest, VariantVisitDispatchUsesBoxBoxTrait) {
    const WorldCollider collider_a = WorldBox{
        .center = pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
        .rotation = pbpt::math::Quat::identity(),
        .half_extents = pbpt::math::Vec3{1.0f, 1.0f, 1.0f},
    };
    const WorldCollider collider_b = WorldBox{
        .center = pbpt::math::Vec3{1.5f, 0.0f, 0.0f},
        .rotation = pbpt::math::Quat::identity(),
        .half_extents = pbpt::math::Vec3{1.0f, 1.0f, 1.0f},
    };

    ContactResult result{};
    std::visit(
        [&](auto&& shape_a, auto&& shape_b) {
            result = ContactPairTrait<
                std::decay_t<decltype(shape_a)>,
                std::decay_t<decltype(shape_b)>>::generate(shape_a, shape_b);
        },
        collider_a,
        collider_b);

    EXPECT_TRUE(result.is_valid());
    EXPECT_GT(result.penetration, 0.0f);
}

}  // namespace rtr::system::physics::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
