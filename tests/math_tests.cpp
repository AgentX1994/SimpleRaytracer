#include <gtest/gtest.h>

#include "math.hpp"

TEST(MathTests, SimpleQuadratic)
{
    double x0, x1;
    auto res = raytracer::SolveQuadratic(1.0, -1.0, -6.0, x0, x1);
    ASSERT_TRUE(res);
    ASSERT_FLOAT_EQ(x0, -2.0);
    ASSERT_FLOAT_EQ(x1, 3.0);
}

TEST(MathTests, SingularQuadratic)
{
    double x0, x1;
    auto res = raytracer::SolveQuadratic(1.0, -6.0, 9.0, x0, x1);
    ASSERT_TRUE(res);
    ASSERT_FLOAT_EQ(x0, 3);
    ASSERT_FLOAT_EQ(x1, 3);
}

TEST(MathTests, UnsolvableQuadratic)
{
    double x0, x1;
    auto res = raytracer::SolveQuadratic(1.0, -2.0, 10.0, x0, x1);
    ASSERT_FALSE(res);
}

TEST(MathTests, Vec3)
{
    {
        raytracer::Vec3<double> v;
        ASSERT_EQ(v.x(), 0.0);
        ASSERT_EQ(v.y(), 0.0);
        ASSERT_EQ(v.z(), 0.0);
        ASSERT_EQ(v.Length(), 0.0);
        ASSERT_EQ(v.LengthSquared(), 0.0);
        ASSERT_FALSE(v.IsUnit());
    }
    {
        raytracer::Vec3<double> v(1.0, 0.0, 0.0);
        ASSERT_EQ(v.x(), 1.0);
        ASSERT_EQ(v.y(), 0.0);
        ASSERT_EQ(v.z(), 0.0);
        ASSERT_EQ(v.Length(), 1.0);
        ASSERT_EQ(v.LengthSquared(), 1.0);
        ASSERT_TRUE(v.IsUnit());
    }
    {
        raytracer::Vec3<double> v(1.0, 1.0, 1.0);
        ASSERT_EQ(v.x(), 1.0);
        ASSERT_EQ(v.y(), 1.0);
        ASSERT_EQ(v.z(), 1.0);
        ASSERT_FLOAT_EQ(v.Length(), std::sqrt(3));
        ASSERT_FLOAT_EQ(v.LengthSquared(), 3.0);
        ASSERT_FALSE(v.IsUnit());
    }
    {
        raytracer::Vec3<double> v1(1.0, 2.0, 3.0);
        raytracer::Vec3<double> v2(4.0, 5.0, 6.0);
        ASSERT_FLOAT_EQ(raytracer::Dot(v1, v2), 32.0);
        auto v_cross = raytracer::Cross(v1, v2);
        ASSERT_FLOAT_EQ(v_cross.x(), -3.0);
        ASSERT_FLOAT_EQ(v_cross.y(), 6.0);
        ASSERT_FLOAT_EQ(v_cross.z(), -3.0);
    }
}
