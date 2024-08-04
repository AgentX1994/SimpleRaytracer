#pragma once

#include <array>
#include <concepts>
#include <cassert>

namespace raytracer
{
    template <std::floating_point T>
    bool SolveQuadratic(T a, T b, T c, T &x0, T &x1)
    {
        T discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0)
        {
            return false;
        }
        else if (discriminant == 0.0)
        {
            x0 = -0.5 * b / a;
            x1 = x0;
        }
        else
        {
            T q = (b > 0.0) ? -0.5 * (b + std::sqrt(discriminant))
                            : -0.5 * (b - std::sqrt(discriminant));
            x0 = q / a;
            x1 = c / q;
        }
        if (x0 > x1)
        {
            std::swap(x0, x1);
        }
        return true;
    }

    template <std::floating_point T>
    class Vec3
    {
    public:
        Vec3() : vec({0.0}) {}
        Vec3(T x, T y, T z) : Vec3()
        {
            vec[0] = x;
            vec[1] = y;
            vec[2] = z;
        }

        T x() { return vec[0]; }
        T y() { return vec[1]; }
        T z() { return vec[2]; }

        T LengthSquared()
        {
            return x() * x() + y() * y() + z() * z();
        }

        T Length()
        {
            return std::sqrt(LengthSquared());
        }

        bool IsUnit()
        {
            return Length() == 1.0;
        }

        Vec3<T> ToUnit()
        {
            auto len = Length();
            return Vec3(
                x() / len,
                y() / len,
                z() / len);
        }

    private:
        std::array<T, 3> vec;
    };

    template <std::floating_point T>
    using Point3 = Vec3<T>;

    template <std::floating_point T>
    inline Vec3<T> operator+(Vec3<T> left, Vec3<T> right)
    {
        T x = left.x() + right.x();
        T y = left.y() + right.y();
        T z = left.z() + right.z();
        return Vec3(x, y, z);
    }

    template <std::floating_point T>
    inline Vec3<T> operator-(Vec3<T> left, Vec3<T> right)
    {
        T x = left.x() - right.x();
        T y = left.y() - right.y();
        T z = left.z() - right.z();
        return Vec3(x, y, z);
    }

    template <std::floating_point T>
    inline Vec3<T> cross(Vec3<T> left, Vec3<T> right)
    {
        T x = left.y() * right.z() - left.z() * right.y();
        T y = left.z() * right.x() - left.x() * right.z();
        T z = left.x() * right.y() - left.y() * right.x();
        return Vec3(x, y, z);
    }

    template <std::floating_point T>
    inline T dot(Vec3<T> left, Vec3<T> right)
    {
        T xs = left.x() * right.x();
        T ys = left.y() * right.y();
        T zs = left.z() * right.z();
        return xs + ys + zs;
    }

    template <std::floating_point T>
    class Ray
    {
    public:
        Ray(Point3<T> origin, Vec3<T> direction) : origin(origin), direction(direction) {}

        Point3<T> origin;
        Vec3<T> direction;
    };

    class Shape;

    template <std::floating_point T>
    struct IntersectionRecord
    {
        T t = std::numeric_limits<T>::infinity();
        Shape *shape = nullptr;
        Ray<T> *ray = nullptr;
    };

    class Shape
    {
    public:
        virtual bool intersect(Ray<float> *ray, float max_distance, IntersectionRecord<float> &record) = 0;
        virtual bool intersect(Ray<double> *ray, double max_distance, IntersectionRecord<double> &record) = 0;
    };

    template <std::floating_point T>
    class Sphere : public Shape
    {
    public:
        Sphere(Point3<T> center, T radius) : center(center), radius(radius) {}

        bool intersect(Ray<float> *ray, float max_distance, IntersectionRecord<float> &record) override
        {
            if constexpr (std::is_same_v<float, T>)
            {
                return intersect_inner(ray, max_distance, record);
            }
            else
            {
                return false;
            }
        }
        bool intersect(Ray<double> *ray, double max_distance, IntersectionRecord<double> &record) override
        {
            return intersect_inner(ray, max_distance, record);
        }

    private:
        bool intersect_inner(Ray<T> *ray, T max_distance, IntersectionRecord<T> &record)
        {
            auto diff = ray->origin - center;

            T a = dot(ray->direction, ray->direction);
            T b = 2.0 * dot(ray->direction, diff);
            T c = diff.LengthSquared() - radius * radius;

            T t0, t1;
            if (!SolveQuadratic(a, b, c, t0, t1))
            {
                return false;
            }

            if (t0 < 0.0)
            {
                t0 = t1;
                if (t0 < 0.0)
                {
                    return false;
                }
            }

            record.t = t0;
            record.ray = ray;
            record.shape = this;
            return true;
        }

        Point3<T> center;
        T radius;
    };
}