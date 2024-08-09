#pragma once

#include <array>
#include <concepts>
#include <cassert>
#include <ostream>

namespace raytracer
{
    template <std::floating_point T>
    bool ApproximateEqual(T x, T y, T relative_difference_factor = 0.0001)
    {
        const auto greater_magnitude = std::max(std::abs(x),std::abs(y));
        return std::abs(x-y) < relative_difference_factor * greater_magnitude;
    }

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
    inline T Clamp(T n, T min, T max)
    {
        return std::max(min, std::min(n, max));
    }

    template <std::floating_point T>
    inline T Saturate(T n)
    {
        return Clamp(n, 0.0, 1.0);
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

        inline T x() const { return vec[0]; }
        inline T y() const { return vec[1]; }
        inline T z() const { return vec[2]; }

        inline T LengthSquared() const
        {
            return Dot(*this, *this);
        }

        inline T Length() const
        {
            return std::sqrt(LengthSquared());
        }

        inline bool IsUnit() const
        {
            return Length() == 1.0;
        }

        inline void Normalize()
        {
            auto len = Length();
            vec[0] /= len;
            vec[1] /= len;
            vec[2] /= len;
        }

        inline Vec3<T> ToUnit() const
        {
            auto len = Length();
            return *this / len;
        }

        inline Vec3<T> Reverse() const
        {
            return -1.0 * *this;
        }

        inline Vec3<T> AbsoluteValue() const
        {
            return Vec3(
                std::abs(x()),
                std::abs(y()),
                std::abs(z())
            );
        }

    private:
        std::array<T, 3> vec;
    };

    template <std::floating_point T>
    std::ostream& operator<<(std::ostream &str, const Vec3<T> &v)
    {
        str << '(' << v.x() << ',' << v.y() << ',' << v.z() << ')';
        return str;
    }

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
    inline Point3<T> operator*(T left, Vec3<T> right)
    {
        T x = left * right.x();
        T y = left * right.y();
        T z = left * right.z();
        return Point3<T>(x, y, z);
    }

    template <std::floating_point T>
    inline Point3<T> operator*(Vec3<T> left, T right)
    {
        T x = left.x() * right;
        T y = left.y() * right;
        T z = left.z() * right;
        return Point3<T>(x, y, z);
    }

    template <std::floating_point T>
    inline Point3<T> operator/(Vec3<T> left, T right)
    {
        T x = left.x() / right;
        T y = left.y() / right;
        T z = left.z() / right;
        return Point3<T>(x, y, z);
    }

    template <std::floating_point T>
    inline Vec3<T> Cross(Vec3<T> left, Vec3<T> right)
    {
        T x = left.y() * right.z() - left.z() * right.y();
        T y = left.z() * right.x() - left.x() * right.z();
        T z = left.x() * right.y() - left.y() * right.x();
        return Vec3(x, y, z);
    }

    template <std::floating_point T>
    inline T Dot(Vec3<T> left, Vec3<T> right)
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

    template <std::floating_point T>
    std::ostream& operator<<(std::ostream &str, const Ray<T> &ray)
    {
        str << ray.origin << "+t*" << ray.direction;
        return str;
    }

    class Shape;

    template <std::floating_point T>
    struct IntersectionRecord
    {
        T t = std::numeric_limits<T>::infinity();
        Shape *shape = nullptr;
        Ray<T> *ray = nullptr;
        Point3<T> position;
        Vec3<T> normal;
    };

    template <std::floating_point T>
    std::ostream& operator<<(std::ostream &str, const IntersectionRecord<T> &record)
    {
        str << "IntersectionRecord t=" << record.t << " shape=" << record.shape << " ray=" << *record.ray << " position=" << record.position << " normal=" << record.normal;
        return str;
    }

    class Shape
    {
    public:
        virtual bool Intersect(Ray<float> *ray, float max_distance, IntersectionRecord<float> &record) = 0;
        virtual bool Intersect(Ray<double> *ray, double max_distance, IntersectionRecord<double> &record) = 0;
    };

    template <std::floating_point T>
    class Sphere : public Shape
    {
    public:
        Sphere(Point3<T> center, T radius) : center(center), radius(radius) {}

        bool Intersect(Ray<float> *ray, float max_distance, IntersectionRecord<float> &record) override
        {
            if constexpr (std::is_same_v<float, T>)
            {
                return IntersectInner(ray, max_distance, record);
            }
            else
            {
                return false;
            }
        }
        bool Intersect(Ray<double> *ray, double max_distance, IntersectionRecord<double> &record) override
        {
            return IntersectInner(ray, max_distance, record);
        }

    private:
        bool IntersectInner(Ray<T> *ray, T max_distance, IntersectionRecord<T> &record)
        {
            auto diff = ray->origin - center;

            T a = Dot(ray->direction, ray->direction);
            T b = 2.0 * Dot(ray->direction, diff);
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

            // Calculate hit location and normal at that location
            Point3<T> position = ray->origin + t0 * ray->direction;
            Vec3<T> normal = (position - center).ToUnit();

#ifndef NDEBUG
            {
                T position_distance = (position - center).LengthSquared();
                assert(ApproximateEqual(position_distance, radius*radius));
            }
#endif

            record.t = t0;
            record.ray = ray;
            record.position = position;
            record.normal = normal;
            record.shape = this;
            return true;
        }

        Point3<T> center;
        T radius;
    };
}
