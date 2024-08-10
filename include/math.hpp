#pragma once

#include <array>
#include <concepts>
#include <cassert>
#include <ostream>
#include <span>
#include <stdexcept>

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
    class Mat4
    {
    public:
        Mat4() : elements({0.0}) {}
        template <typename ...VARARGS>
        Mat4(VARARGS&&...elems) : elements{{std::forward<VARARGS>(elems)...}} {}

        static inline Mat4<T> Scale(Vec3<T> s)
        {
            auto m = Mat4<T>();
            m.elements[0] = s.x();
            m.elements[5] = s.y();
            m.elements[10] = s.z();
            m.elements[15] = 1.0;
            return m;
        }

        static inline Mat4<T> Rotation(Vec3<T> r)
        {
            auto alpha = r.x();
            auto beta = r.y();
            auto gamma = r.z();

            auto sin_a = std::sin(alpha);
            auto cos_a = std::cos(alpha);
            auto sin_b = std::sin(beta);
            auto cos_b = std::cos(beta);
            auto sin_g = std::sin(gamma);
            auto cos_g = std::cos(gamma);

            auto m11 = cos_b*cos_g;
            auto m12 = sin_a*sin_b*cos_g - cos_a*sin_g;
            auto m13 = cos_a*sin_b*cos_g + sin_a*sin_g;
            auto m14 = (T)0.0;
            auto m21 = cos_b*sin_g;
            auto m22 = sin_a*sin_b*sin_g + cos_a*cos_g;
            auto m23 = cos_a*sin_b*sin_g - sin_a*cos_g;
            auto m24 = (T)0.0;
            auto m31 = -sin_b;
            auto m32 = sin_a*cos_b;
            auto m33 = cos_a*cos_b;
            auto m34 = (T)0.0;
            auto m41 = (T)0.0;
            auto m42 = (T)0.0;
            auto m43 = (T)0.0;
            auto m44 = (T)1.0;
            return Mat4<T>(
                m11, m12, m13, m14,
                m21, m22, m23, m24,
                m31, m32, m33, m34,
                m41, m42, m43, m44
            );
        }

        static inline Mat4<T> Translation(Point3<T> p)
        {
            auto m = Mat4<T>::Identity();
            m.elements[3] = p.x();
            m.elements[7] = p.y();
            m.elements[11] = p.z();
            m.elements[15] = 1.0;
            return m;
        }

        static inline Mat4<T> Identity()
        {
            return Mat4<T>::Scale(Vec3<T>(1.0, 1.0, 1.0));
        }

        std::span<T, 4> operator[](size_t i)
        {
            assert(i < 4);
            return std::span<T, 4>(elements.begin() + (4*i), 4);
        }

        const std::span<const T, 4> operator[](size_t i) const
        {
            assert(i < 4);
            return std::span<const T, 4>(elements.data() + (4*i), 4);
        }

        Mat4<T> Invert() const
        {
            Mat4<T> out;
            if (!InternalInvertMatrix(elements.data(), out.elements.data()))
            {
                throw std::runtime_error("Non-Invertable Matrix!");
            }
            return out;
        }

        Mat4<T> Transpose() const
        {
            return Mat4<T>(
                elements[0], elements[4], elements[8], elements[12],
                elements[1], elements[5], elements[9], elements[13],
                elements[2], elements[6], elements[10], elements[14],
                elements[3], elements[7], elements[11], elements[15]
            );
        }

        // Currently, Vec3 and Point3 are just aliases for the same type, so
        // we cannot use operator overloads with them without causing ambiquity
        // TODO: fix this
        Vec3<T> TransformVec(Vec3<T> v) const
        {
            auto x = elements[0]*v.x() + elements[1]*v.y() + elements[2]*v.z();
            auto y = elements[4]*v.x() + elements[5]*v.y() + elements[6]*v.z();
            auto z = elements[8]*v.x() + elements[9]*v.y() + elements[10]*v.z();
            return Vec3<T>(x, y, z);
        }

        Point3<T> TransformPoint(Point3<T> p) const
        {
            auto x = elements[0]*p.x() + elements[1]*p.y() + elements[2]*p.z() + elements[3];
            auto y = elements[4]*p.x() + elements[5]*p.y() + elements[6]*p.z() + elements[7];
            auto z = elements[8]*p.x() + elements[9]*p.y() + elements[10]*p.z() + elements[11];
            return Point3<T>(x, y, z);
        }

    private:
        static bool InternalInvertMatrix(const T m[16], T invOut[16])
        {
            /* Taken from https://stackoverflow.com/a/1148405 */
            double inv[16], det;
            int i;

            inv[0] = m[5]  * m[10] * m[15] - 
                     m[5]  * m[11] * m[14] - 
                     m[9]  * m[6]  * m[15] + 
                     m[9]  * m[7]  * m[14] +
                     m[13] * m[6]  * m[11] - 
                     m[13] * m[7]  * m[10];

            inv[4] = -m[4]  * m[10] * m[15] + 
                      m[4]  * m[11] * m[14] + 
                      m[8]  * m[6]  * m[15] - 
                      m[8]  * m[7]  * m[14] - 
                      m[12] * m[6]  * m[11] + 
                      m[12] * m[7]  * m[10];

            inv[8] = m[4]  * m[9] * m[15] - 
                     m[4]  * m[11] * m[13] - 
                     m[8]  * m[5] * m[15] + 
                     m[8]  * m[7] * m[13] + 
                     m[12] * m[5] * m[11] - 
                     m[12] * m[7] * m[9];

            inv[12] = -m[4]  * m[9] * m[14] + 
                       m[4]  * m[10] * m[13] +
                       m[8]  * m[5] * m[14] - 
                       m[8]  * m[6] * m[13] - 
                       m[12] * m[5] * m[10] + 
                       m[12] * m[6] * m[9];

            inv[1] = -m[1]  * m[10] * m[15] + 
                      m[1]  * m[11] * m[14] + 
                      m[9]  * m[2] * m[15] - 
                      m[9]  * m[3] * m[14] - 
                      m[13] * m[2] * m[11] + 
                      m[13] * m[3] * m[10];

            inv[5] = m[0]  * m[10] * m[15] - 
                     m[0]  * m[11] * m[14] - 
                     m[8]  * m[2] * m[15] + 
                     m[8]  * m[3] * m[14] + 
                     m[12] * m[2] * m[11] - 
                     m[12] * m[3] * m[10];

            inv[9] = -m[0]  * m[9] * m[15] + 
                      m[0]  * m[11] * m[13] + 
                      m[8]  * m[1] * m[15] - 
                      m[8]  * m[3] * m[13] - 
                      m[12] * m[1] * m[11] + 
                      m[12] * m[3] * m[9];

            inv[13] = m[0]  * m[9] * m[14] - 
                      m[0]  * m[10] * m[13] - 
                      m[8]  * m[1] * m[14] + 
                      m[8]  * m[2] * m[13] + 
                      m[12] * m[1] * m[10] - 
                      m[12] * m[2] * m[9];

            inv[2] = m[1]  * m[6] * m[15] - 
                     m[1]  * m[7] * m[14] - 
                     m[5]  * m[2] * m[15] + 
                     m[5]  * m[3] * m[14] + 
                     m[13] * m[2] * m[7] - 
                     m[13] * m[3] * m[6];

            inv[6] = -m[0]  * m[6] * m[15] + 
                      m[0]  * m[7] * m[14] + 
                      m[4]  * m[2] * m[15] - 
                      m[4]  * m[3] * m[14] - 
                      m[12] * m[2] * m[7] + 
                      m[12] * m[3] * m[6];

            inv[10] = m[0]  * m[5] * m[15] - 
                      m[0]  * m[7] * m[13] - 
                      m[4]  * m[1] * m[15] + 
                      m[4]  * m[3] * m[13] + 
                      m[12] * m[1] * m[7] - 
                      m[12] * m[3] * m[5];

            inv[14] = -m[0]  * m[5] * m[14] + 
                       m[0]  * m[6] * m[13] + 
                       m[4]  * m[1] * m[14] - 
                       m[4]  * m[2] * m[13] - 
                       m[12] * m[1] * m[6] + 
                       m[12] * m[2] * m[5];

            inv[3] = -m[1] * m[6] * m[11] + 
                      m[1] * m[7] * m[10] + 
                      m[5] * m[2] * m[11] - 
                      m[5] * m[3] * m[10] - 
                      m[9] * m[2] * m[7] + 
                      m[9] * m[3] * m[6];

            inv[7] = m[0] * m[6] * m[11] - 
                     m[0] * m[7] * m[10] - 
                     m[4] * m[2] * m[11] + 
                     m[4] * m[3] * m[10] + 
                     m[8] * m[2] * m[7] - 
                     m[8] * m[3] * m[6];

            inv[11] = -m[0] * m[5] * m[11] + 
                       m[0] * m[7] * m[9] + 
                       m[4] * m[1] * m[11] - 
                       m[4] * m[3] * m[9] - 
                       m[8] * m[1] * m[7] + 
                       m[8] * m[3] * m[5];

            inv[15] = m[0] * m[5] * m[10] - 
                      m[0] * m[6] * m[9] - 
                      m[4] * m[1] * m[10] + 
                      m[4] * m[2] * m[9] + 
                      m[8] * m[1] * m[6] - 
                      m[8] * m[2] * m[5];

            det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

            if (det == 0)
                return false;

            det = 1.0 / det;

            for (i = 0; i < 16; i++)
                invOut[i] = inv[i] * det;

            return true;
        }

        std::array<T, 16> elements;
    };

    template <std::floating_point T>
    Mat4<T> operator+(const Mat4<T> &left, const Mat4<T> &right)
    {
        return Mat4<T>({
            left[0][0]+right[0][0], left[0][1]+right[0][1], left[0][2]+right[0][2], left[0][3]+right[0][3],
            left[1][0]+right[1][0], left[1][1]+right[1][1], left[1][2]+right[1][2], left[1][3]+right[1][3],
            left[2][0]+right[2][0], left[2][1]+right[2][1], left[2][2]+right[2][2], left[2][3]+right[2][3],
            left[3][0]+right[3][0], left[3][1]+right[3][1], left[3][2]+right[3][2], left[3][3]+right[3][3],
        });
    }

    template <std::floating_point T>
    Mat4<T> operator*(const Mat4<T> &left, const Mat4<T> &right)
    {
        // Matrix multiplication
        // Element x, y of the output is the dot product of the xth row of the left and yth column of the right matrix
        // TODO: optimize this
        Mat4<T> out;
        out[0][0] = left[0][0]*right[0][0]
                  + left[0][1]*right[1][0]
                  + left[0][2]*right[2][0]
                  + left[0][3]*right[3][0];
        out[0][1] = left[0][0]*right[0][1]
                  + left[0][1]*right[1][1]
                  + left[0][2]*right[2][1]
                  + left[0][3]*right[3][1];
        out[0][2] = left[0][0]*right[0][2]
                  + left[0][1]*right[1][2]
                  + left[0][2]*right[2][2]
                  + left[0][3]*right[3][2];
        out[0][3] = left[0][0]*right[0][3]
                  + left[0][1]*right[1][3]
                  + left[0][2]*right[2][3]
                  + left[0][3]*right[3][3];
        out[1][0] = left[1][0]*right[0][0]
                  + left[1][1]*right[1][0]
                  + left[1][2]*right[2][0]
                  + left[1][3]*right[3][0];
        out[1][1] = left[1][0]*right[0][1]
                  + left[1][1]*right[1][1]
                  + left[1][2]*right[2][1]
                  + left[1][3]*right[3][1];
        out[1][2] = left[1][0]*right[0][2]
                  + left[1][1]*right[1][2]
                  + left[1][2]*right[2][2]
                  + left[1][3]*right[3][2];
        out[1][3] = left[1][0]*right[0][3]
                  + left[1][1]*right[1][3]
                  + left[1][2]*right[2][3]
                  + left[1][3]*right[3][3];
        out[2][0] = left[2][0]*right[0][0]
                  + left[2][1]*right[1][0]
                  + left[2][2]*right[2][0]
                  + left[2][3]*right[3][0];
        out[2][1] = left[2][0]*right[0][1]
                  + left[2][1]*right[1][1]
                  + left[2][2]*right[2][1]
                  + left[2][3]*right[3][1];
        out[2][2] = left[2][0]*right[0][2]
                  + left[2][1]*right[1][2]
                  + left[2][2]*right[2][2]
                  + left[2][3]*right[3][2];
        out[2][3] = left[2][0]*right[0][3]
                  + left[2][1]*right[1][3]
                  + left[2][2]*right[2][3]
                  + left[2][3]*right[3][3];
        out[3][0] = left[3][0]*right[0][0]
                  + left[3][1]*right[1][0]
                  + left[3][2]*right[2][0]
                  + left[3][3]*right[3][0];
        out[3][1] = left[3][0]*right[0][1]
                  + left[3][1]*right[1][1]
                  + left[3][2]*right[2][1]
                  + left[3][3]*right[3][1];
        out[3][2] = left[3][0]*right[0][2]
                  + left[3][1]*right[1][2]
                  + left[3][2]*right[2][2]
                  + left[3][3]*right[3][2];
        out[3][3] = left[3][0]*right[0][3]
                  + left[3][1]*right[1][3]
                  + left[3][2]*right[2][3]
                  + left[3][3]*right[3][3];
        //for (size_t r = 0; r < 4; ++r)
        //{
        //    for (size_t c = 0; c < 4; ++c)
        //    {
        //        auto m1 = left[r][0]*right[0][c];
        //        auto m2 = left[r][1]*right[1][c];
        //        auto m3 = left[r][2]*right[2][c];
        //        auto m4 = left[r][3]*right[3][c];
        //        out[r][c] = m1+m2+m3+m4;
        //    }
        //}
        return out;
    }

    template <std::floating_point T>
    class Ray
    {
    public:
        Ray(Point3<T> origin, Vec3<T> direction) : origin(origin), direction(direction) {}

        Point3<T> origin;
        Vec3<T> direction;

        Point3<T> Evaluate(T t)
        {
            return origin + t*direction;
        }
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

    class Sphere : public Shape
    {
    public:
        Sphere(){}

        bool Intersect(Ray<float> *ray, float max_distance, IntersectionRecord<float> &record) override
        {
            return IntersectInner(ray, max_distance, record);
        }
        bool Intersect(Ray<double> *ray, double max_distance, IntersectionRecord<double> &record) override
        {
            return IntersectInner(ray, max_distance, record);
        }

    private:
        template <std::floating_point T>
        bool IntersectInner(Ray<T> *ray, T max_distance, IntersectionRecord<T> &record)
        {
            // Center is implicitly at 0, 0, 0, and radius is 1
            auto diff = ray->origin;

            T a = Dot(ray->direction, ray->direction);
            T b = 2.0 * Dot(ray->direction, diff);
            T c = diff.LengthSquared() - 1;

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
            Vec3<T> normal = position.ToUnit();

#ifndef NDEBUG
            {
                T position_distance = position.LengthSquared();
                assert(ApproximateEqual(position_distance, (T)1.0));
            }
#endif

            record.t = t0;
            record.ray = ray;
            record.position = position;
            record.normal = normal;
            record.shape = this;
            return true;
        }
    };
}
