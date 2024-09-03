#pragma once

#include <array>
#include <cassert>
#include <concepts>
#include <iostream>
#include <ostream>
#include <span>
#include <stdexcept>

namespace raytracer
{
bool ApproximateEqual(float x, float y,
                      float relative_difference_factor = 0.0001f);

bool SolveQuadratic(float a, float b, float c, float &x0, float &x1);

inline float Clamp(float n, float min, float max)
{
    return std::max(min, std::min(n, max));
}

inline float Saturate(float n) { return Clamp(n, 0.0f, 1.0f); }

class Vec3fBase;
class Vec3f;
class Point3f;
inline Vec3f Cross(const Vec3f left, const Vec3f right);
inline float Dot(const Vec3fBase left, const Vec3fBase right);
inline float Dot(const Vec3f left, const Vec3f right);
inline Vec3fBase operator*(float left, const Vec3fBase right);
inline Vec3f operator*(float left, const Vec3f right);
inline Point3f operator*(float left, const Point3f right);

class Vec3fBase
{
   public:
    Vec3fBase();
    explicit Vec3fBase(float x);
    Vec3fBase(float x, float y, float z);

    inline float x() const { return vec[0]; }
    inline float y() const { return vec[1]; }
    inline float z() const { return vec[2]; }

    inline float LengthSquared() const { return Dot(*this, *this); }

    inline float Length() const { return std::sqrt(LengthSquared()); }

    inline Vec3fBase AbsoluteValue() const
    {
        return Vec3fBase(std::abs(x()), std::abs(y()), std::abs(z()));
    }

    inline Vec3fBase operator+(const Vec3fBase right) const
    {
        float x = this->x() + right.x();
        float y = this->y() + right.y();
        float z = this->z() + right.z();
        return Vec3fBase(x, y, z);
    }

    inline Vec3fBase operator-(const Vec3fBase right) const
    {
        float x = this->x() - right.x();
        float y = this->y() - right.y();
        float z = this->z() - right.z();
        return Vec3fBase(x, y, z);
    }

    inline Vec3fBase operator*(float right) const
    {
        float x = this->x() * right;
        float y = this->y() * right;
        float z = this->z() * right;
        return Vec3fBase(x, y, z);
    }

    inline Vec3fBase operator/(float right) const
    {
        float x = this->x() / right;
        float y = this->y() / right;
        float z = this->z() / right;
        return Vec3fBase(x, y, z);
    }

   protected:
    std::array<float, 3> vec;
};

class Vec3f : public Vec3fBase
{
   public:
    Vec3f();
    explicit Vec3f(float x);
    Vec3f(float x, float y, float z);
    explicit Vec3f(const Point3f &p);

    inline bool IsUnit() const { return Length() == 1.0f; }

    inline void Normalize()
    {
        auto len = Length();
        vec[0] /= len;
        vec[1] /= len;
        vec[2] /= len;
    }

    inline Vec3f ToUnit() const
    {
        auto len = Length();
        return *this / len;
    }

    inline Vec3f Reverse() const { return -1.0f * *this; }

    inline Vec3f AbsoluteValue() const
    {
        auto ret = Vec3fBase::AbsoluteValue();
        return Vec3f(ret.x(), ret.y(), ret.z());
    }

    inline Vec3f operator+(const Vec3f right) const
    {
        float x = this->x() + right.x();
        float y = this->y() + right.y();
        float z = this->z() + right.z();
        return Vec3f(x, y, z);
    }

    inline Vec3f operator-(const Vec3f right) const
    {
        float x = this->x() - right.x();
        float y = this->y() - right.y();
        float z = this->z() - right.z();
        return Vec3f(x, y, z);
    }

    inline Vec3f operator*(float right) const
    {
        float x = this->x() * right;
        float y = this->y() * right;
        float z = this->z() * right;
        return Vec3f(x, y, z);
    }

    inline Vec3f operator/(float right) const
    {
        float x = this->x() / right;
        float y = this->y() / right;
        float z = this->z() / right;
        return Vec3f(x, y, z);
    }

    inline Vec3f &operator+=(const Vec3f right)
    {
        *this = *this + right;
        return *this;
    }
};

class Point3f : public Vec3fBase
{
   public:
    Point3f();
    explicit Point3f(float x);
    Point3f(float x, float y, float z);
    explicit Point3f(const Vec3f &v);

    inline Point3f AbsoluteValue() const
    {
        auto ret = Vec3fBase::AbsoluteValue();
        return Point3f(ret.x(), ret.y(), ret.z());
    }

    inline Point3f operator+(const Point3f right) const
    {
        float x = this->x() + right.x();
        float y = this->y() + right.y();
        float z = this->z() + right.z();
        return Point3f(x, y, z);
    }

    inline Vec3f operator-(const Point3f right) const
    {
        float x = this->x() - right.x();
        float y = this->y() - right.y();
        float z = this->z() - right.z();
        return Vec3f(x, y, z);
    }

    inline Point3f operator*(float right) const
    {
        float x = this->x() * right;
        float y = this->y() * right;
        float z = this->z() * right;
        return Point3f(x, y, z);
    }

    inline Point3f operator/(float right) const
    {
        float x = this->x() / right;
        float y = this->y() / right;
        float z = this->z() / right;
        return Point3f(x, y, z);
    }

    inline Point3f operator+(const Vec3f right) const
    {
        float x = this->x() + right.x();
        float y = this->y() + right.y();
        float z = this->z() + right.z();
        return Point3f(x, y, z);
    }

    inline Point3f operator-(const Vec3f right) const
    {
        float x = this->x() - right.x();
        float y = this->y() - right.y();
        float z = this->z() - right.z();
        return Point3f(x, y, z);
    }

    inline Point3f &operator+=(const Vec3f right)
    {
        *this = *this + right;
        return *this;
    }
};

std::ostream &operator<<(std::ostream &str, const Vec3f &v);
std::ostream &operator<<(std::ostream &str, const Point3f &v);

inline Vec3fBase operator*(float left, const Vec3fBase right)
{
    float x = left * right.x();
    float y = left * right.y();
    float z = left * right.z();
    return Vec3fBase(x, y, z);
}

inline Vec3f operator*(float left, const Vec3f right)
{
    float x = left * right.x();
    float y = left * right.y();
    float z = left * right.z();
    return Vec3f(x, y, z);
}

inline Point3f operator*(float left, const Point3f right)
{
    float x = left * right.x();
    float y = left * right.y();
    float z = left * right.z();
    return Point3f(x, y, z);
}

inline Vec3f operator/(float left, const Vec3f right)
{
    float x = left / right.x();
    float y = left / right.y();
    float z = left / right.z();
    return Vec3f(x, y, z);
}

inline Vec3f Cross(const Vec3f left, const Vec3f right)
{
    float x = left.y() * right.z() - left.z() * right.y();
    float y = left.z() * right.x() - left.x() * right.z();
    float z = left.x() * right.y() - left.y() * right.x();
    return Vec3f(x, y, z);
}

inline float Dot(const Vec3fBase left, const Vec3fBase right)
{
    float xs = left.x() * right.x();
    float ys = left.y() * right.y();
    float zs = left.z() * right.z();
    return xs + ys + zs;
}

inline float Dot(const Vec3f left, const Vec3f right)
{
    float xs = left.x() * right.x();
    float ys = left.y() * right.y();
    float zs = left.z() * right.z();
    return xs + ys + zs;
}

Vec3f AxisAngleToEuler(const Vec3f axis, float angle);

struct FresnelTerms
{
    float reflective;
    float refractive;
};

FresnelTerms fresnel(const Vec3f &incoming, const Vec3f &normal,
                     float refractive_index);

inline Vec3f Reflect(const Vec3f &direction, const Vec3f &normal)
{
    auto new_dir = direction - 2.0f * Dot(direction, normal) * normal;
    return new_dir;
}

Vec3f Refract(const Vec3f &incoming, const Vec3f &normal,
              float refractive_index);

class Mat4f
{
   public:
    Mat4f();
    Mat4f(const Mat4f &other);
    Mat4f(std::convertible_to<float> auto &&...elems)
        : elements{{std::forward<typeof(elems)>(elems)...}}
    {
    }

    static Mat4f Scale(Vec3f s);

    static Mat4f Rotation(Vec3f r);

    static Mat4f Translation(Point3f p);

    static inline Mat4f Identity()
    {
        return Mat4f::Scale(Vec3f(1.0f, 1.0f, 1.0f));
    }

    std::span<float, 4> operator[](size_t i);

    const std::span<const float, 4> operator[](size_t i) const;

    Mat4f Invert() const;

    inline Mat4f Transpose() const
    {
        return Mat4f(elements[0], elements[4], elements[8], elements[12],
                     elements[1], elements[5], elements[9], elements[13],
                     elements[2], elements[6], elements[10], elements[14],
                     elements[3], elements[7], elements[11], elements[15]);
    }

    Vec3f operator*(const Vec3f v) const;
    Point3f operator*(const Point3f v) const;

   private:
    static bool InternalInvertMatrix(const float m[16], float invOut[16]);

    std::array<float, 16> elements;
};

Mat4f operator+(const Mat4f &left, const Mat4f &right);

Mat4f operator*(const Mat4f &left, const Mat4f &right);

class Ray
{
   public:
    Ray(Point3f origin, Vec3f direction) : origin(origin), direction(direction)
    {
        inverse_direction = 1.0f / direction;
        signs[0] = inverse_direction.x() < 0.0f;
        signs[1] = inverse_direction.y() < 0.0f;
        signs[2] = inverse_direction.z() < 0.0f;
    }

    inline Point3f Evaluate(float t) { return origin + t * direction; }

    Point3f origin;
    Vec3f direction;
    Vec3f inverse_direction;
    std::array<int, 3> signs;
};

std::ostream &operator<<(std::ostream &str, const Ray &ray);

class Shape;

struct IntersectionRecord
{
    float t = std::numeric_limits<float>::infinity();
    Shape *shape = nullptr;
    Ray *ray = nullptr;
    Point3f position;
    Vec3f normal;
    float u = 0.0f;
    float v = 0.0f;
};

std::ostream &operator<<(std::ostream &str, const IntersectionRecord &record);

class Shape
{
   public:
    virtual ~Shape() = default;

    virtual bool Intersect(Ray *ray, float min_distance, float max_distance,
                           IntersectionRecord &record) = 0;
};

class Sphere : public Shape
{
   public:
    Sphere() {}

    bool Intersect(Ray *ray, float min_distance, float max_distance,
                   IntersectionRecord &record) override;
};

class Plane : public Shape
{
   public:
    Plane() {}

    bool Intersect(Ray *ray, float min_distance, float max_distance,
                   IntersectionRecord &record) override;
};

class Disc : public Shape
{
   public:
    Disc() {}

    bool Intersect(Ray *ray, float min_distance, float max_distance,
                   IntersectionRecord &record) override;
};

class AxisAlignedBox : public Shape
{
   public:
    AxisAlignedBox() = default;
    AxisAlignedBox(Point3f a, Point3f b);

    void AddPoint(Point3f p);

    bool Intersect(Ray *ray, float min_distance, float max_distance,
                   IntersectionRecord &record) override;

    std::array<Point3f, 2> bounds;
};

class Triangle : public Shape
{
   public:
    Triangle(const Point3f a, const Point3f b, const Point3f c)
        : a(a), b(b), c(c)
    {
    }

    bool Intersect(Ray *ray, float min_distance, float max_distance,
                   IntersectionRecord &record) override;

    Point3f a, b, c;
};

class Mesh : public Shape
{
   public:
    Mesh();
    Mesh(std::vector<Triangle> tris);

    bool Intersect(Ray *ray, float min_distance, float max_distance,
                   IntersectionRecord &record) override;

   private:
    std::vector<Triangle> triangles;
    AxisAlignedBox bounding_box;
};
}  // namespace raytracer
