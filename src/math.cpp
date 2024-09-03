#include "math.hpp"

#include <array>
#include <cassert>
#include <concepts>
#include <iostream>
#include <numbers>
#include <ostream>
#include <span>
#include <stdexcept>

namespace raytracer
{
bool ApproximateEqual(float x, float y, float relative_difference_factor)
{
    const auto greater_magnitude = std::max(std::abs(x), std::abs(y));
    return std::abs(x - y) < relative_difference_factor * greater_magnitude;
}

bool SolveQuadratic(float a, float b, float c, float &x0, float &x1)
{
    float discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0.0f)
    {
        return false;
    }
    else if (discriminant == 0.0f)
    {
        x0 = -0.5f * b / a;
        x1 = x0;
    }
    else
    {
        float q = (b > 0.0f) ? -0.5f * (b + std::sqrt(discriminant))
                             : -0.5f * (b - std::sqrt(discriminant));
        x0 = q / a;
        x1 = c / q;
    }
    if (x0 > x1)
    {
        std::swap(x0, x1);
    }
    return true;
}

Vec3fBase::Vec3fBase() : vec({0.0f}) {}

Vec3fBase::Vec3fBase(float x) : Vec3fBase(x, x, x) {}

Vec3fBase::Vec3fBase(float x, float y, float z) : Vec3fBase()
{
    vec[0] = x;
    vec[1] = y;
    vec[2] = z;
}

Vec3f::Vec3f() : Vec3fBase() {}
Vec3f::Vec3f(float x) : Vec3fBase(x) {}
Vec3f::Vec3f(float x, float y, float z) : Vec3fBase(x, y, z) {}
Vec3f::Vec3f(const Point3f &p) : Vec3f(p.x(), p.y(), p.z()) {}

Point3f::Point3f() : Vec3fBase() {}
Point3f::Point3f(float x) : Vec3fBase(x) {}
Point3f::Point3f(float x, float y, float z) : Vec3fBase(x, y, z) {}
Point3f::Point3f(const Vec3f &p) : Point3f(p.x(), p.y(), p.z()) {}

Vec3f AxisAngleToEuler(const Vec3f axis, float angle)
{
    // Adapted from
    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToEuler/index.htm
    auto x = axis.x();
    auto y = axis.y();
    auto z = axis.z();
    auto s = std::sin(angle);
    auto c = std::cos(angle);
    auto t = 1.0f - c;

    float y_len = x * y * t + z * s;
    float heading = 0.0f;
    float attitude = 0.0f;
    float bank = 0.0f;
    if (ApproximateEqual(y_len, 1.0f))
    {  // north pole singularity detected
        heading =
            2.0f * atan2(x * std::sin(angle / 2.0f), std::cos(angle / 2.0f));
        attitude = std::numbers::pi_v<float> / 2.0f;
        bank = 0.0f;
    }
    else if (ApproximateEqual(y_len, -1.0f))
    {  // south pole singularity detected
        heading =
            -2.0f * atan2(x * std::sin(angle / 2.0f), std::cos(angle / 2.0f));
        attitude = -std::numbers::pi_v<float> / 2.0f;
        bank = 0;
    }
    else
    {
        heading = std::atan2(y * s - x * z * t, 1.0f - (y * y + z * z) * t);
        attitude = std::asin(x * y * t + z * s);
        bank = std::atan2(x * s - y * z * t, 1.0f - (x * x + z * z) * t);
    }
    return Vec3f(bank, heading, attitude);
}

FresnelTerms fresnel(const Vec3f &incoming, const Vec3f &normal,
                     float refractive_index)
{
    float cos_i = Clamp(Dot(incoming, normal), -1.0f, 1.0f);

    float eta_i = 1.0f, eta_t = refractive_index;
    if (cos_i > 0.0f)
    {
        std::swap(eta_i, eta_t);
    }
    float sin_t =
        eta_i / eta_t * std::sqrt(std::max(0.0f, 1.0f - cos_i * cos_i));
    if (sin_t >= 1.0f)
    {
        // Total internal reflection
        return FresnelTerms{1.0f, 0.0f};
    }
    else
    {
        float cos_t = std::sqrt(std::max(0.0f, 1.0f - sin_t * sin_t));
        cos_i = std::abs(cos_i);
        float r_perp = ((eta_t * cos_i) - (eta_i * cos_t)) /
                       ((eta_t * cos_i) + (eta_i * cos_t));
        float r_para = ((eta_i * cos_i) - (eta_t * cos_t)) /
                       ((eta_i * cos_i) + (eta_t * cos_t));
        float reflective = (r_perp * r_perp + r_para * r_para) / 2.0f;
        return FresnelTerms{reflective, 1.0f - reflective};
    }
}

Vec3f Refract(const Vec3f &incoming, const Vec3f &normal,
              float refractive_index)
{
    float cos_i = Clamp(Dot(incoming, normal), -1.0f, 1.0f);

    float eta_i = 1.0f, eta_t = refractive_index;
    Vec3f n = normal;
    if (cos_i < 0.0f)
    {
        cos_i = -cos_i;
    }
    else
    {
        std::swap(eta_i, eta_t);
        n = n.Reverse();
    }
    float eta = eta_i / eta_t;
    float k = 1.0f - eta * eta * (1.0f - cos_i * cos_i);
    if (k < 0.0f)
    {
        return Vec3f();
    }
    else
    {
        return eta * incoming + (eta * cos_i - std::sqrt(k)) * n;
    }
}

std::ostream &operator<<(std::ostream &str, const Vec3f &v)
{
    str << '(' << v.x() << ',' << v.y() << ',' << v.z() << ')';
    return str;
}

std::ostream &operator<<(std::ostream &str, const Point3f &p)
{
    str << '(' << p.x() << ',' << p.y() << ',' << p.z() << ')';
    return str;
}

Mat4f::Mat4f() : elements({0.0f}) {}
Mat4f::Mat4f(const Mat4f &other) : elements(other.elements) {}

Mat4f Mat4f::Scale(Vec3f s)
{
    auto m = Mat4f();
    m.elements[0] = s.x();
    m.elements[5] = s.y();
    m.elements[10] = s.z();
    m.elements[15] = 1.0f;
    return m;
}

Mat4f Mat4f::Rotation(Vec3f r)
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

    auto m11 = cos_b * cos_g;
    auto m12 = sin_a * sin_b * cos_g - cos_a * sin_g;
    auto m13 = cos_a * sin_b * cos_g + sin_a * sin_g;
    auto m14 = 0.0f;
    auto m21 = cos_b * sin_g;
    auto m22 = sin_a * sin_b * sin_g + cos_a * cos_g;
    auto m23 = cos_a * sin_b * sin_g - sin_a * cos_g;
    auto m24 = 0.0f;
    auto m31 = -sin_b;
    auto m32 = sin_a * cos_b;
    auto m33 = cos_a * cos_b;
    auto m34 = 0.0f;
    auto m41 = 0.0f;
    auto m42 = 0.0f;
    auto m43 = 0.0f;
    auto m44 = 1.0f;
    return Mat4f(m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34,
                 m41, m42, m43, m44);
}

Mat4f Mat4f::Translation(Point3f p)
{
    auto m = Mat4f::Identity();
    m.elements[3] = p.x();
    m.elements[7] = p.y();
    m.elements[11] = p.z();
    m.elements[15] = 1.0f;
    return m;
}

std::span<float, 4> Mat4f::operator[](size_t i)
{
    assert(i < 4);
    return std::span<float, 4>(elements.begin() + (4 * i), 4);
}

const std::span<const float, 4> Mat4f::operator[](size_t i) const
{
    assert(i < 4);
    return std::span<const float, 4>(elements.data() + (4 * i), 4);
}

Mat4f Mat4f::Invert() const
{
    Mat4f out;
    if (!InternalInvertMatrix(elements.data(), out.elements.data()))
    {
        throw std::runtime_error("Non-Invertable Matrix!");
    }
    return out;
}

Vec3f Mat4f::operator*(const Vec3f v) const
{
    auto x = elements[0] * v.x() + elements[1] * v.y() + elements[2] * v.z();
    auto y = elements[4] * v.x() + elements[5] * v.y() + elements[6] * v.z();
    auto z = elements[8] * v.x() + elements[9] * v.y() + elements[10] * v.z();
    return Vec3f(x, y, z);
}

Point3f Mat4f::operator*(const Point3f p) const
{
    auto x = elements[0] * p.x() + elements[1] * p.y() + elements[2] * p.z() +
             elements[3];
    auto y = elements[4] * p.x() + elements[5] * p.y() + elements[6] * p.z() +
             elements[7];
    auto z = elements[8] * p.x() + elements[9] * p.y() + elements[10] * p.z() +
             elements[11];
    return Point3f(x, y, z);
}

bool Mat4f::InternalInvertMatrix(const float m[16], float invOut[16])
{
    /* Taken from https://stackoverflow.com/a/1148405 */
    float inv[16], det;
    int i;

    inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] +
             m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];

    inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] +
             m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] +
             m[12] * m[7] * m[10];

    inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] +
             m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];

    inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] +
              m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] +
              m[12] * m[6] * m[9];

    inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] +
             m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] +
             m[13] * m[3] * m[10];

    inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] +
             m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];

    inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] -
             m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];

    inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] +
              m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];

    inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] +
             m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];

    inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] -
             m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];

    inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] +
              m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];

    inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] -
              m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] -
             m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] +
             m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] -
              m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] +
              m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0.0f) return false;

    det = 1.0f / det;

    for (i = 0; i < 16; i++) invOut[i] = inv[i] * det;

    return true;
}

Mat4f operator+(const Mat4f &left, const Mat4f &right)
{
    return Mat4f({
        left[0][0] + right[0][0],
        left[0][1] + right[0][1],
        left[0][2] + right[0][2],
        left[0][3] + right[0][3],
        left[1][0] + right[1][0],
        left[1][1] + right[1][1],
        left[1][2] + right[1][2],
        left[1][3] + right[1][3],
        left[2][0] + right[2][0],
        left[2][1] + right[2][1],
        left[2][2] + right[2][2],
        left[2][3] + right[2][3],
        left[3][0] + right[3][0],
        left[3][1] + right[3][1],
        left[3][2] + right[3][2],
        left[3][3] + right[3][3],
    });
}

Mat4f operator*(const Mat4f &left, const Mat4f &right)
{
    // Matrix multiplication
    // Element x, y of the output is the dot product of the xth row of the
    // left and yth column of the right matrix
    // TODO: optimize this
    Mat4f out;
    out[0][0] = left[0][0] * right[0][0] + left[0][1] * right[1][0] +
                left[0][2] * right[2][0] + left[0][3] * right[3][0];
    out[0][1] = left[0][0] * right[0][1] + left[0][1] * right[1][1] +
                left[0][2] * right[2][1] + left[0][3] * right[3][1];
    out[0][2] = left[0][0] * right[0][2] + left[0][1] * right[1][2] +
                left[0][2] * right[2][2] + left[0][3] * right[3][2];
    out[0][3] = left[0][0] * right[0][3] + left[0][1] * right[1][3] +
                left[0][2] * right[2][3] + left[0][3] * right[3][3];
    out[1][0] = left[1][0] * right[0][0] + left[1][1] * right[1][0] +
                left[1][2] * right[2][0] + left[1][3] * right[3][0];
    out[1][1] = left[1][0] * right[0][1] + left[1][1] * right[1][1] +
                left[1][2] * right[2][1] + left[1][3] * right[3][1];
    out[1][2] = left[1][0] * right[0][2] + left[1][1] * right[1][2] +
                left[1][2] * right[2][2] + left[1][3] * right[3][2];
    out[1][3] = left[1][0] * right[0][3] + left[1][1] * right[1][3] +
                left[1][2] * right[2][3] + left[1][3] * right[3][3];
    out[2][0] = left[2][0] * right[0][0] + left[2][1] * right[1][0] +
                left[2][2] * right[2][0] + left[2][3] * right[3][0];
    out[2][1] = left[2][0] * right[0][1] + left[2][1] * right[1][1] +
                left[2][2] * right[2][1] + left[2][3] * right[3][1];
    out[2][2] = left[2][0] * right[0][2] + left[2][1] * right[1][2] +
                left[2][2] * right[2][2] + left[2][3] * right[3][2];
    out[2][3] = left[2][0] * right[0][3] + left[2][1] * right[1][3] +
                left[2][2] * right[2][3] + left[2][3] * right[3][3];
    out[3][0] = left[3][0] * right[0][0] + left[3][1] * right[1][0] +
                left[3][2] * right[2][0] + left[3][3] * right[3][0];
    out[3][1] = left[3][0] * right[0][1] + left[3][1] * right[1][1] +
                left[3][2] * right[2][1] + left[3][3] * right[3][1];
    out[3][2] = left[3][0] * right[0][2] + left[3][1] * right[1][2] +
                left[3][2] * right[2][2] + left[3][3] * right[3][2];
    out[3][3] = left[3][0] * right[0][3] + left[3][1] * right[1][3] +
                left[3][2] * right[2][3] + left[3][3] * right[3][3];
    return out;
}

std::ostream &operator<<(std::ostream &str, const Ray &ray)
{
    str << ray.origin << "+t*" << ray.direction;
    return str;
}

class Shape;

std::ostream &operator<<(std::ostream &str, const IntersectionRecord &record)
{
    str << "IntersectionRecord t=" << record.t << " shape=" << record.shape
        << " ray="
        << ((record.ray != nullptr) ? *record.ray : Ray(Point3f(), Vec3f()))
        << " position=" << record.position << " normal=" << record.normal;
    return str;
}

bool Sphere::Intersect(Ray *ray, float min_distance, float max_distance,
                       IntersectionRecord &record)
{
    // Center is implicitly at 0, 0, 0, and radius is 1
    auto diff = ray->origin;

    float a = Dot(ray->direction, ray->direction);
    float b = 2.0f * Dot(ray->direction, diff);
    float c = diff.LengthSquared() - 1;

    float t0, t1;
    if (!SolveQuadratic(a, b, c, t0, t1))
    {
        return false;
    }

    if (t0 < 0.0f)
    {
        t0 = t1;
        if (t0 < 0.0f)
        {
            return false;
        }
    }

    if (t0 < min_distance || t0 >= max_distance)
    {
        return false;
    }

    // Calculate hit location and normal at that location
    Point3f position = ray->origin + t0 * ray->direction;
    Vec3f normal = Vec3f(position).ToUnit();

#ifndef NDEBUG
    {
        float position_distance = position.LengthSquared();
        assert(ApproximateEqual(position_distance, 1.0f));
    }
#endif

    record.t = t0;
    record.ray = ray;
    record.position = position;
    record.normal = normal;
    record.shape = this;
    // Adapted from https://gamedev.stackexchange.com/a/201460
    // This is the Equirectangular projection
    record.u = std::atan2(position.z(), -position.x()) /
               (2.0f * std::numbers::pi_v<float>)+0.5f;
    record.v = position.y() * 0.5f + 0.5f;
    return true;
}

bool Plane::Intersect(Ray *ray, float min_distance, float max_distance,
                      IntersectionRecord &record)
{
    // This plane is the x-y plane, with equation 0x+0y+z=0,
    // and width and height 1
    // based on
    // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-plane-and-ray-disk-intersection.html
    float denom = ray->direction.z();
    if (std::abs(denom) > 1e-6)
    {
        // Since we assume this plane is the x-y plane, the distance
        // from the origin to the plane is just the z coordinate of the
        // ray
        float t = -ray->origin.z() / denom;
        if (t < min_distance || t >= max_distance)
        {
            return false;
        }
        auto pos = ray->Evaluate(t);
        // this should determine if the pos is outside of the desired
        // region of the plane
        auto pos_x = std::abs(pos.x());
        auto pos_y = std::abs(pos.y());
        if (pos_x > 0.5f || pos_y > 0.5f)
        {
            return false;
        }
        record.t = t;
        record.ray = ray;
        record.position = pos;
        record.normal = Vec3f(0.0f, 0.0f, 1.0f);
        record.shape = this;
        // Compute UV coords
        // Since this is on the x-y plane, we can just take the x coord of the
        // position and y coordinate of the position as the u and v,
        // respectively, and rescale to 0 to 1.
        // each coord goes from -1 to 1, so
        // multiply by 0.5 and add 0.5
        record.u = pos.x() * 0.5f + 0.5f;
        record.v = pos.y() * 0.5f + 0.5f;
        return true;
    }

    return false;
}

bool Disc::Intersect(Ray *ray, float min_distance, float max_distance,
                     IntersectionRecord &record)
{
    // The logic is basically the same as with planes, but with
    // a different distance check
    // This plane is the x-y plane, with equation 0x+0y+z=0,
    // and radius 1
    // based on
    // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-plane-and-ray-disk-intersection.html
    float denom = ray->direction.z();
    if (std::abs(denom) > 1e-6)
    {
        // Since we assume this plane is the x-y plane, the distance
        // from the origin to the plane is just the z coordinate of the
        // ray
        float t = -ray->origin.z() / denom;
        if (t < min_distance || t >= max_distance)
        {
            return false;
        }
        auto pos = ray->Evaluate(t);
        //  this should determine if the pos is outside of the desired
        //  region of the plane
        if (pos.LengthSquared() > 1.0f)
        {
            return false;
        }
        record.t = t;
        record.ray = ray;
        record.position = pos;
        record.normal = Vec3f(0.0f, 0.0f, 1.0f);
        record.shape = this;
        // Compute UV coords
        // Since this is on the x-y plane, we can just take the x coord of the
        // position and y coordinate of the position as the u and v,
        // respectively, and rescale to 0 to 1.
        // each coord goes from -1 to 1, so
        // multiply by 0.5 and add 0.5
        record.u = pos.x() * 0.5f + 0.5f;
        record.v = pos.y() * 0.5f + 0.5f;
        return true;
    }

    return false;
}

bool Triangle::Intersect(Ray *ray, float min_distance, float max_distance,
                         IntersectionRecord &record)
{
    // Triangle intersection algorithm from:
    // https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection.html
    auto v0v1 = b - a;
    auto v0v2 = c - a;
    auto pvec = Cross(ray->direction, v0v2);
    auto det = Dot(v0v1, pvec);

    if (ApproximateEqual(det, 0.0f))
    {
        return false;
    }

    float invdet = 1.0f / det;

    auto tvec = ray->origin - a;
    auto u = Dot(tvec, pvec) * invdet;
    if (u < 0.0f || u > 1.0f)
    {
        return false;
    }

    auto qvec = Cross(tvec, v0v1);
    auto v = Dot(ray->direction, qvec) * invdet;
    if (v < 0.0f || u + v > 1.0f)
    {
        return false;
    }

    // If we get here, we know we hit the triangle

    auto t = Dot(v0v2, qvec) * invdet;

    if (t < min_distance || t > max_distance)
    {
        return false;
    }

    record.t = t;
    record.ray = ray;
    record.position = ray->Evaluate(t);
    // This may be equivalent to one of the other vectors here but I can't tell
    record.normal = Cross(v0v1, v0v2).ToUnit();
    record.shape = this;
    record.u = u;
    record.v = v;

    return true;
}
}  // namespace raytracer
