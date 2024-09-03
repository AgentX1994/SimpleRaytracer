#pragma once

#include <concepts>

#include "math.hpp"

namespace raytracer
{
class Camera
{
   public:
    Camera() : position(), up(0.0f, 1.0f, 0.0f), forward(0.0f, 0.0f, -1.0f) {}
    Camera(Point3f position, Vec3f up, Vec3f forward)
        : position(position), up(up), forward(forward)
    {
        UpdateTransform();
    }

    inline void SetPosition(Point3f new_position)
    {
        position = new_position;
        UpdateTransform();
    }

    inline void SetForward(Vec3f new_forward)
    {
        forward = new_forward;
        forward.Normalize();
        UpdateTransform();
    }

    inline void SetUp(Vec3f new_up)
    {
        up = new_up;
        up.Normalize();
        UpdateTransform();
    }

    inline void Move(Vec3f movement)
    {
        auto right = Cross(up, forward);
        position = position + movement.x() * right + movement.y() * up +
                   movement.z() * forward;
        UpdateTransform();
    }

    inline void Rotate(Vec3f rotation)
    {
        forward = Mat4f::Rotation(rotation) * forward;
        forward.Normalize();
        UpdateTransform();
    }

    inline Point3f GetPosition() { return position; }

    inline Mat4f GetTransform() { return transform; }

   private:
    void UpdateTransform();

    Point3f position;
    Vec3f up;
    Vec3f forward;
    Mat4f transform;
};
}  // namespace raytracer
