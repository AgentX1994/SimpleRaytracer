#pragma once

#include <concepts>

#include "math.hpp"

namespace raytracer
{
template <std::floating_point T>
class Camera
{
   public:
    Camera() : position(), up(0.0, 1.0, 0.0), forward(0.0, 0.0, -1.0) {}
    Camera(Point3<T> position, Vec3<T> up, Vec3<T> forward)
        : position(position), up(up), forward(forward)
    {
        UpdateTransform();
    }

    void SetPosition(Point3<T> new_position)
    {
        position = new_position;
        UpdateTransform();
    }

    void SetForward(Vec3<T> new_forward)
    {
        forward = new_forward;
        forward.Normalize();
        UpdateTransform();
    }

    void SetUp(Vec3<T> new_up)
    {
        up = new_up;
        up.Normalize();
        UpdateTransform();
    }

    void Move(Vec3<T> movement)
    {
        auto right = Cross(up, forward);
        position = position + movement.x() * right + movement.y() * up +
                   movement.z() * forward;
        UpdateTransform();
    }

    void Rotate(Vec3<T> rotation)
    {
        forward = Mat4<double>::Rotation(rotation).TransformVec(forward);
        forward.Normalize();
        UpdateTransform();
    }

    Point3<T> GetPosition() { return position; }

    Mat4<T> GetTransform() { return transform; }

   private:
    void UpdateTransform()
    {
        // Forward is our opposite our camera_direction
        auto new_forward = forward.Reverse();
        // Compute the right vector using a temporary up vector (TODO roll
        // support)
        auto right = Cross(up, new_forward);
        // Compute up from right and forward
        auto new_up = Cross(new_forward, right);
        transform = Mat4<double>(
            right.x(), new_up.x(), new_forward.x(), position.x(), right.y(),
            new_up.y(), new_forward.y(), position.y(), right.z(), new_up.z(),
            new_forward.z(), position.z(), 0.0, 0.0, 0.0, 1.0);
    }

    Point3<T> position;
    Vec3<T> up;
    Vec3<T> forward;
    Mat4<T> transform;
};
}  // namespace raytracer
