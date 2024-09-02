#include "camera.hpp"

namespace raytracer
{
void Camera::UpdateTransform()
{
    // Forward is our opposite our camera_direction
    auto new_forward = forward.Reverse();
    // Compute the right vector using a temporary up vector (TODO roll
    // support)
    auto right = Cross(up, new_forward);
    // Compute up from right and forward
    auto new_up = Cross(new_forward, right);
    transform =
        Mat4f(right.x(), new_up.x(), new_forward.x(), position.x(), right.y(),
              new_up.y(), new_forward.y(), position.y(), right.z(), new_up.z(),
              new_forward.z(), position.z(), 0.0f, 0.0f, 0.0f, 1.0f);
}
}  // namespace raytracer
