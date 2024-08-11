#pragma once

#include <concepts>

#include "color.hpp"
#include "math.hpp"

namespace raytracer
{
template <std::floating_point T>
class Light
{
   public:
    Light(Point3<T> pos, T diffuse_power, Color<T> diffuse, T specular_power,
          Color<T> specular)
        : position(pos),
          diffuse_power(diffuse_power),
          diffuse(diffuse),
          specular_power(specular_power),
          specular(specular)
    {
    }

    Point3<T> position;
    T diffuse_power;
    Color<T> diffuse;
    T specular_power;
    Color<T> specular;
};

template <std::floating_point T>
std::ostream &operator<<(std::ostream &str, const Light<T> &l)
{
    str << "Light position=" << l.position
        << " diffuse power=" << l.diffuse_power << " diffuse=" << l.diffuse
        << " specular power=" << l.specular_power << " specular=" << l.specular;
    return str;
}
}  // namespace raytracer
