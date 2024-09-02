#pragma once

#include <concepts>

#include "color.hpp"
#include "math.hpp"

namespace raytracer
{
class Light
{
   public:
    Light(Point3f pos, float diffuse_power, Color diffuse, float specular_power,
          Color specular)
        : position(pos),
          diffuse_power(diffuse_power),
          diffuse(diffuse),
          specular_power(specular_power),
          specular(specular)
    {
    }

    Point3f position;
    float diffuse_power;
    Color diffuse;
    float specular_power;
    Color specular;
};

inline std::ostream &operator<<(std::ostream &str, const Light &l)
{
    str << "Light position=" << l.position
        << " diffuse power=" << l.diffuse_power << " diffuse=" << l.diffuse
        << " specular power=" << l.specular_power << " specular=" << l.specular;
    return str;
}
}  // namespace raytracer
