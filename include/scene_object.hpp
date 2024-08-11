#pragma once

#include <concepts>

#include "color.hpp"
#include "light.hpp"
#include "material.hpp"
#include "math.hpp"

namespace raytracer
{
template <std::floating_point T>
class SceneObject
{
   public:
    SceneObject(Shape *shape, Material<T> *material)
        : shape(shape), material(material) {};

    bool Intersect(Ray<T> *r, T max_distance, IntersectionRecord<T> &record)
    {
        return shape->Intersect(r, max_distance, record);
    }

    Color<T> Shade(const IntersectionRecord<T> &record,
                   const std::vector<Light<T>> &lights)
    {
        return material->Shade(record, lights);
    }

   private:
    Shape *shape;
    Material<T> *material;
};
}  // namespace raytracer
