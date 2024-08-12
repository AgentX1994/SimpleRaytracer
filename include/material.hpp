#pragma once

#include <concepts>

#include "color.hpp"
#include "light.hpp"
#include "math.hpp"

namespace raytracer
{
template <std::floating_point T>
class Material
{
   public:
    Material<T>(T reflectivity, T transmissibility)
        : reflectivity(reflectivity), transmissibility(transmissibility)
    {
    }
    virtual ~Material() = default;

    virtual Color<T> Shade(const IntersectionRecord<T> &record,
                           const std::vector<Light<T>> &lights) = 0;

    bool IsReflective() const { return reflectivity > 0.0; }
    T GetReflectivity() const { return reflectivity; }

    bool IsTransmissive() const { return transmissibility > 0.0; }
    T GetTransmissibility() const { return transmissibility; }

    T reflectivity = 0.0;
    T transmissibility = 0.0;
};

template <std::floating_point T>
class BlinnPhongMaterial : public Material<T>
{
   public:
    BlinnPhongMaterial(Color<T> base, T reflectivity, T transmissibility)
        : Material<T>(reflectivity, transmissibility), base_color(base)
    {
    }

    Color<T> Shade(const IntersectionRecord<T> &record,
                   const std::vector<Light<T>> &lights) override
    {
        // Init to ambient light
        Color<T> c = {0.1, 0.1, 0.1};
        Vec3<T> view_dir = record.ray->direction.Reverse();
        for (auto light : lights)
        {
            // std::cout << "Calculating Shading for hit record " << record << "
            // and light " << l << '\n';
            Vec3<T> light_dir = light.position - record.position;
            T distance_squared = light_dir.LengthSquared();
            light_dir.Normalize();

            T n_dot_l = Dot(record.normal, light_dir);
            T diffuse_intensity = Saturate(n_dot_l);

            Vec3<T> half_vec = light_dir + view_dir;
            half_vec.Normalize();
            T n_dot_h = Dot(record.normal, half_vec);
            T specular_intensity = std::pow(Saturate(n_dot_h), 32.0);

            c += specular_intensity * light.specular_power * light.specular /
                     distance_squared +
                 diffuse_intensity * light.diffuse_power * light.diffuse *
                     base_color / distance_squared;
        }

        return c;
    }

   private:
    Color<T> base_color;
};

template <std::floating_point T>
class NormalMaterial : public Material<T>
{
   public:
    NormalMaterial<T>() : Material<T>(0.0, 0.0) {}

    Color<T> Shade(const IntersectionRecord<T> &record,
                   const std::vector<Light<T>> & /*lights*/) override
    {
        Vec3<T> normal = 0.5 * record.normal + Vec3<T>(0.5, 0.5, 0.5);
        Color<T> c = {normal.x(), normal.y(), normal.z()};
        return c;
    }
};

template <std::floating_point T>
class PositionMaterial : public Material<T>
{
   public:
    PositionMaterial<T>() : Material<T>(0.0, 0.0) {}

    Color<T> Shade(const IntersectionRecord<T> &record,
                   const std::vector<Light<T>> & /*lights*/) override
    {
        Point3<T> pos = 5.0 * record.position.AbsoluteValue();
        Color<T> c = {pos.x(), pos.y(), pos.z()};
        return c;
    }
};
}  // namespace raytracer
