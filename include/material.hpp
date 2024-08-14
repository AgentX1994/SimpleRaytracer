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
    Material<T>() {}
    virtual ~Material() = default;

    virtual Color<T> Shade(const IntersectionRecord<T> &record,
                           const std::vector<Light<T>> &lights) = 0;

    virtual FresnelTerms<T> GetFresnelTerms(const Vec3<T> &incoming,
                                            const Vec3<T> &normal) const
    {
        return FresnelTerms<T>();
    };

    virtual T GetRefractiveIndex() { return 1.0; }
};

template <std::floating_point T>
class BlinnPhongMaterial : public Material<T>
{
   public:
    BlinnPhongMaterial(Color<T> base, T reflectivity, T transmissibility)
        : Material<T>(),
          base_color(base),
          reflectivity(reflectivity),
          transmissibility(transmissibility)
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

    FresnelTerms<T> GetFresnelTerms(const Vec3<T> &incoming,
                                    const Vec3<T> &normal) const override
    {
        return FresnelTerms<T>{reflectivity, transmissibility};
    };

   private:
    Color<T> base_color;
    T reflectivity, transmissibility;
};

template <std::floating_point T>
class NormalMaterial : public Material<T>
{
   public:
    NormalMaterial<T>() : Material<T>() {}

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
    PositionMaterial<T>() : Material<T>() {}

    Color<T> Shade(const IntersectionRecord<T> &record,
                   const std::vector<Light<T>> & /*lights*/) override
    {
        Point3<T> pos = 5.0 * record.position.AbsoluteValue();
        Color<T> c = {pos.x(), pos.y(), pos.z()};
        return c;
    }
};

template <std::floating_point T>
class GlassMaterial : public Material<T>
{
   public:
    GlassMaterial<T>(T refractive_index)
        : Material<T>(), refractive_index(refractive_index)
    {
    }

    Color<T> Shade(const IntersectionRecord<T> &record,
                   const std::vector<Light<T>> & /*lights*/) override
    {
        return Color<T>();
    }

    FresnelTerms<T> GetFresnelTerms(const Vec3<T> &incoming,
                                    const Vec3<T> &normal) const override
    {
        auto terms = fresnel(incoming, normal, refractive_index);
        assert(ApproximateEqual(1.0, terms.refractive + terms.reflective));
        return terms;
    };

    virtual T GetRefractiveIndex() override { return refractive_index; }

   private:
    T refractive_index;
};
}  // namespace raytracer
