#pragma once

#include <concepts>

#include "color.hpp"
#include "light.hpp"
#include "math.hpp"

namespace raytracer
{
class Material
{
   public:
    Material() {}
    virtual ~Material() = default;

    virtual Color Shade(const IntersectionRecord &record,
                        const std::vector<Light> &lights) = 0;

    virtual FresnelTerms GetFresnelTerms(const Vec3f &incoming,
                                         const Vec3f &normal) const
    {
        return FresnelTerms();
    };

    virtual inline float GetRefractiveIndex() { return 1.0f; }
};

class BlinnPhongMaterial : public Material
{
   public:
    BlinnPhongMaterial(Color base, float reflectivity, float transmissibility);

    Color Shade(const IntersectionRecord &record,
                const std::vector<Light> &lights) override;

    FresnelTerms GetFresnelTerms(const Vec3f &incoming,
                                 const Vec3f &normal) const override;

   private:
    Color base_color;
    float reflectivity, transmissibility;
};

class NormalMaterial : public Material
{
   public:
    NormalMaterial() : Material() {}

    Color Shade(const IntersectionRecord &record,
                const std::vector<Light> & /*lights*/) override;
};

class PositionMaterial : public Material
{
   public:
    PositionMaterial() : Material() {}

    Color Shade(const IntersectionRecord &record,
                const std::vector<Light> & /*lights*/) override;
};

class GlassMaterial : public Material
{
   public:
    GlassMaterial(float refractive_index);

    Color Shade(const IntersectionRecord &record,
                const std::vector<Light> & /*lights*/) override;

    FresnelTerms GetFresnelTerms(const Vec3f &incoming,
                                 const Vec3f &normal) const override;

    float GetRefractiveIndex() override { return refractive_index; }

   private:
    float refractive_index;
};
}  // namespace raytracer
