#include "material.hpp"

#include "color.hpp"
#include "light.hpp"
#include "math.hpp"

namespace raytracer
{
BlinnPhongMaterial::BlinnPhongMaterial(Color base, float reflectivity,
                                       float transmissibility)
    : Material(),
      base_color(base),
      reflectivity(reflectivity),
      transmissibility(transmissibility)
{
}

Color BlinnPhongMaterial::Shade(const IntersectionRecord &record,
                                const std::vector<Light> &lights)
{
    // Init to ambient light
    Color c = {0.1f, 0.1f, 0.1f};
    Vec3f view_dir = record.ray->direction.Reverse();
    for (auto light : lights)
    {
        // std::cout << "Calculating Shading for hit record " << record << "
        // and light " << l << '\n';
        Vec3f light_dir = light.position - record.position;
        float distance_squared = light_dir.LengthSquared();
        light_dir.Normalize();

        float n_dot_l = Dot(record.normal, light_dir);
        float diffuse_intensity = Saturate(n_dot_l);

        Vec3f half_vec = light_dir + view_dir;
        half_vec.Normalize();
        float n_dot_h = Dot(record.normal, half_vec);
        float specular_intensity = std::pow(Saturate(n_dot_h), 32.0f);

        c += specular_intensity * light.specular_power * light.specular /
                 distance_squared +
             diffuse_intensity * light.diffuse_power * light.diffuse *
                 base_color / distance_squared;
    }

    return c;
}

FresnelTerms BlinnPhongMaterial::GetFresnelTerms(const Vec3f &incoming,
                                                 const Vec3f &normal) const
{
    return FresnelTerms{reflectivity, transmissibility};
};

Color NormalMaterial::Shade(const IntersectionRecord &record,
                            const std::vector<Light> & /*lights*/)
{
    Vec3f normal = 0.5f * record.normal + Vec3f(0.5f, 0.5f, 0.5f);
    Color c = {normal.x(), normal.y(), normal.z()};
    return c;
}

Color PositionMaterial::Shade(const IntersectionRecord &record,
                              const std::vector<Light> & /*lights*/)
{
    Point3f pos = 5.0f * record.position.AbsoluteValue();
    Color c = {pos.x(), pos.y(), pos.z()};
    return c;
}

GlassMaterial::GlassMaterial(float refractive_index)
    : Material(), refractive_index(refractive_index)
{
}

Color GlassMaterial::Shade(const IntersectionRecord &record,
                           const std::vector<Light> & /*lights*/)
{
    return Color();
}

FresnelTerms GlassMaterial::GetFresnelTerms(const Vec3f &incoming,
                                            const Vec3f &normal) const
{
    auto terms = fresnel(incoming, normal, refractive_index);
    assert(ApproximateEqual(1.0f, terms.refractive + terms.reflective));
    return terms;
}

Color UVMaterial::Shade(const IntersectionRecord &record,
                        const std::vector<Light> & /*lights*/)
{
    return Color{record.u, record.v, 0.0f};
}
}  // namespace raytracer
