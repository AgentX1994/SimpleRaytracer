#pragma once

#include <concepts>

#include "light.hpp"
#include "material.hpp"
#include "math.hpp"

namespace raytracer
{
class SceneNode
{
   public:
    SceneNode(Shape *shape, Material *material)
        : shape(shape), material(material)
    {
    }

    bool Intersect(Ray *r, float min_distance, float max_distance,
                   IntersectionRecord &record);

    inline Color Shade(const IntersectionRecord &record,
                       const std::vector<Light> &lights) const
    {
        return material->Shade(record, lights);
    }

    inline FresnelTerms GetFresnelTerms(const Vec3f &incoming,
                                        const Vec3f &normal) const
    {
        return material->GetFresnelTerms(incoming, normal);
    }

    inline float GetRefractiveIndex() const
    {
        return material->GetRefractiveIndex();
    }

    inline void SetTranslation(float x, float y, float z)
    {
        translation = Point3f(x, y, z);
        transform_dirty = true;
    }

    inline void SetTranslation(Point3f t)
    {
        translation = t;
        transform_dirty = true;
    }

    inline void SetRotation(float x, float y, float z)
    {
        rotation = Vec3f(x, y, z);
        transform_dirty = true;
    }

    inline void SetRotation(Vec3f r)
    {
        rotation = r;
        transform_dirty = true;
    }

    inline void SetScale(float s)
    {
        scale = s;
        transform_dirty = true;
    }

    inline void SetScale(float x, float y, float z)
    {
        scale = Vec3f(x, y, z);
        transform_dirty = true;
    }

    inline void SetScale(Vec3f s)
    {
        scale = s;
        transform_dirty = true;
    }

    void UpdateTransforms();

    inline Shape *GetShape() { return shape; }
    inline Material *GetMaterial() { return material; }

   private:
    Shape *shape;
    Material *material;

    Point3f translation;
    Vec3f rotation;
    Vec3f scale;
    Mat4f cached_transform;
    Mat4f cached_world_to_model;
    Mat4f cached_normal_matrix;
    bool transform_dirty;
};

class SceneTree
{
   public:
    SceneTree() {}

    inline SceneNode &AddNode(Shape *shape, Material *material)
    {
        return nodes.emplace_back(shape, material);
    }

    SceneNode *GetNode(Shape *shape, Material *material);

    inline std::vector<SceneNode>::iterator begin() { return nodes.begin(); }

    inline std::vector<SceneNode>::iterator end() { return nodes.end(); }

    inline std::vector<SceneNode>::const_iterator cbegin()
    {
        return nodes.cbegin();
    }

    inline std::vector<SceneNode>::const_iterator cend()
    {
        return nodes.cend();
    }

   private:
    std::vector<SceneNode> nodes;
};
}  // namespace raytracer
