#pragma once

#include <concepts>

#include "math.hpp"

namespace raytracer
{
template <std::floating_point T>
class SceneNode
{
   public:
    SceneNode(Shape *shape, Material<T> *material)
        : shape(shape), material(material)
    {
    }

    bool Intersect(Ray<T> *r, T max_distance, IntersectionRecord<T> &record)
    {
        UpdateTransforms();
        // Transform the ray into object space, then Intersect, then untransform
        r->direction = cached_world_to_model.TransformVec(r->direction);
        r->origin = cached_world_to_model.TransformPoint(r->origin);
        auto res = shape->Intersect(r, max_distance, record);
        r->direction = cached_transform.TransformVec(r->direction);
        r->origin = cached_transform.TransformPoint(r->origin);
        if (res)
        {
            record.normal = cached_normal_matrix.TransformVec(record.normal);
            record.normal.Normalize();
            record.position = r->Evaluate(record.t);
        }
        return res;
    }

    Color<T> Shade(const IntersectionRecord<T> &record,
                   const std::vector<Light<T>> &lights) const
    {
        return material->Shade(record, lights);
    }

    void SetTranslation(T x, T y, T z)
    {
        translation = Point3<T>(x, y, z);
        transform_dirty = true;
    }

    void SetTranslation(Point3<T> t)
    {
        translation = t;
        transform_dirty = true;
    }

    void SetRotation(T x, T y, T z)
    {
        rotation = Vec3<T>(x, y, z);
        transform_dirty = true;
    }

    void SetRotation(Vec3<T> r)
    {
        rotation = r;
        transform_dirty = true;
    }

    void SetScale(T s)
    {
        scale = s;
        transform_dirty = true;
    }

    void SetScale(T x, T y, T z)
    {
        scale = Vec3<T>(x, y, z);
        transform_dirty = true;
    }

    void SetScale(Vec3<T> s)
    {
        scale = s;
        transform_dirty = true;
    }

    void UpdateTransforms()
    {
        if (transform_dirty)
        {
            auto trans_mat = Mat4<T>::Translation(translation);
            auto rot_mat = Mat4<T>::Rotation(rotation);
            auto scale_mat = Mat4<T>::Scale(scale);
            cached_transform = trans_mat * rot_mat * scale_mat;
            cached_world_to_model = cached_transform.Invert();
            cached_normal_matrix = cached_world_to_model.Transpose();
            transform_dirty = false;
        }
    }

    Shape *GetShape() { return shape; }
    Material<T> *GetMaterial() { return material; }

   private:
    Shape *shape;
    Material<T> *material;

    Point3<T> translation;
    Vec3<T> rotation;
    Vec3<T> scale;
    Mat4<T> cached_transform;
    Mat4<T> cached_world_to_model;
    Mat4<T> cached_normal_matrix;
    bool transform_dirty;
};

template <std::floating_point T>
class SceneTree
{
   public:
    SceneTree() {}

    SceneNode<T> &AddNode(Shape *shape, Material<T> *material)
    {
        return nodes.emplace_back(shape, material);
    }

    SceneNode<T> *GetNode(Shape *shape, Material<T> *material)
    {
        auto it = std::find(nodes.begin(), nodes.end(),
                            [shape, material](auto &node) {
                                return node.GetShape() == shape &&
                                       node.GetMaterial() == material;
                            });
        if (it == nodes.end())
        {
            return nullptr;
        }
        else
        {
            return it.GetObject();
        }
    }

    std::vector<SceneNode<T>>::iterator begin() { return nodes.begin(); }

    std::vector<SceneNode<T>>::iterator end() { return nodes.end(); }

    std::vector<SceneNode<T>>::const_iterator cbegin()
    {
        return nodes.cbegin();
    }

    std::vector<SceneNode<T>>::const_iterator cend() { return nodes.cend(); }

   private:
    std::vector<SceneNode<T>> nodes;
};
}  // namespace raytracer
