#pragma once

#include <concepts>

#include "math.hpp"
#include "scene_object.hpp"

namespace raytracer {
    template <std::floating_point T>
    struct SceneNode
    {
        SceneNode<T> *parent;
        SceneObject<T> *object;

        Point3<T> translation;
        Vec3<T> rotation;
        Vec3<T> scale;
        Mat4<T> cached_transform;
        Mat4<T> cached_world_to_model;
        Mat4<T> cached_normal_matrix;
        bool transform_dirty;

        bool Intersect(Ray<T> *r, T max_distance, IntersectionRecord<T> &record)
        {
            UpdateTransforms();
            // Transform the ray into object space, then Intersect, then untransform
            r->direction = cached_world_to_model.TransformVec(r->direction);
            r->origin = cached_world_to_model.TransformPoint(r->origin);
            auto res = object->Intersect(r, max_distance, record);
            r->direction = cached_transform.TransformVec(r->direction);
            r->origin = cached_transform.TransformPoint(r->origin);
            if (res) {
                record.normal = cached_normal_matrix.TransformVec(record.normal);
                record.normal.Normalize();
                record.position = r->Evaluate(record.t);
            }
            return res;
        }

        Color<T> Shade(const IntersectionRecord<T> &record, const std::vector<Light<T>> &lights)
        {
            return object->Shade(record, lights);
        }

    private:
        void UpdateTransforms()
        {
            if (transform_dirty)
            {
                auto trans_mat = Mat4<T>::Translation(translation);
                auto rot_mat = Mat4<T>::Rotation(rotation);
                auto scale_mat = Mat4<T>::Scale(scale);
                cached_transform = trans_mat
                    * rot_mat
                    * scale_mat;
                cached_world_to_model = cached_transform.Invert();
                cached_normal_matrix = cached_world_to_model.Transpose();
                transform_dirty = false;
            }
        }
    };
    template <std::floating_point T>
    class SceneTree
    {
    public:
        SceneTree() {}
    private:
        
    };
}
