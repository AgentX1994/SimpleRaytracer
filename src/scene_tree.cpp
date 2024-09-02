#include "scene_tree.hpp"

namespace raytracer
{

bool SceneNode::Intersect(Ray *r, float min_distance, float max_distance,
                          IntersectionRecord &record)
{
    UpdateTransforms();
    // Transform the ray into object space, then Intersect, then untransform
    r->direction = cached_world_to_model.TransformVec(r->direction);
    r->origin = cached_world_to_model.TransformPoint(r->origin);
    auto res = shape->Intersect(r, min_distance, max_distance, record);
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

void SceneNode::UpdateTransforms()
{
    if (transform_dirty)
    {
        auto trans_mat = Mat4f::Translation(translation);
        auto rot_mat = Mat4f::Rotation(rotation);
        auto scale_mat = Mat4f::Scale(scale);
        cached_transform = trans_mat * rot_mat * scale_mat;
        cached_world_to_model = cached_transform.Invert();
        cached_normal_matrix = cached_world_to_model.Transpose();
        transform_dirty = false;
    }
}

SceneNode *SceneTree::GetNode(Shape *shape, Material *material)
{
    auto it = std::find_if(
        nodes.begin(), nodes.end(),
        [shape, material](auto &node)
        { return node.GetShape() == shape && node.GetMaterial() == material; });
    if (it == nodes.end())
    {
        return nullptr;
    }
    else
    {
        return &*it;
    }
}
}  // namespace raytracer
