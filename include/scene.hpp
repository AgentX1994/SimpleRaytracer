#pragma once

#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "camera.hpp"
#include "material.hpp"
#include "scene_tree.hpp"

namespace raytracer
{
using ShapeMap = std::unordered_map<std::string, std::shared_ptr<Shape>>;

using MaterialMap = std::unordered_map<std::string, std::shared_ptr<Material>>;

using NodeVector = std::vector<SceneNode>;

using LightVector = std::vector<Light>;

class Scene
{
   public:
    static Scene LoadFromJson(const std::string& filepath);

    Camera camera;
    Color clear_color;
    ShapeMap shapes;
    MaterialMap materials;
    NodeVector objects;
    LightVector lights;
};
}  // namespace raytracer
