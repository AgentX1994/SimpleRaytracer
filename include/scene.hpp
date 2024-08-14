#pragma once

#include <concepts>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "camera.hpp"
#include "material.hpp"
#include "math.hpp"
#include "scene_tree.hpp"

namespace raytracer
{
using ShapeMap = std::unordered_map<std::string, std::shared_ptr<Shape>>;

template <std::floating_point T>
using MaterialMap =
    std::unordered_map<std::string, std::shared_ptr<Material<T>>>;

template <std::floating_point T>
using NodeVector = std::vector<SceneNode<T>>;

template <std::floating_point T>
using LightVector = std::vector<Light<T>>;

static std::shared_ptr<Shape> ReadShape(const nlohmann::json& shape_obj)
{
    auto& type = shape_obj["type"];
    if (type == "sphere")
    {
        return std::make_shared<Sphere>();
    }
    else if (type == "plane")
    {
        return std::make_shared<Plane>();
    }
    else if (type == "disc")
    {
        return std::make_shared<Disc>();
    }
    else
    {
        throw std::runtime_error("Unknown shape type!");
    }
}

template <std::floating_point T>
std::shared_ptr<Material<T>> ReadMaterial(const nlohmann::json& mat_obj)
{
    auto type = mat_obj["type"];
    if (type == "blinnphong")
    {
        auto& base_obj = mat_obj["base"];
        auto reflectivity = mat_obj.value("reflectivity", (T)0.0);
        auto transmissibility = mat_obj.value("transmissibility", (T)0.0);
        Color<T> base = {base_obj[0], base_obj[1], base_obj[2]};
        return std::make_shared<BlinnPhongMaterial<T>>(base, reflectivity,
                                                       transmissibility);
    }
    else if (type == "normal")
    {
        return std::make_shared<NormalMaterial<T>>();
    }
    else if (type == "position")
    {
        return std::make_shared<PositionMaterial<T>>();
    }
    else if (type == "glass")
    {
        auto refractive_index = mat_obj.value("refractive_index", 1.5);
        return std::make_shared<GlassMaterial<T>>(refractive_index);
    }
    else
    {
        throw std::runtime_error("Unknown material type!");
    }
}

template <std::floating_point T>
SceneNode<T> ReadSceneObject(const ShapeMap& shapes,
                             const MaterialMap<T>& materials,
                             const nlohmann::json& object_obj)
{
    auto& shape_name = object_obj["shape"];
    auto& material_name = object_obj["material"];
    auto position = object_obj["position"];
    auto rotation = object_obj["rotation"];
    auto scale = object_obj["scale"];

    if (!shapes.contains(shape_name))
    {
        throw std::runtime_error("Encountered unknown shape name!");
    }
    if (!materials.contains(material_name))
    {
        throw std::runtime_error("Encountered unknown material name!");
    }
    if (position.is_null())
    {
        position = {0.0, 0.0, 0.0};
    }
    if (rotation.is_null())
    {
        rotation = {0.0, 0.0, 0.0};
    }
    if (scale.is_null())
    {
        scale = {1.0, 1.0, 1.0};
    }

    SceneNode<T> node(shapes.at(shape_name).get(),
                      materials.at(material_name).get());
    node.SetTranslation(Point3<T>(position[0], position[1], position[2]));
    node.SetRotation(Vec3<T>(rotation[0], rotation[1], rotation[2]));
    node.SetScale(Vec3<T>(scale[0], scale[1], scale[2]));

    return node;
}

template <std::floating_point T>
Light<T> ReadLight(const nlohmann::json& light_obj)
{
    if (light_obj["type"] != "point")
    {
        throw std::runtime_error("Unknown light type!");
    }
    auto& position = light_obj["position"];
    auto& diffuse_power = light_obj["diffuse_power"];
    auto& diffuse = light_obj["diffuse"];
    auto& specular_power = light_obj["specular_power"];
    auto& specular = light_obj["specular"];

    return Light<T>(Point3<T>(position[0], position[1], position[2]),
                    diffuse_power, Color<T>{diffuse[0], diffuse[1], diffuse[2]},
                    specular_power,
                    Color<T>{specular[0], specular[1], specular[2]});
}

template <std::floating_point T>
class Scene
{
   public:
    static Scene<T> LoadFromJson(const std::string& filepath)
    {
        std::ifstream f(filepath);
        auto scene_obj = nlohmann::json::parse(f);

        // Thoughts on file content:
        //
        // {
        //     "camera": { camera details }
        //     "materials": [
        //         {
        //             "type": <one of the types of materials>,
        //             "name": <name>
        //             ... // other details depend on type
        //         }
        //     ],
        //     "shapes": [
        //         {
        //             "type": <one of the types of shapes>,
        //             "name": <name>
        //             ...
        //         }
        //     ],
        //     "objects": [
        //         {
        //             "shape": <name from shapes>,
        //             "position": [1.0, 2.0, 3.0], // x, y, z, default to
        //             origin "rotation": [0.0, 0.0, 0.0], // x, y, z, default
        //             to 0s "scale": [1.0, 1.0, 1.0],    // x, y, z, default to
        //             1s "material": <name from materails>
        //         },
        //         ...
        //     ],
        //     "lights": [
        //         {
        //             "type": "point", // more to come
        //             "position": [x, y, z],
        //             "diffuse_power": power,
        //             "diffuse": [r, g, b],
        //             "specular_power": power,
        //             "specular": [r, g, b],
        //         }
        //     ]
        // }

        if (!scene_obj.is_object())
        {
            throw std::runtime_error("Json file is not an object!");
        }

        // Handle camera
        Scene<T> scene;
        auto& camera_obj = scene_obj["camera"];
        auto& camera_pos = camera_obj["position"];
        scene.camera.SetPosition(
            Point3<T>(camera_pos[0], camera_pos[1], camera_pos[2]));
        auto& camera_forward = camera_obj["forward"];
        scene.camera.SetForward(
            Vec3<T>(camera_forward[0], camera_forward[1], camera_forward[2]));
        auto& camera_up = camera_obj["up"];
        scene.camera.SetUp(Vec3<T>(camera_up[0], camera_up[1], camera_up[2]));

        if (scene_obj.contains("clear_color"))
        {
            auto& clear_color = scene_obj["clear_color"];
            scene.clear_color =
                Color<T>{clear_color[0], clear_color[1], clear_color[2]};
        }

        for (auto& shape_obj : scene_obj["shapes"])
        {
            scene.shapes[shape_obj["name"]] = ReadShape(shape_obj);
        }

        for (auto& mat_obj : scene_obj["materials"])
        {
            auto& name = mat_obj["name"];
            scene.materials[name] = ReadMaterial<T>(mat_obj);
        }

        for (auto& obj_obj : scene_obj["objects"])
        {
            scene.objects.emplace_back(
                ReadSceneObject(scene.shapes, scene.materials, obj_obj));
        }

        for (auto& light_obj : scene_obj["lights"])
        {
            scene.lights.emplace_back(ReadLight<T>(light_obj));
        }

        return scene;
    }

    Camera<T> camera;
    Color<T> clear_color;
    ShapeMap shapes;
    MaterialMap<T> materials;
    NodeVector<T> objects;
    LightVector<T> lights;
};
}  // namespace raytracer
