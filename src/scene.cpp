#include "scene.hpp"

#include <stdexcept>

#include "math.hpp"

namespace raytracer
{
std::shared_ptr<Shape> ReadShape(const nlohmann::json& shape_obj)
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
    else if (type == "triangle")
    {
        auto a = shape_obj["a"];
        auto b = shape_obj["b"];
        auto c = shape_obj["c"];
        if (!a.is_array() || a.size() != 3)
        {
            throw std::runtime_error(
                "Field \"a\" of triangle shape should be an array of size 3");
        }
        if (!b.is_array() || b.size() != 3)
        {
            throw std::runtime_error(
                "Field \"b\" of triangle shape should be an array of size 3");
        }
        if (!c.is_array() || c.size() != 3)
        {
            throw std::runtime_error(
                "Field \"c\" of triangle shape should be an array of size 3");
        }
        return std::make_shared<Triangle>(Point3f(a[0], a[1], a[2]),
                                          Point3f(b[0], b[1], b[2]),
                                          Point3f(c[0], c[1], c[2]));
    }
    else
    {
        throw std::runtime_error("Unknown shape type!");
    }
}

std::shared_ptr<Material> ReadMaterial(const nlohmann::json& mat_obj)
{
    auto type = mat_obj["type"];
    if (type == "blinnphong")
    {
        auto& base_obj = mat_obj["base"];
        auto reflectivity = mat_obj.value("reflectivity", 0.0f);
        auto transmissibility = mat_obj.value("transmissibility", 0.0f);
        Color base = {base_obj[0], base_obj[1], base_obj[2]};
        return std::make_shared<BlinnPhongMaterial>(base, reflectivity,
                                                    transmissibility);
    }
    else if (type == "normal")
    {
        return std::make_shared<NormalMaterial>();
    }
    else if (type == "position")
    {
        return std::make_shared<PositionMaterial>();
    }
    else if (type == "glass")
    {
        auto refractive_index = mat_obj.value("refractive_index", 1.5f);
        return std::make_shared<GlassMaterial>(refractive_index);
    }
    else if (type == "uv")
    {
        return std::make_shared<UVMaterial>();
    }
    else
    {
        throw std::runtime_error("Unknown material type!");
    }
}

SceneNode ReadSceneObject(const ShapeMap& shapes, const MaterialMap& materials,
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
        position = {0.0f, 0.0f, 0.0f};
    }
    if (rotation.is_null())
    {
        rotation = {0.0f, 0.0f, 0.0f};
    }
    if (scale.is_null())
    {
        scale = {1.0f, 1.0f, 1.0f};
    }

    SceneNode node(shapes.at(shape_name).get(),
                   materials.at(material_name).get());
    node.SetTranslation(Point3f(position[0], position[1], position[2]));
    node.SetRotation(Vec3f(rotation[0], rotation[1], rotation[2]));
    node.SetScale(Vec3f(scale[0], scale[1], scale[2]));

    return node;
}

Light ReadLight(const nlohmann::json& light_obj)
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

    return Light(Point3f(position[0], position[1], position[2]), diffuse_power,
                 Color{diffuse[0], diffuse[1], diffuse[2]}, specular_power,
                 Color{specular[0], specular[1], specular[2]});
}

Scene Scene::LoadFromJson(const std::string& filepath)
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
    Scene scene;
    auto& camera_obj = scene_obj["camera"];
    auto& camera_pos = camera_obj["position"];
    scene.camera.SetPosition(
        Point3f(camera_pos[0], camera_pos[1], camera_pos[2]));
    auto& camera_forward = camera_obj["forward"];
    scene.camera.SetForward(
        Vec3f(camera_forward[0], camera_forward[1], camera_forward[2]));
    auto& camera_up = camera_obj["up"];
    scene.camera.SetUp(Vec3f(camera_up[0], camera_up[1], camera_up[2]));

    if (scene_obj.contains("clear_color"))
    {
        auto& clear_color = scene_obj["clear_color"];
        scene.clear_color =
            Color{clear_color[0], clear_color[1], clear_color[2]};
    }

    for (auto& shape_obj : scene_obj["shapes"])
    {
        scene.shapes[shape_obj["name"]] = ReadShape(shape_obj);
    }

    for (auto& mat_obj : scene_obj["materials"])
    {
        auto& name = mat_obj["name"];
        scene.materials[name] = ReadMaterial(mat_obj);
    }

    for (auto& obj_obj : scene_obj["objects"])
    {
        scene.objects.emplace_back(
            ReadSceneObject(scene.shapes, scene.materials, obj_obj));
    }

    for (auto& light_obj : scene_obj["lights"])
    {
        scene.lights.emplace_back(ReadLight(light_obj));
    }

    return scene;
}
}  // namespace raytracer
