#pragma once

#include <memory>
#include <thread>
#include <vector>

#include "material.hpp"
#include "scene_object.hpp"
#include "scene_tree.hpp"

namespace raytracer
{
class Raytracer
{
   public:
    Raytracer(int width, int height);
    ~Raytracer();

    void StartTrace();
    void StopTrace();

    const std::vector<uint8_t> &GetPixels();

   private:
    int width;
    int height;
    bool running = false;

    void ThreadTraceScene(int start_x, int start_y, int width, int height,
                          int full_width, int full_height);

    std::vector<uint8_t> pixel_data;
    std::vector<std::thread> tracing_threads;
    Point3<double> camera_position;
    std::vector<std::unique_ptr<Shape>> shapes;
    std::vector<std::unique_ptr<Material<double>>> materials;
    std::vector<SceneObject<double>> objects;
    SceneTree<double> scene_tree;
    std::vector<Light<double>> lights;
};
}  // namespace raytracer
