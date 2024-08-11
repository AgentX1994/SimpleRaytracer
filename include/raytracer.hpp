#pragma once

#include <memory>
#include <thread>
#include <vector>

#include "scene.hpp"

namespace raytracer
{
class Raytracer
{
   public:
    Raytracer(int width, int height, Scene<double> scene);
    ~Raytracer();

    void StartTrace();
    void StopTrace();

    const std::vector<uint8_t> &GetPixels();

   private:
    int width;
    int height;
    Scene<double> scene;
    bool running = false;

    void ThreadTraceScene(int start_x, int start_y, int width, int height,
                          int full_width, int full_height);

    std::vector<uint8_t> pixel_data;
    std::vector<std::thread> tracing_threads;
};
}  // namespace raytracer
