#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include "scene.hpp"

namespace raytracer
{
class Raytracer
{
   public:
    Raytracer(int width, int height, Scene &scene);
    ~Raytracer();

    void StartTrace();
    bool IsTraceDone();
    void StopTrace();

    const std::vector<uint8_t> &GetPixels();

    bool CheckIntersection(Point3f origin, Vec3f direction) const;

   private:
    constexpr static float BIAS = 0.01f;

    int width;
    int height;
    Scene &scene;
    bool running = false;

    void ThreadTraceScene(int thread_index, int start_x, int start_y, int width,
                          int height, int full_width, int full_height);

    Color TraceRay(Ray r, float min_distance, size_t rays_remaining);

    std::vector<uint8_t> pixel_data;
    // TODO variable amount of threads
    std::array<std::thread, 16> tracing_threads;
    std::array<std::atomic<bool>, 16> thread_status;
};
}  // namespace raytracer
