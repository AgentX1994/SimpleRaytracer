#include "raytracer.hpp"

#include <algorithm>
#include <iostream>
#include <numbers>
#include <thread>

#include "color.hpp"
#include "light.hpp"
#include "scene_tree.hpp"

static void TraceThreadFunction() {}

namespace raytracer
{
Raytracer::Raytracer(int width, int height, Scene<double> &scene)
    : width(width),
      height(height),
      scene(scene),
      pixel_data(4 * width * height, 0)
{
}

Raytracer::~Raytracer() {}

void Raytracer::StartTrace()
{
    if (running == true)
    {
        StopTrace();
    }
    running = true;
    int cell_width = width / 4;
    int cell_height = height / 4;
    for (size_t i = 0; i < 16; i++)
    {
        int start_x = (i * cell_width) % width;
        int start_y = (i / 4) * cell_height;
        thread_status[i].store(false);
        tracing_threads[i] =
            std::thread(&Raytracer::ThreadTraceScene, this, i, start_x, start_y,
                        cell_width, cell_height, width, height);
    }
}

bool Raytracer::IsTraceDone()
{
    return std::all_of(thread_status.begin(), thread_status.end(),
                       [](auto &b) { return b.load(); });
}

void Raytracer::StopTrace()
{
    running = false;
    for (auto &t : tracing_threads)
    {
        if (t.joinable())
        {
            t.join();
        }
    }
}

const std::vector<uint8_t> &Raytracer::GetPixels() { return pixel_data; }

void Raytracer::ThreadTraceScene(int thread_index, int start_x, int start_y,
                                 int width, int height, int full_width,
                                 int full_height)
{
    std::cout << "Tracing thread started, id = " << thread_index
              << ", start_x = " << start_x << ", start_y = " << start_y << '\n';
    auto camera_transform = scene.camera.GetTransform();
    for (int dx = start_x; dx < start_x + width; ++dx)
    {
        if (!running)
        {
            break;
        }
        for (int dy = start_y; dy < start_y + height; ++dy)
        {
            if (!running)
            {
                break;
            }
            // from
            // https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-generating-camera-rays/generating-camera-rays
            double pixel_x_ndc = ((double)dx + 0.5) / (double)full_width;
            double pixel_y_ndc = ((double)dy + 0.5) / (double)full_height;

            double pixel_screen_x = 2.0 * pixel_x_ndc - 1.0;
            double pixel_screen_y = 2.0 * pixel_y_ndc - 1.0;

            double aspect_ratio = (double)full_width / (double)full_height;
            double fovx = 90.0;
            double fovy = 90.0;

            constexpr double DEGREES_TO_RADIANS = std::numbers::pi / 180.0;
            double pixel_camera_x = pixel_screen_x * aspect_ratio *
                                    std::tan(fovx / 2.0 * DEGREES_TO_RADIANS);
            double pixel_camera_y =
                pixel_screen_y * std::tan(fovy / 2.0 * DEGREES_TO_RADIANS);
            Vec3 pixel_camera_space(pixel_camera_x, pixel_camera_y, -1.0);
            auto pixel_world_space =
                camera_transform.TransformPoint(pixel_camera_space);

            auto r =
                Ray(scene.camera.GetPosition(),
                    (pixel_world_space - scene.camera.GetPosition()).ToUnit());

            auto c = TraceRay(r, 0.0);

            auto pixel_start_index =
                4 * full_width * (full_height - dy) + 4 * dx;
            // Format is RGBA8888 which is RRGGBBAA
            pixel_data[pixel_start_index] = c.r * 255;
            pixel_data[pixel_start_index + 1] = c.g * 255;
            pixel_data[pixel_start_index + 2] = c.b * 255;
            pixel_data[pixel_start_index + 3] = 255;
        }
    }
    std::cout << "Tracing thread " << thread_index << " done\n";
    thread_status[thread_index].store(true);
}

Color<double> Raytracer::TraceRay(Ray<double> r, double min_distance)
{
    IntersectionRecord<double> record;

    // TODO find a better way to save the hit node
    const SceneNode<double> *hit_node = nullptr;

    for (auto &n : scene.objects)
    {
        if (n.Intersect(&r, min_distance, record.t, record))
        {
            hit_node = &n;
        }
    }

    if (hit_node != nullptr)
    {
        auto c = hit_node->Shade(record, scene.lights);

        if (hit_node->IsReflective())
        {
            auto reflection_dir = Reflect(r.direction, record.normal);
            auto reflection_ray = Ray<double>(record.position, reflection_dir);

            c += TraceRay(reflection_ray, 0.1);
        }

        return c;
    }
    else
    {
        return Color<double>{0.0, 0.0, 0.0};
    }
}
}  // namespace raytracer
