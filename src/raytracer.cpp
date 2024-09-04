#include "raytracer.hpp"

#include <algorithm>
#include <iostream>
#include <numbers>
#include <thread>

#include "color.hpp"
#include "light.hpp"
#include "math.hpp"
#include "scene_tree.hpp"

static void TraceThreadFunction() {}

namespace raytracer
{
Raytracer::Raytracer(int width, int height, Scene &scene)
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

bool Raytracer::CheckIntersection(Point3f origin, Vec3f direction) const
{
    Ray r(origin, direction);
    IntersectionRecord record;

    for (auto &n : scene.objects)
    {
        if (n.Intersect(&r, BIAS, record.t, record))
        {
            return true;
        }
    }

    return false;
}

void Raytracer::ThreadTraceScene(int thread_index, int start_x, int start_y,
                                 int width, int height, int full_width,
                                 int full_height)
{
    // std::cout << "Tracing thread started, id = " << thread_index
    //           << ", start_x = " << start_x << ", start_y = " << start_y <<
    //           '\n';
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
            float pixel_x_ndc = ((float)dx + 0.5f) / (float)full_width;
            float pixel_y_ndc = ((float)dy + 0.5f) / (float)full_height;

            float pixel_screen_x = 2.0f * pixel_x_ndc - 1.0f;
            float pixel_screen_y = 2.0f * pixel_y_ndc - 1.0f;

            float aspect_ratio = (float)full_width / (float)full_height;
            float fovx = 90.0f;
            float fovy = 90.0f;

            constexpr float DEGREES_TO_RADIANS =
                std::numbers::pi_v<float> / 180.0f;
            float pixel_camera_x = pixel_screen_x * aspect_ratio *
                                   std::tan(fovx / 2.0f * DEGREES_TO_RADIANS);
            float pixel_camera_y =
                pixel_screen_y * std::tan(fovy / 2.0f * DEGREES_TO_RADIANS);
            Point3f pixel_camera_space(pixel_camera_x, pixel_camera_y, -1.0f);
            auto pixel_world_space = camera_transform * pixel_camera_space;

            auto r =
                Ray(scene.camera.GetPosition(),
                    (pixel_world_space - scene.camera.GetPosition()).ToUnit());

            constexpr size_t MAX_RAYS_TO_TRACE = 10;
            auto c = TraceRay(r, 0.0f, MAX_RAYS_TO_TRACE);

            auto pixel_start_index =
                4 * full_width * (full_height - dy) + 4 * dx;
            // Format is RGBA8888 which is RRGGBBAA
            c = c.SaturateColor();
            pixel_data[pixel_start_index] = c.r * 255;
            pixel_data[pixel_start_index + 1] = c.g * 255;
            pixel_data[pixel_start_index + 2] = c.b * 255;
            pixel_data[pixel_start_index + 3] = 255;
        }
    }
    // std::cout << "Tracing thread " << thread_index << " done\n";
    thread_status[thread_index].store(true);
}

Color Raytracer::TraceRay(Ray r, float min_distance, size_t rays_remaining)
{
    if (rays_remaining <= 0)
    {
        return Color();
    }
    IntersectionRecord record;

    // TODO find a better way to save the hit node
    const SceneNode *hit_node = nullptr;

    for (auto &n : scene.objects)
    {
        if (n.Intersect(&r, min_distance, record.t, record))
        {
            hit_node = &n;
        }
    }

    if (hit_node != nullptr)
    {
        auto c = hit_node->Shade(record, scene.lights, this);

        FresnelTerms terms =
            hit_node->GetFresnelTerms(record.ray->direction, record.normal);
        bool outside = Dot(record.ray->direction, record.normal) < 0;
        Vec3f bias = BIAS * record.normal;
        if (terms.reflective != 0.0f)
        {
            auto reflection_dir = Reflect(record.ray->direction, record.normal);
            Point3f new_origin =
                outside ? record.position + bias : record.position - bias;
            auto reflection_ray = Ray(new_origin, reflection_dir);

            c += terms.reflective *
                 TraceRay(reflection_ray, 0.0f, rays_remaining - 1);
        }
        if (terms.refractive != 0.0f)
        {
            auto refraction_dir = Refract(record.ray->direction, record.normal,
                                          hit_node->GetRefractiveIndex());
            Point3f new_origin =
                outside ? record.position - bias : record.position + bias;
            auto refraction_ray = Ray(new_origin, refraction_dir);

            c += terms.refractive *
                 TraceRay(refraction_ray, 0.0f, rays_remaining - 1);
        }

        return c;
    }
    else
    {
        return scene.clear_color;
    }
}
}  // namespace raytracer
