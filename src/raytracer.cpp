#include "raytracer.hpp"

#include <numbers>
#include <iostream>
#include <thread>

#include "color.hpp"
#include "material.hpp"
#include "light.hpp"
#include "scene_object.hpp"
#include "scene_tree.hpp"

namespace raytracer
{
    Raytracer::Raytracer(int width, int height)
        : width(width),
          height(height),
          pixel_data(4*width*height, 0)
    {}

    Raytracer::~Raytracer() {}

    void Raytracer::StartTrace()
    {
        tracing_thread = std::thread(&Raytracer::ThreadTraceScene, this);
    }

    void Raytracer::StopTrace()
    {
        if (tracing_thread.joinable())
        {
            tracing_thread.join();
        }
    }

    const std::vector<uint8_t> &Raytracer::GetPixels()
    {
        return pixel_data;
    }

    void Raytracer::ThreadTraceScene()
    {
        Point3<double> cam_pos(0, 0, 0);

        // The scene is just a sphere at 0, 0, -5
        Sphere sphere;
        BlinnPhongMaterial<double> material1(Color<double>{0.25, 0.25, 1.0});
        BlinnPhongMaterial<double> material2(Color<double>{1.0, 0.25, 0.25});
        BlinnPhongMaterial<double> material3(Color<double>{1.0, 1.0, 1.0});
        //NormalMaterial<double> material;
        //PositionMaterial<double> material;
        SceneObject<double> sphere1(&sphere, &material1);
        SceneObject<double> sphere2(&sphere, &material2);
        SceneObject<double> sphere3(&sphere, &material3);
        SceneTree<double> scene_tree;
        {
            SceneNode<double> &node = scene_tree.AddNode(&sphere1);
            node.SetTranslation(0.0, 0.0, -5.0);
            node.SetScale(2.0);
        }
        {
            SceneNode<double> &node = scene_tree.AddNode(&sphere2);
            node.SetTranslation(4.0, 3.0, -5.0);
            node.SetScale(1.0);
        }
        {
            SceneNode<double> &node = scene_tree.AddNode(&sphere3);
            node.SetTranslation(7.0, 5.0, -10.0);
            node.SetScale(4.0);
        }
        std::vector<Light<double>> lights = {
            Light<double>(
                Point3<double>(3.0, 3.0, 3.0),
                40.0,
                Color<double>{1.0, 1.0, 1.0},
                40.0,
                Color<double>{1.0, 1.0, 1.0})};

        int counter = 0;
        for (int dx = 0; dx < width; ++dx)
        {
            for (int dy = 0; dy < height; ++dy)
            {
                // from https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-generating-camera-rays/generating-camera-rays
                double pixel_x_ndc = ((double)dx + 0.5) / (double)width;
                double pixel_y_ndc = ((double)dy + 0.5) / (double)height;

                double pixel_screen_x = 2.0 * pixel_x_ndc - 1.0;
                double pixel_screen_y = 2.0 * pixel_y_ndc - 1.0;

                double aspect_ratio = (double)width / (double)height;
                double fovx = 90.0;
                double fovy = 90.0;

                constexpr double DEGREES_TO_RADIANS = std::numbers::pi / 180.0;
                double pixel_camera_x = pixel_screen_x * aspect_ratio * std::tan(fovx / 2.0 * DEGREES_TO_RADIANS);
                double pixel_camera_y = pixel_screen_y * std::tan(fovy / 2.0 * DEGREES_TO_RADIANS);
                Vec3 pixel_camera_space(pixel_camera_x, pixel_camera_y, -1.0);
                auto dir = pixel_camera_space.ToUnit();

                auto r = Ray(cam_pos, dir);

                IntersectionRecord<double> record;

                // TODO find a better way to save the hit node
                const SceneNode<double>* hit_node = nullptr;

                for (auto &n : scene_tree)
                {
                    if (n.Intersect(&r, record.t, record))
                    {
                        hit_node = &n;
                    }
                }

                auto pixel_start_index = 4 * width * (height - dy) + 4 * dx;
                if (hit_node != nullptr)
                {
                    auto c = hit_node->Shade(record, lights);
                    // Format is RGBA8888 which is RRGGBBAA
                    pixel_data[pixel_start_index] = c.r * 255;
                    pixel_data[pixel_start_index + 1] = c.g * 255;
                    pixel_data[pixel_start_index + 2] = c.b * 255;
                    pixel_data[pixel_start_index + 3] = 255;
                    counter++;
                }
                else
                {
                    pixel_data[pixel_start_index] = 0;
                    pixel_data[pixel_start_index + 1] = 0;
                    pixel_data[pixel_start_index + 2] = 0;
                    pixel_data[pixel_start_index + 3] = 255;
                }
            }
        }

        std::cout << "Image done, " << counter << " hits\n";
    }
}
