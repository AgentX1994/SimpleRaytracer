#include "raytracer.hpp"

#include <numbers>
#include <iostream>

#include "color.hpp"
#include "material.hpp"
#include "light.hpp"
#include "scene_object.hpp"

namespace raytracer
{
    Raytracer::Raytracer(int width, int height) : width(width), height(height) {}
    Raytracer::~Raytracer() {}

    std::vector<uint8_t> Raytracer::TraceScene()
    {
        std::vector<uint8_t> pixel_data(4 * width * height, 0);
        Point3<double> cam_pos(0, 0, 0);

        // The scene is just a sphere at 0, 0, -5
        Sphere sphere(Point3<double>(0.0, 0.0, -5.0), 2.0);
        BlinnPhongMaterial<double> material(Color<double>{0.25, 0.25, 1.0});
        //NormalMaterial<double> material;
        //PositionMaterial<double> material;
        SceneObject<double> sphere_object(&sphere, &material);
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

                auto pixel_start_index = 4 * width * (height - dy) + 4 * dx;

                if (sphere_object.Intersect(&r, std::numeric_limits<double>::infinity(), record))
                {
                    auto c = sphere_object.Shade(record, lights);
                    if (c.r < 0.1 && c.g < 0.1 && c.b < 0.1)
                    {
                        std::cout << "Hit without color at pixel " << dx << ", " << dy << '\n';
                    }
                    //std::cout << "HIT! Record = " << record << " Color = " << c << '\n';
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
                }
            }
        }

        std::cout << "Image done, " << counter << " hits\n";

        return pixel_data;
    }
}
