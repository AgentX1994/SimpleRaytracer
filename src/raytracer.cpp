#include "raytracer.hpp"

#include <numbers>
#include <iostream>

namespace raytracer
{
    Raytracer::Raytracer(int width, int height) : width(width), height(height) {}
    Raytracer::~Raytracer() {}

    std::vector<uint8_t> Raytracer::TraceScene()
    {
        std::vector<uint8_t> pixel_data(3 * width * height, 0);
        Point3<double> cam_pos(0, 0, 0);

        // The scene is just a sphere at 0, 0, -5
        Sphere sphere(Point3<double>(0.0, 0.0, -5.0), 2.0);

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

                // println!("Now rendering screen coords ({}, {})", pixel_screen_x, pixel_screen_y);
                double fovx = 90.0;
                double fovy = 90.0;

                constexpr double DEGREES_TO_RADIANS = std::numbers::pi / 180.0;
                double aspect_ratio = (double)width / (double)height;
                double pixel_camera_x = pixel_screen_x * aspect_ratio * std::tan(fovx / 2.0 * DEGREES_TO_RADIANS);
                double pixel_camera_y = pixel_screen_y * std::tan(fovy / 2.0 * DEGREES_TO_RADIANS);
                Vec3 pixel_camera_space(pixel_camera_x, pixel_camera_y, -1.0);
                auto dir = pixel_camera_space.ToUnit();

                // std::cout << "Dir is " << dir.x() << ", " << dir.y() << ", " << dir.z() << '\n';

                auto r = Ray(cam_pos, dir);

                IntersectionRecord<double> record;

                auto pixel_start_index = 3 * width * dy + 3 * dx;

                if (sphere.intersect(&r, std::numeric_limits<double>::infinity(), record))
                {
                    // std::cout << "HIT!\n";
                    pixel_data[pixel_start_index] = 255;
                    pixel_data[pixel_start_index + 1] = 255;
                    pixel_data[pixel_start_index + 2] = 255;
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