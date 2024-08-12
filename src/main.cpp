#include <iostream>
#include <vector>

#include "raytracer.hpp"
#include "scene.hpp"
#include "sdl_window.hpp"

using namespace raytracer;

constexpr int WIDTH = 800;
constexpr int HEIGHT = 600;

int main(int argc, char **argv)
{
    std::string file_name = "../scenes/default.json";
    if (argc == 2)
    {
        file_name = argv[1];
    }
    auto scene = Scene<double>::LoadFromJson(file_name);
    Raytracer rt(WIDTH, HEIGHT, scene);
    // Raytrace
    rt.StartTrace();

    SdlWindow window(WIDTH, HEIGHT);
    auto tex = window.MakeTexture(WIDTH, HEIGHT);

    double movement_speed = 0.25;
    double rotation_speed = 0.1;

    while (true)
    {
        Vec3<double> movement;
        Vec3<double> rotation;
        auto event = window.PollEvent();
        if (event.type == SDL_QUIT)
        {
            break;
        }
        else if (event.type == SDL_KEYDOWN)
        {
            if (event.key.keysym.sym == SDLK_ESCAPE)
            {
                break;
            }
            else if (event.key.keysym.sym == SDLK_a)
            {
                movement = movement + Vec3<double>(movement_speed, 0.0, 0.0);
            }
            else if (event.key.keysym.sym == SDLK_d)
            {
                movement = movement + Vec3<double>(-movement_speed, 0.0, 0.0);
            }
            else if (event.key.keysym.sym == SDLK_w)
            {
                movement = movement + Vec3<double>(0.0, 0.0, movement_speed);
            }
            else if (event.key.keysym.sym == SDLK_s)
            {
                movement = movement + Vec3<double>(0.0, 0.0, -movement_speed);
            }
            else if (event.key.keysym.sym == SDLK_SPACE)
            {
                movement = movement + Vec3<double>(0.0, movement_speed, 0.0);
            }
            else if (event.key.keysym.sym == SDLK_c)
            {
                movement = movement + Vec3<double>(0.0, -movement_speed, 0.0);
            }
            else if (event.key.keysym.sym == SDLK_q)
            {
                rotation = rotation + Vec3<double>(0.0, rotation_speed, 0.0);
            }
            else if (event.key.keysym.sym == SDLK_e)
            {
                rotation = rotation + Vec3<double>(0.0, -rotation_speed, 0.0);
            }
            else if (event.key.keysym.sym == SDLK_r)
            {
                rotation = rotation + Vec3<double>(rotation_speed, 0.0, 0.0);
            }
            else if (event.key.keysym.sym == SDLK_f)
            {
                rotation = rotation + Vec3<double>(-rotation_speed, 0.0, 0.0);
            }
        }

        if ((movement.LengthSquared() > 0 || rotation.LengthSquared() > 0.0) &&
            rt.IsTraceDone())
        {
            scene.camera.Move(movement);
            scene.camera.Rotate(rotation);
            rt.StartTrace();
        }

        window.UpdateTexture(tex, WIDTH, HEIGHT, rt.GetPixels());

        window.Clear();
        window.DrawTexture(tex);
        window.Present();
    }

    rt.StopTrace();

    window.DestroyTexture(tex);
}
