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
    auto scene = Scene::LoadFromJson(file_name);
    Raytracer rt(WIDTH, HEIGHT, scene);
    // Raytrace
    rt.StartTrace();

    SdlWindow window(WIDTH, HEIGHT);
    auto tex = window.MakeTexture(WIDTH, HEIGHT);

    float movement_speed = 1.25f;
    float rotation_speed = 0.1f;
    bool speed_up = false;

    while (true)
    {
        Vec3f movement;
        Vec3f rotation;
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
                movement += Vec3f(movement_speed, 0.0f, 0.0f);
            }
            else if (event.key.keysym.sym == SDLK_d)
            {
                movement += Vec3f(-movement_speed, 0.0f, 0.0f);
            }
            else if (event.key.keysym.sym == SDLK_w)
            {
                movement += Vec3f(0.0f, 0.0f, movement_speed);
            }
            else if (event.key.keysym.sym == SDLK_s)
            {
                movement += Vec3f(0.0f, 0.0f, -movement_speed);
            }
            else if (event.key.keysym.sym == SDLK_SPACE)
            {
                movement += Vec3f(0.0f, movement_speed, 0.0f);
            }
            else if (event.key.keysym.sym == SDLK_c)
            {
                movement += Vec3f(0.0f, -movement_speed, 0.0f);
            }
            else if (event.key.keysym.sym == SDLK_q)
            {
                rotation += Vec3f(0.0f, rotation_speed, 0.0f);
            }
            else if (event.key.keysym.sym == SDLK_e)
            {
                rotation += Vec3f(0.0f, -rotation_speed, 0.0f);
            }
            else if (event.key.keysym.sym == SDLK_r)
            {
                // This rotation should be about the camera's right axis
                auto right = scene.camera.GetRight();
                auto to_rotate = AxisAngleToEuler(right, -rotation_speed);
                std::cout << "Rotating " << -rotation_speed << " radians about "
                          << right << ": " << to_rotate << '\n';
                rotation += to_rotate;
            }
            else if (event.key.keysym.sym == SDLK_f)
            {
                // This rotation should be about the camera's right axis
                auto right = scene.camera.GetRight();
                auto to_rotate = AxisAngleToEuler(right, rotation_speed);
                std::cout << "Rotating " << rotation_speed << " radians about "
                          << right << ": " << to_rotate << '\n';
                rotation += to_rotate;
            }
            else if (event.key.keysym.sym == SDLK_y)
            {
                if (speed_up)
                {
                    movement_speed /= 5.0f;
                    speed_up = false;
                }
                else
                {
                    movement_speed *= 5.0f;
                    speed_up = true;
                }
            }
        }

        if ((movement.LengthSquared() > 0 || rotation.LengthSquared() > 0.0f) &&
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
